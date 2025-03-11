/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_i2s_sam0

#include <errno.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <soc.h>

#ifndef CONFIG_I2S_SAM0_RX_BLOCK_COUNT
#define CONFIG_I2S_SAM0_RX_BLOCK_COUNT 16
#endif
#ifndef CONFIG_I2S_SAM0_TX_BLOCK_COUNT
#define CONFIG_I2S_SAM0_TX_BLOCK_COUNT 16
#endif

#if __DCACHE_PRESENT == 1
#define DCACHE_INVALIDATE(addr, size) \
	SCB_InvalidateDCache_by_Addr((uint32_t *)addr, size)
#define DCACHE_CLEAN(addr, size) \
	SCB_CleanDCache_by_Addr((uint32_t *)addr, size)
#else
#define DCACHE_INVALIDATE(addr, size) {; }
#define DCACHE_CLEAN(addr, size) {; }
#endif

#define I2S_SAM0_WORD_SIZE_BITS_MIN 2
#define I2S_SAM0_WORD_SIZE_BITS_MAX 32
#define I2S_SAM0_WORD_PER_FRAME_MIN 1
#define I2S_SAM0_WORD_PER_FRAME_MAX 16

struct queue_item
{
	void *mem_block;
	size_t size;
};

/* Minimal ring buffer implementation */
struct ring_buffer
{
	struct queue_item *buf;
	uint16_t len;
	uint16_t head;
	uint16_t tail;
};

/* Device constant configuration parameters */
struct i2s_sam0_dev_cfg
{
	const struct device *dma_dev;
	const struct pinctrl_dev_config *pcfg;
	I2s *regs;
	volatile uint32_t *mclk;
	uint32_t mclk_mask;
	uint32_t gclk_gen;
	uint16_t gclk_id;
	uint32_t freq;
	uint16_t prescaler;

	void (*irq_config)(const struct i2s_sam0_dev_cfg *, const struct device *);
	void (*irq_callback_isr)(const struct device *);
	uint8_t irq_id;
	uint32_t irq_prio;
	bool mck_enable;

	uint8_t tx_dma_request;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_request;
	uint8_t rx_dma_channel;
};

struct stream
{
	int num;
	const struct device *dev;
	int32_t state;
	struct k_sem sem;
	uint32_t dma_channel;
	uint8_t dma_perid;
	uint8_t word_size_bytes;
	bool last_block;
	struct i2s_config cfg;
	struct ring_buffer mem_block_queue;
	void *mem_block;
	int (*stream_start)(struct stream *, I2s *const,
						const struct device *);
	void (*stream_disable)(struct stream *, I2s *const,
						   const struct device *);
	void (*queue_drop)(struct stream *);
	int (*set_data_format)(const struct i2s_sam0_dev_cfg *const,
						   const struct i2s_config *);
};

/* Device run time data */
struct i2s_sam0_dev_data
{
	int num;
	struct stream rx;
	struct stream tx;
	struct device *dev;
	struct i2s_sam0_dev_cfg *cfg;
};

#define MODULO_INC(val, max) { val = (++val < max) ? val : 0; }

static void tx_stream_disable(struct stream *stream, I2s *const i2s,
							  const struct device *dma_dev);
static void rx_stream_disable(struct stream *stream, I2s *const i2s,
							  const struct device *dma_dev);

/*
 * Get data from the queue
 */
static int queue_get(struct ring_buffer *rb, void **mem_block, size_t *size)
{
	unsigned int key;

	key = irq_lock();

	if (rb->tail == rb->head)
	{
		/* Ring buffer is empty */
		irq_unlock(key);
		return -ENOMEM;
	}

	*mem_block = rb->buf[rb->tail].mem_block;
	*size = rb->buf[rb->tail].size;
	MODULO_INC(rb->tail, rb->len);

	irq_unlock(key);

	return 0;
}

/*
 * Put data in the queue
 */
static int queue_put(struct ring_buffer *rb, void *mem_block, size_t size)
{
	uint16_t head_next;
	unsigned int key;

	key = irq_lock();

	head_next = rb->head;
	MODULO_INC(head_next, rb->len);

	if (head_next == rb->tail)
	{
		/* Ring buffer is full */
		irq_unlock(key);
		return -ENOMEM;
	}

	rb->buf[rb->head].mem_block = mem_block;
	rb->buf[rb->head].size = size;
	rb->head = head_next;

	irq_unlock(key);

	return 0;
}

static int reload_dma(const struct device *dev_dma, uint32_t channel,
					  void *src, void *dst, size_t size)
{
	int ret;

	ret = dma_reload(dev_dma, channel, (uint32_t)src, (uint32_t)dst, size);
	if (ret < 0)
	{
		return ret;
	}

	ret = dma_start(dev_dma, channel);

	return ret;
}

static int start_dma(const struct device *dev_dma, uint32_t channel,
					 struct dma_config *cfg, void *src, void *dst,
					 uint32_t blk_size)
{
	struct dma_block_config blk_cfg;
	int ret;

	(void)memset(&blk_cfg, 0, sizeof(blk_cfg));
	blk_cfg.block_size = blk_size;
	blk_cfg.source_address = (uint32_t)src;
	blk_cfg.dest_address = (uint32_t)dst;

	cfg->head_block = &blk_cfg;

	ret = dma_config(dev_dma, channel, cfg);
	if (ret < 0)
	{
		return ret;
	}

	ret = dma_start(dev_dma, channel);

	return ret;
}

/* This function is executed in the interrupt context */
static void dma_rx_callback(const struct device *dma_dev, void *user_data,
							uint32_t channel, int status)
{
	const struct device *dev = (struct device *)user_data;
	const struct i2s_sam0_dev_cfg *const dev_cfg = dev->config;
	struct i2s_sam0_dev_data *const dev_data = dev->data;
	I2s *const i2s = dev_cfg->regs;
	struct stream *stream = &dev_data->rx;
	int ret;

	ARG_UNUSED(user_data);
	__ASSERT_NO_MSG(stream->mem_block != NULL);

	/* Stop reception if there was an error */
	if (stream->state == I2S_STATE_ERROR)
	{
		goto rx_disable;
	}

	/* All block data received */
	ret = queue_put(&stream->mem_block_queue, stream->mem_block,
					stream->cfg.block_size);
	if (ret < 0)
	{
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}
	stream->mem_block = NULL;
	k_sem_give(&stream->sem);

	/* Stop reception if we were requested */
	if (stream->state == I2S_STATE_STOPPING)
	{
		stream->state = I2S_STATE_READY;
		goto rx_disable;
	}

	/* Prepare to receive the next data block */
	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
						   K_NO_WAIT);
	if (ret < 0)
	{
		stream->state = I2S_STATE_ERROR;
		goto rx_disable;
	}

	/* Assure cache coherency before DMA write operation */
	DCACHE_INVALIDATE(stream->mem_block, stream->cfg.block_size);

	ret = reload_dma(dev_cfg->dma_dev, stream->dma_channel,
					 (void *)&i2s->DATA[0], stream->mem_block, 
					 stream->cfg.block_size);
	if (ret < 0)
	{
		goto rx_disable;
	}

	return;

rx_disable:
	rx_stream_disable(stream, i2s, dev_cfg->dma_dev);
}

/* This function is executed in the interrupt context */
static void dma_tx_callback(const struct device *dma_dev, void *user_data,
							uint32_t channel, int status)
{
	const struct device *dev = (struct device *)user_data;
	const struct i2s_sam0_dev_cfg *const dev_cfg = dev->config;
	struct i2s_sam0_dev_data *const dev_data = dev->data;
	I2s *const i2s = dev_cfg->regs;
	struct stream *stream = &dev_data->tx;
	size_t mem_block_size;
	int ret;

	ARG_UNUSED(user_data);
	__ASSERT_NO_MSG(stream->mem_block != NULL);

	/* All block data sent */
	k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
	stream->mem_block = NULL;

	/* Stop transmission if there was an error */
	if (stream->state == I2S_STATE_ERROR)
	{
		goto tx_disable;
	}

	/* Stop transmission if we were requested */
	if (stream->last_block)
	{
		stream->state = I2S_STATE_READY;
		goto tx_disable;
	}

	/* Prepare to send the next data block */
	ret = queue_get(&stream->mem_block_queue, &stream->mem_block,
					&mem_block_size);
	if (ret < 0)
	{
		if (stream->state == I2S_STATE_STOPPING)
		{
			stream->state = I2S_STATE_READY;
		}
		else
		{
			stream->state = I2S_STATE_ERROR;
		}
		goto tx_disable;
	}
	k_sem_give(&stream->sem);

	/* Assure cache coherency before DMA read operation */
	DCACHE_CLEAN(stream->mem_block, mem_block_size);

	ret = reload_dma(dev_cfg->dma_dev, stream->dma_channel,
					 stream->mem_block, (void *)&i2s->DATA[1], 
					 mem_block_size);
	if (ret < 0)
	{
		goto tx_disable;
	}

	return;

tx_disable:
	tx_stream_disable(stream, i2s, dev_cfg->dma_dev);
}

static int set_rx_data_format(const struct i2s_sam0_dev_cfg *const dev_cfg,
							  const struct i2s_config *i2s_cfg)
{
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK)
	{

	case I2S_FMT_DATA_FORMAT_I2S:
		break;
	default:
	}

	return 0;
}

static int set_tx_data_format(const struct i2s_sam0_dev_cfg *const dev_cfg,
							  const struct i2s_config *i2s_cfg)
{
	switch (i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK)
	{

	case I2S_FMT_DATA_FORMAT_I2S:
	default:
	}

	return 0;
}

static uint8_t get_word_size_bytes(uint8_t bit_size)
{
	uint8_t byte_size_min = (bit_size + 7) / 8U;
	uint8_t byte_size;

	byte_size = (byte_size_min == 3U) ? 4 : byte_size_min;

	return byte_size;
}

static const struct i2s_config *i2s_sam0_config_get(const struct device *dev,
													enum i2s_dir dir)
{
	struct i2s_sam0_dev_data *const dev_data = dev->data;
	struct stream *stream;

	if (dir == I2S_DIR_RX)
	{
		stream = &dev_data->rx;
	}
	else
	{
		stream = &dev_data->tx;
	}

	if (stream->state == I2S_STATE_NOT_READY)
	{
		return NULL;
	}

	return &stream->cfg;
}

static int i2s_sam0_configure(const struct device *dev, enum i2s_dir dir,
							  const struct i2s_config *i2s_cfg)
{
	const struct i2s_sam0_dev_cfg *const dev_cfg = dev->config;
	struct i2s_sam0_dev_data *const dev_data = dev->data;
	uint8_t num_words = i2s_cfg->channels;
	uint8_t word_size_bits = i2s_cfg->word_size;
	struct stream *stream;
	int ret;

	if (dir == I2S_DIR_RX)
	{
		stream = &dev_data->rx;
	}
	else if (dir == I2S_DIR_TX)
	{
		stream = &dev_data->tx;
	}
	else if (dir == I2S_DIR_BOTH)
	{
		return -ENOSYS;
	}
	else
	{
		return -EINVAL;
	}

	if (stream->state != I2S_STATE_NOT_READY &&
		stream->state != I2S_STATE_READY)
	{
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0U)
	{
		stream->queue_drop(stream);
		(void)memset(&stream->cfg, 0, sizeof(struct i2s_config));
		stream->state = I2S_STATE_NOT_READY;
		return 0;
	}

	if (i2s_cfg->format & I2S_FMT_FRAME_CLK_INV)
	{
		return -EINVAL;
	}

	if (i2s_cfg->format & I2S_FMT_BIT_CLK_INV)
	{
		return -EINVAL;
	}

	if (word_size_bits < I2S_SAM0_WORD_SIZE_BITS_MIN ||
		word_size_bits > I2S_SAM0_WORD_SIZE_BITS_MAX)
	{
		return -EINVAL;
	}

	if (num_words < I2S_SAM0_WORD_PER_FRAME_MIN ||
		num_words > I2S_SAM0_WORD_PER_FRAME_MAX)
	{
		return -EINVAL;
	}

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));

	ret = stream->set_data_format(dev_cfg, i2s_cfg);
	if (ret < 0)
	{
		return ret;
	}

	/* Set up DMA channel parameters */
	stream->word_size_bytes = get_word_size_bytes(word_size_bits);

	stream->state = I2S_STATE_READY;

	return 0;
}

static int rx_stream_start(struct stream *stream, I2s *const i2s,
						   const struct device *dma_dev)
{
	int ret;

	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
						   K_NO_WAIT);
	if (ret < 0)
	{
		return ret;
	}

	struct dma_config dma_cfg = {
		.source_data_size = stream->word_size_bytes,
		.dest_data_size = stream->word_size_bytes,
		.block_count = 1,
		.dma_slot = stream->dma_perid,
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.user_data = (void *)stream->dev,
		.dma_callback = dma_rx_callback,
	};

	ret = start_dma(dma_dev, stream->dma_channel, &dma_cfg,
					(void *)&i2s->DATA[stream->num], // todo: start dma
					stream->mem_block,
					stream->cfg.block_size);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static int tx_stream_start(struct stream *stream, I2s *const i2s,
						   const struct device *dma_dev)
{
	size_t mem_block_size;
	int ret;

	ret = queue_get(&stream->mem_block_queue, &stream->mem_block,
					&mem_block_size);
	if (ret < 0)
	{
		return ret;
	}
	k_sem_give(&stream->sem);

	struct dma_config dma_cfg = {
		.source_data_size = stream->word_size_bytes,
		.dest_data_size = stream->word_size_bytes,
		.block_count = 1,
		.dma_slot = stream->dma_perid,
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.user_data = (void *)stream->dev,
		.dma_callback = dma_tx_callback,
	};

	/* Assure cache coherency before DMA read operation */
	DCACHE_CLEAN(stream->mem_block, mem_block_size);

	ret = start_dma(dma_dev, stream->dma_channel, &dma_cfg,
					stream->mem_block, (void *)&i2s->DATA[stream->num],
					mem_block_size);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static void rx_stream_disable(struct stream *stream, I2s *const i2s,
							  const struct device *dma_dev)
{
	dma_stop(dma_dev, stream->dma_channel);
	if (stream->mem_block != NULL)
	{
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}
}

static void tx_stream_disable(struct stream *stream, I2s *const i2s,
							  const struct device *dma_dev)
{
	dma_stop(dma_dev, stream->dma_channel);
	if (stream->mem_block != NULL)
	{
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}
}

static void rx_queue_drop(struct stream *stream)
{
	size_t size;
	void *mem_block;

	while (queue_get(&stream->mem_block_queue, &mem_block, &size) == 0)
	{
		k_mem_slab_free(stream->cfg.mem_slab, mem_block);
	}

	k_sem_reset(&stream->sem);
}

static void tx_queue_drop(struct stream *stream)
{
	size_t size;
	void *mem_block;
	unsigned int n = 0U;

	while (queue_get(&stream->mem_block_queue, &mem_block, &size) == 0)
	{
		k_mem_slab_free(stream->cfg.mem_slab, mem_block);
		n++;
	}

	for (; n > 0; n--)
	{
		k_sem_give(&stream->sem);
	}
}

static int i2s_sam0_trigger(const struct device *dev, enum i2s_dir dir,
							enum i2s_trigger_cmd cmd)
{
	const struct i2s_sam0_dev_cfg *const dev_cfg = dev->config;
	struct i2s_sam0_dev_data *const dev_data = dev->data;
	I2s *const i2s = dev_cfg->regs;
	struct stream *stream;
	unsigned int key;
	int ret;

	if (dir == I2S_DIR_RX)
	{
		stream = &dev_data->rx;
	}
	else if (dir == I2S_DIR_TX)
	{
		stream = &dev_data->tx;
	}
	else if (dir == I2S_DIR_BOTH)
	{
		return -ENOSYS;
	}
	else
	{
		return -EINVAL;
	}

	switch (cmd)
	{
	case I2S_TRIGGER_START:
		if (stream->state != I2S_STATE_READY)
		{
			return -EIO;
		}

		__ASSERT_NO_MSG(stream->mem_block == NULL);

		ret = stream->stream_start(stream, i2s, dev_cfg->dma_dev);
		if (ret < 0)
		{
			return ret;
		}

		stream->state = I2S_STATE_RUNNING;
		stream->last_block = false;
		break;

	case I2S_TRIGGER_STOP:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING)
		{
			irq_unlock(key);
			return -EIO;
		}
		stream->state = I2S_STATE_STOPPING;
		irq_unlock(key);
		stream->last_block = true;
		break;

	case I2S_TRIGGER_DRAIN:
		key = irq_lock();
		if (stream->state != I2S_STATE_RUNNING)
		{
			irq_unlock(key);
			return -EIO;
		}
		stream->state = I2S_STATE_STOPPING;
		irq_unlock(key);
		break;

	case I2S_TRIGGER_DROP:
		if (stream->state == I2S_STATE_NOT_READY)
		{
			return -EIO;
		}
		stream->stream_disable(stream, i2s, dev_cfg->dma_dev);
		stream->queue_drop(stream);
		stream->state = I2S_STATE_READY;
		break;

	case I2S_TRIGGER_PREPARE:
		if (stream->state != I2S_STATE_ERROR)
		{
			return -EIO;
		}
		stream->state = I2S_STATE_READY;
		stream->queue_drop(stream);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int i2s_sam0_read(const struct device *dev, void **mem_block,
						 size_t *size)
{
	struct i2s_sam0_dev_data *const dev_data = dev->data;
	int ret;

	if (dev_data->rx.state == I2S_STATE_NOT_READY)
	{
		return -EIO;
	}

	if (dev_data->rx.state != I2S_STATE_ERROR)
	{
		ret = k_sem_take(&dev_data->rx.sem,
						 SYS_TIMEOUT_MS(dev_data->rx.cfg.timeout));
		if (ret < 0)
		{
			return ret;
		}
	}

	/* Get data from the beginning of RX queue */
	ret = queue_get(&dev_data->rx.mem_block_queue, mem_block, size);
	if (ret < 0)
	{
		return -EIO;
	}

	return 0;
}

static int i2s_sam0_write(const struct device *dev, void *mem_block,
						  size_t size)
{
	struct i2s_sam0_dev_data *const dev_data = dev->data;
	int ret;

	if (dev_data->tx.state != I2S_STATE_RUNNING &&
		dev_data->tx.state != I2S_STATE_READY)
	{
		return -EIO;
	}

	ret = k_sem_take(&dev_data->tx.sem,
					 SYS_TIMEOUT_MS(dev_data->tx.cfg.timeout));
	if (ret < 0)
	{
		return ret;
	}

	/* Add data to the end of the TX queue */
	queue_put(&dev_data->tx.mem_block_queue, mem_block, size);

	return 0;
}

static void __maybe_unused i2s_sam0_isr(const struct device *dev)
{
}

static int i2s_sam0_init(const struct device *dev)
{
	const struct i2s_sam0_dev_cfg *const dev_cfg = dev->config;
	struct i2s_sam0_dev_data *const dev_data = dev->data;
	I2s *const i2s = dev_cfg->regs;
	int ret;

	/* Configure interrupts */
	dev_cfg->irq_config(dev_cfg, dev);

	/* Initialize semaphores */
	k_sem_init(&dev_data->rx.sem, 0, CONFIG_I2S_SAM0_RX_BLOCK_COUNT);
	k_sem_init(&dev_data->tx.sem, CONFIG_I2S_SAM0_TX_BLOCK_COUNT,
			   CONFIG_I2S_SAM0_TX_BLOCK_COUNT);

	if (!device_is_ready(dev_cfg->dma_dev))
	{
		return -ENODEV;
	}

	/* Connect pins to the peripheral */
	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0)
	{
		return ret;
	}

	*dev_cfg->mclk |= dev_cfg->mclk_mask;
#ifdef MCLK
	GCLK->PCHCTRL[dev_cfg->gclk_id].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(dev_cfg->gclk_gen);
#else
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(dev_cfg->gclk_gen) | GCLK_CLKCTRL_ID(dev_cfg->gclk_id);
#endif
	
	i2s->CLKCTRL[dev_data->num].bit.MCKEN = !!dev_cfg->mck_enable;
	i2s->CLKCTRL[dev_data->num].bit.SLOTSIZE = 3;
	i2s->CLKCTRL[dev_data->num].bit.FSWIDTH = 0;
	i2s->CLKCTRL[dev_data->num].bit.NBSLOTS = 2;
	i2s->CLKCTRL[dev_data->num].bit.BITDELAY = 0;
	i2s->CTRLA.vec.CKEN = 1 << dev_data->num;
	i2s->CTRLA.vec.SEREN = 1 << dev_data->num;
	i2s->CTRLA.bit.ENABLE = 1;
	
	/* Enable module's IRQ */
	irq_enable(dev_cfg->irq_id);

	dev_data->tx.dev = dev_data->rx.dev = dev;

	return 0;
}

static DEVICE_API(i2s, i2s_sam0_driver_api) = {
	.configure = i2s_sam0_configure,
	.config_get = i2s_sam0_config_get,
	.trigger = i2s_sam0_trigger,
	.read = i2s_sam0_read,
	.write = i2s_sam0_write,
};

static void i2s_sam0_irq_config(const struct i2s_sam0_dev_cfg *cfg, const struct device *dev)
{
#ifdef CONFIG_DYNAMIC_INTERRUPTS
	if (cfg->irq_callback_isr) {
		arch_irq_connect_dynamic(cfg->irq_id, cfg->irq_prio, cfg->irq_callback_isr, dev, 0);
	}
#endif
}

#define I2S_SAM0_DMA_CHANNELS(n)                                  \
	.dma_dev = DEVICE_DT_GET(ATMEL_SAM0_DT_INST_DMA_CTLR(n, tx)), \
	.tx_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, tx),      \
	.tx_dma_channel = ATMEL_SAM0_DT_INST_DMA_CHANNEL(n, tx),      \
	.rx_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, rx),      \
	.rx_dma_channel = ATMEL_SAM0_DT_INST_DMA_CHANNEL(n, rx),

#define I2S_SAM0_CONFIG_DEFN(n)                                                    \
	static const struct i2s_sam0_dev_cfg i2s_sam0_config_##n = {                   \
		.regs = (I2s *)DT_INST_REG_ADDR(n),                                        \
		.gclk_gen = ATMEL_SAM0_DT_INST_ASSIGNED_CLOCKS_CELL_BY_NAME(n, gclk, gen), \
		.gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id),                       \
		.mclk = ATMEL_SAM0_DT_INST_MCLK_PM_REG_ADDR_OFFSET(n),                     \
		.mclk_mask = ATMEL_SAM0_DT_INST_MCLK_PM_PERIPH_MASK(n, bit),               \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                 \
		.irq_config = i2s_sam0_irq_config,                                         \
		.irq_callback_isr = i2s_sam0_isr,										   \
		.irq_id = DT_INST_IRQN(n),                                                 \
		.irq_prio = DT_INST_IRQ(n, priority),							           \
		.mck_enable = !DT_PROP_OR(n, no_mck, 1), 								       \
		I2S_SAM0_DMA_CHANNELS(n)}

#define I2S_SAM0_DMA_CFG(num, dir, type) (&i2s_sam0_config_##num)->dir##_dma_##type

#define I2S_SAM0_DEVICE_INIT(n)                                            \
	PINCTRL_DT_INST_DEFINE(n);                                             \
	I2S_SAM0_CONFIG_DEFN(n);                                               \
	struct queue_item rx_##n_ring_buf[CONFIG_I2S_SAM0_RX_BLOCK_COUNT + 1]; \
	struct queue_item tx_##n_ring_buf[CONFIG_I2S_SAM0_TX_BLOCK_COUNT + 1]; \
	static struct i2s_sam0_dev_data i2s_sam0_data_##n = {                  \
		.num = n,                                                          \
		.rx = {															   \
			.num = n,                                                      \
			.dma_channel = I2S_SAM0_DMA_CFG(n, rx, channel),               \
			.dma_perid = I2S_SAM0_DMA_CFG(n, rx, request),                 \
			.mem_block_queue.buf = rx_##n_ring_buf,                        \
			.mem_block_queue.len = ARRAY_SIZE(rx_##n_ring_buf),            \
			.stream_start = rx_stream_start,                               \
			.stream_disable = rx_stream_disable,                           \
			.queue_drop = rx_queue_drop,                                   \
			.set_data_format = set_rx_data_format,                         \
		},                                                                 \
		.tx = {                                                            \
			.num = n,                                                      \
			.dma_channel = I2S_SAM0_DMA_CFG(n, tx, channel),               \
			.dma_perid = I2S_SAM0_DMA_CFG(n, tx, request),                 \
			.mem_block_queue.buf = tx_##n_ring_buf,                        \
			.mem_block_queue.len = ARRAY_SIZE(tx_##n_ring_buf),            \
			.stream_start = tx_stream_start,                               \
			.stream_disable = tx_stream_disable,                           \
			.queue_drop = tx_queue_drop,                                   \
			.set_data_format = set_tx_data_format,                         \
		},                                                                 \
	};                                                                     \
	DEVICE_DT_INST_DEFINE(n, i2s_sam0_init, NULL,                          \
						  &i2s_sam0_data_##n,                              \
						  &i2s_sam0_config_##n, POST_KERNEL,              \
						  CONFIG_I2S_INIT_PRIORITY,                        \
						  &i2s_sam0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_SAM0_DEVICE_INIT)
