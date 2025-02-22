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
#include <soc.h>

#ifndef CONFIG_I2S_SAM0_RX_BLOCK_COUNT
#define CONFIG_I2S_SAM0_RX_BLOCK_COUNT 16
#endif
#ifndef CONFIG_I2S_SAM0_TX_BLOCK_COUNT
#define CONFIG_I2S_SAM0_TX_BLOCK_COUNT 16
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

	void (*irq_config)(void);
	uint8_t irq_id;

	uint8_t tx_dma_request;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_request;
	uint8_t rx_dma_channel;

};

struct stream
{
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

static void i2s_sam0_dma_tx_done(const struct device *dma_dev, void *arg,
								 uint32_t id, int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

	struct i2s_sam0_dev_data *const dev_data =
		(struct i2s_sam0_dev_data *const)arg;
	const struct i2s_sam0_dev_cfg *const cfg = dev_data->cfg;

	I2s *const i2s = cfg->regs;

	i2s->INTENSET.vec.TXRDY = (1 << dev_data->num);
}

static void i2s_sam0_dma_rx_done(const struct device *dma_dev, void *arg,
								 uint32_t id, int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);
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
	struct i2s_sam_dev_data *const dev_data = dev->data;
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
						   const struct device *dev_dma)
{
	int ret;

	ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->mem_block,
						   K_NO_WAIT);
	if (ret < 0)
	{
		return ret;
	}

	/* Workaround for a hardware bug: DMA engine will read first data
	 * item even if SSC_SR.RXEN (Receive Enable) is not set. An extra read
	 * before enabling DMA engine sets hardware FSM in the correct state.
	 */
	(void)ssc->SSC_RHR;

	struct dma_config dma_cfg = {
		.source_data_size = stream->word_size_bytes,
		.dest_data_size = stream->word_size_bytes,
		.block_count = 1,
		.dma_slot = stream->dma_perid,
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.dma_callback = i2s_sam0_dma_rx_done,
	};

	ret = start_dma(dev_dma, stream->dma_channel, &dma_cfg,
					(void *)&(ssc->SSC_RHR), stream->mem_block,
					stream->cfg.block_size);
	if (ret < 0)
	{
		LOG_ERR("Failed to start RX DMA transfer: %d", ret);
		return ret;
	}

	return 0;
}

static int tx_stream_start(struct stream *stream, I2s *const i2s,
						   const struct device *dev_dma)
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
		.dma_callback = i2s_sam0_dma_tx_done,
	};

	/* Assure cache coherency before DMA read operation */
	DCACHE_CLEAN(stream->mem_block, mem_block_size);

	ret = start_dma(dev_dma, stream->dma_channel, &dma_cfg,
					stream->mem_block, (void *)&(ssc->SSC_THR),
					mem_block_size);
	if (ret < 0)
	{
		LOG_ERR("Failed to start TX DMA transfer: %d", ret);
		return ret;
	}

	return 0;
}

static void rx_stream_disable(struct stream *stream, I2s *const i2s,
							  const struct device *dev_dma)
{
	dma_stop(dev_dma, stream->dma_channel);
	if (stream->mem_block != NULL)
	{
		k_mem_slab_free(stream->cfg.mem_slab, stream->mem_block);
		stream->mem_block = NULL;
	}
}

static void tx_stream_disable(struct stream *stream, I2s *const i2s,
							  const struct device *dev_dma)
{
	dma_stop(dev_dma, stream->dma_channel);
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

		ret = stream->stream_start(stream, ssc, dev_cfg->dev_dma);
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
		stream->stream_disable(stream, ssc, dev_cfg->dev_dma);
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

static void i2s_sam0_isr(const struct device *dev)
{
	const struct i2s_sam0_dev_cfg *const dev_cfg = dev->config;
	struct i2s_sam0_dev_data *const dev_data = dev->data;
	I2s *const i2s = dev_cfg->regs;
	uint32_t isr_status;

	/* Retrieve interrupt status */
	isr_status = ssc->SSC_SR & ssc->SSC_IMR;

	/* Check for RX buffer overrun */
	if (isr_status & SSC_SR_OVRUN)
	{
		dev_data->rx.state = I2S_STATE_ERROR;
		/* Disable interrupt */
		ssc->SSC_IDR = SSC_IDR_OVRUN;
	}
	/* Check for TX buffer underrun */
	if (isr_status & SSC_SR_TXEMPTY)
	{
		dev_data->tx.state = I2S_STATE_ERROR;
		/* Disable interrupt */
		ssc->SSC_IDR = SSC_IDR_TXEMPTY;
	}
}

static int i2s_sam0_init(const struct device *dev)
{
	const struct i2s_sam0_dev_cfg *const dev_cfg = dev->config;
	struct i2s_sam0_dev_data *const dev_data = dev->data;
	I2s *const i2s = dev_cfg->regs;
	int ret;

	/* Configure interrupts */
	dev_cfg->irq_config();

	/* Initialize semaphores */
	k_sem_init(&dev_data->rx.sem, 0, CONFIG_I2S_SAM0_RX_BLOCK_COUNT);
	k_sem_init(&dev_data->tx.sem, CONFIG_I2S_SAM0_TX_BLOCK_COUNT,
			   CONFIG_I2S_SAM0_TX_BLOCK_COUNT);

	if (!device_is_ready(dev_cfg->dev_dma))
	{
		return -ENODEV;
	}

	/* Connect pins to the peripheral */
	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0)
	{
		return ret;
	}

	*cfg->mclk |= cfg->mclk_mask;

#ifdef MCLK
	GCLK->PCHCTRL[cfg->gclk_id].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN(cfg->gclk_gen);
#else
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(cfg->gclk_gen) | GCLK_CLKCTRL_ID(cfg->gclk_id);
#endif

	i2s->CTRLA.bit.CKEN0 = 1;
	i2s->CTRLA.bit.SEREN0 = 1;
	i2s->CTRLA.bit.ENABLE = 1;

	/* Enable module's IRQ */
	irq_enable(dev_cfg->irq_id);

	return 0;
}

static DEVICE_API(i2s, i2s_sam0_driver_api) = {
	.configure = i2s_sam0_configure,
	.config_get = i2s_sam0_config_get,
	.trigger = i2s_sam0_trigger,
	.read = i2s_sam0_read,
	.write = i2s_sam0_write,
};

static const struct device *get_dev_from_dma_channel(uint32_t dma_channel)
{
	return &DEVICE_DT_NAME_GET(DT_DRV_INST(0));
}

static void i2s_sam0_irq_config(void)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), i2s_sam0_isr,
		    DEVICE_DT_INST_GET(0), 0);
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
		.irq_config = i2s_sam0_irq_config,										   \
		.irq_id = DT_INST_IRQN(n),												   \
		I2S_SAM0_DMA_CHANNELS(n)}


#define I2S_SAM0_DMA_CFG(num, dir, type) (&i2s_sam0_config_##num)->dir##_dma_##type

#define I2S_SAM0_DEVICE_INIT(n)                               			\
	PINCTRL_DT_INST_DEFINE(n);                                			\
	struct queue_item rx_##n_ring_buf[CONFIG_I2S_SAM_SSC_RX_BLOCK_COUNT + 1]; \
	struct queue_item tx_##n_ring_buf[CONFIG_I2S_SAM_SSC_TX_BLOCK_COUNT + 1]; \
	static struct i2s_sam0_dev_data i2s_sam0_data_##n = {     			\
		.num = n,                                             			\
		.rx = {															\
			.dma_channel = I2S_SAM0_DMA_CFG(n, rx, channel), 			\
			.dma_perid = I2S_SAM0_DMA_CFG(n, rx, request), 				\
			.mem_block_queue.buf = rx_##n_ring_buf,						\
			.mem_block_queue.len = ARRAY_SIZE(rx_##n_ring_buf),			\
			.stream_start = rx_stream_start,							\
			.stream_disable = rx_stream_disable,						\
			.queue_drop = rx_queue_drop,								\
			.set_data_format = set_rx_data_format,						\
		},																\
		.tx = {															\
			.dma_channel = I2S_SAM0_DMA_CFG(n, tx, channel),			\
			.dma_perid = I2S_SAM0_DMA_CFG(n, tx, request),				\
			.mem_block_queue.buf = tx_##n_ring_buf,						\
			.mem_block_queue.len = ARRAY_SIZE(tx_##n_ring_buf),			\
			.stream_start = tx_stream_start,							\
			.stream_disable = tx_stream_disable,						\
			.queue_drop = tx_queue_drop,								\
			.set_data_format = set_tx_data_format,						\
		},																\
	};																	\
	I2S_SAM0_CONFIG_DEFN(n);                                  			\
	DEVICE_DT_INST_DEFINE(n, i2s_sam0_init, NULL,             			\
						  &i2s_sam0_data_##n,                 			\
						  &i2s_sam0_config_##n, PRE_KERNEL_1, 			\
						  CONFIG_I2S_INIT_PRIORITY,           			\
						  &i2s_sam0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_SAM0_DEVICE_INIT)
