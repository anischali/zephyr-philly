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


struct queue_item {
	void *mem_block;
	size_t size;
};

/* Minimal ring buffer implementation */
struct ring_buffer {
	struct queue_item *buf;
	uint16_t len;
	uint16_t head;
	uint16_t tail;
};

/* Device constant configuration parameters */
struct i2s_sam0_dev_cfg {
	const struct device *dma_dev;
	const struct pinctrl_dev_config *pcfg;
	I2s *regs;
	volatile uint32_t *mclk;
	uint32_t mclk_mask;
	uint32_t gclk_gen;
	uint16_t gclk_id;

	uint8_t tx_dma_request;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_request;
	uint8_t rx_dma_channel;
};

struct stream {
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
struct i2s_sam0_dev_data {
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
		(struct i2s_sam0_dev_data *const) arg;
	const struct i2s_sam0_dev_cfg *const cfg = dev_data->cfg;

	I2s * const i2s = cfg->regs;

	i2s->INTENSET.vec.TXRDY = (1 << dev_data->num);
}

static void i2s_sam0_dma_rx_done(const struct device *dma_dev, void *arg,
				  uint32_t id, int error_code)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(id);
	ARG_UNUSED(error_code);

#if 0
	struct i2s_sam0_dev_data *const dev_data =
		(struct i2s_sam0_dev_data *const)arg;
	const struct device *dev = dev_data->dev;
	const struct i2s_sam0_dev_cfg *const cfg = dev_data->cfg;
	I2s * const regs = cfg->regs;
	unsigned int key = irq_lock();

	if (dev_data->rx_len == 0U) {
		irq_unlock(key);
		return;
	}
	i2s_sam0_notify_rx_processed(dev_data, dev_data->rx_len);

	if (dev_data->async_cb) {
		struct uart_event evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf = {
				.buf = dev_data->rx_buf,
			},
		};

		dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
	}

	/* No next buffer, so end the transfer */
	if (!dev_data->rx_next_len) {
		dev_data->rx_buf = NULL;
		dev_data->rx_len = 0U;

		if (dev_data->async_cb) {
			struct uart_event evt = {
				.type = UART_RX_DISABLED,
			};

			dev_data->async_cb(dev, &evt, dev_data->async_cb_data);
		}

		irq_unlock(key);
		return;
	}

	dev_data->rx_buf = dev_data->rx_next_buf;
	dev_data->rx_len = dev_data->rx_next_len;
	dev_data->rx_next_buf = NULL;
	dev_data->rx_next_len = 0U;
	dev_data->rx_processed_len = 0U;

	dma_reload(cfg->dma_dev, cfg->rx_dma_channel,
		   (uint32_t)(&(regs->DATA.reg)),
		   (uint32_t)dev_data->rx_buf, dev_data->rx_len);

	/*
	 * If there should be a timeout, handle starting the DMA in the
	 * ISR, since reception resets it and DMA completion implies
	 * reception.  This also catches the case of DMA completion during
	 * timeout handling.
	 */
	if (dev_data->rx_timeout_time != SYS_FOREVER_US) {
		dev_data->rx_waiting_for_irq = true;
		regs->INTENSET.reg = SERCOM_USART_INTENSET_RXC;
		irq_unlock(key);
		return;
	}

	/* Otherwise, start the transfer immediately. */
	dma_start(cfg->dma_dev, cfg->rx_dma_channel);

	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	dev_data->async_cb(dev, &evt, dev_data->async_cb_data);

	irq_unlock(key);
#endif
}


static int i2s_sam0_configure(const struct device *dev, enum i2s_dir dir,
			     const struct i2s_config *i2s_cfg)
{
	return 0;
}

static const struct i2s_config *i2s_sam0_config_get(const struct device *dev, enum i2s_dir dir)
{
	return NULL;
}

static int i2s_sam0_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	return -ENOTSUP;
}

static int i2s_sam0_read(const struct device *dev, void **mem_block, size_t *size)
{
	return -ENOTSUP;
}

static int i2s_sam0_write(const struct device *dev, void *mem_block, size_t size)
{
	return -ENOTSUP;
}

static DEVICE_API(i2s, i2s_sam0_driver_api) = {
	.configure = i2s_sam0_configure,
	.config_get = i2s_sam0_config_get,
	.trigger = i2s_sam0_trigger,
	.read = i2s_sam0_read,
	.write = i2s_sam0_write,
};

static int i2s_sam0_init(const struct device *dev)
{
	const struct i2s_sam0_dev_cfg *const cfg = dev->config;
	struct i2s_sam0_dev_data *dev_data = dev->data;

	I2s *const i2s = cfg->regs;
	int retval;

	/* Initialize semaphores */
	k_sem_init(&dev_data->rx.sem, 0, CONFIG_I2S_SAM0_RX_BLOCK_COUNT);
	k_sem_init(&dev_data->tx.sem, CONFIG_I2S_SAM0_TX_BLOCK_COUNT,
		   CONFIG_I2S_SAM0_TX_BLOCK_COUNT);

	*cfg->mclk |= cfg->mclk_mask;

#ifdef MCLK
	GCLK->PCHCTRL[cfg->gclk_id].reg = GCLK_PCHCTRL_CHEN
					| GCLK_PCHCTRL_GEN(cfg->gclk_gen);
#else
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN
			  | GCLK_CLKCTRL_GEN(cfg->gclk_gen)
			  | GCLK_CLKCTRL_ID(cfg->gclk_id);
#endif

	retval = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	dev_data->dev = (struct device *)dev;
	dev_data->cfg = (struct i2s_sam0_dev_cfg *)cfg;
	if (!device_is_ready(cfg->dma_dev)) {
		return -ENODEV;
	}

	if (cfg->tx_dma_channel != 0xFFU) {
		struct dma_config dma_cfg = { 0 };
		struct dma_block_config dma_blk = { 0 };

		dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.user_data = dev_data;
		dma_cfg.dma_callback = i2s_sam0_dma_tx_done;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;
		dma_cfg.dma_slot = cfg->tx_dma_request;

		dma_blk.block_size = 1;
		dma_blk.dest_address = (uint32_t)(&(i2s->DATA[dev_data->num].reg));
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		retval = dma_config(cfg->dma_dev, cfg->tx_dma_channel,
				    &dma_cfg);
		if (retval != 0) {
			return retval;
		}
	}

	if (cfg->rx_dma_channel != 0xFFU) {
		struct dma_config dma_cfg = { 0 };
		struct dma_block_config dma_blk = { 0 };

		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_cfg.source_data_size = 1;
		dma_cfg.dest_data_size = 1;
		dma_cfg.user_data = dev_data;
		dma_cfg.dma_callback = i2s_sam0_dma_rx_done;
		dma_cfg.block_count = 1;
		dma_cfg.head_block = &dma_blk;
		dma_cfg.dma_slot = cfg->rx_dma_request;

		dma_blk.block_size = 1;
		dma_blk.source_address = (uint32_t)(&(i2s->DATA[dev_data->num].reg));
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

		retval = dma_config(cfg->dma_dev, cfg->rx_dma_channel,
				    &dma_cfg);
		if (retval != 0) {
			return retval;
		}
	}

	i2s->CTRLA.bit.CKEN0 = 1;
	i2s->CTRLA.bit.SEREN0 = 1;
	i2s->CTRLA.bit.ENABLE = 1;

	return 0;	
}

#define I2S_SAM0_DMA_CHANNELS(n)					\
	.dma_dev = DEVICE_DT_GET(ATMEL_SAM0_DT_INST_DMA_CTLR(n, tx)),	\
	.tx_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, tx),	\
	.tx_dma_channel = ATMEL_SAM0_DT_INST_DMA_CHANNEL(n, tx),	\
	.rx_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, rx),	\
	.rx_dma_channel = ATMEL_SAM0_DT_INST_DMA_CHANNEL(n, rx),


#define I2S_SAM0_CONFIG_DEFN(n)					\
static const struct i2s_sam0_dev_cfg i2s_sam0_config_##n = {		\
	.regs = (I2s *)DT_INST_REG_ADDR(n),			\
	.gclk_gen = ATMEL_SAM0_DT_INST_ASSIGNED_CLOCKS_CELL_BY_NAME(n, gclk, gen),		\
	.gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id),		\
	.mclk = ATMEL_SAM0_DT_INST_MCLK_PM_REG_ADDR_OFFSET(n),		\
	.mclk_mask = ATMEL_SAM0_DT_INST_MCLK_PM_PERIPH_MASK(n, bit),	\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	I2S_SAM0_DMA_CHANNELS(n)					\
}

#define I2S_SAM0_DEVICE_INIT(n)					\
PINCTRL_DT_INST_DEFINE(n);						\
static struct i2s_sam0_dev_data i2s_sam0_data_##n = { \
	.num = n, \
};			\
I2S_SAM0_CONFIG_DEFN(n);						\
DEVICE_DT_INST_DEFINE(n, i2s_sam0_init, NULL,				\
		    &i2s_sam0_data_##n,				\
		    &i2s_sam0_config_##n, PRE_KERNEL_1,		\
		    CONFIG_I2S_INIT_PRIORITY,			\
		    &i2s_sam0_driver_api);				\

DT_INST_FOREACH_STATUS_OKAY(I2S_SAM0_DEVICE_INIT)
