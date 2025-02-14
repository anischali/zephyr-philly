/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam0_i2s

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
struct sam0_i2s_dev_cfg {
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
	int (*set_data_format)(const struct sam0_i2s_dev_cfg *const,
			       const struct i2s_config *);
};

/* Device run time data */
struct sam0_i2s_dev_data {
	struct stream rx;
	struct stream tx;
};

static int sam0_i2s_configure(const struct device *dev, enum i2s_dir dir,
			     const struct i2s_config *i2s_cfg)
{
	return 0;
}

static const struct i2s_config *sam0_i2s_config_get(const struct device *dev, enum i2s_dir dir)
{
	return NULL;
}

static int sam0_i2s_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	return -ENOTSUP;
}

static int sam0_i2s_read(const struct device *dev, void **mem_block, size_t *size)
{
	return -ENOTSUP;
}

static int sam0_i2s_write(const struct device *dev, void *mem_block, size_t size)
{
	return -ENOTSUP;
}

static DEVICE_API(i2s, sam0_i2s_driver_api) = {
	.configure = sam0_i2s_configure,
	.config_get = sam0_i2s_config_get,
	.trigger = sam0_i2s_trigger,
	.read = sam0_i2s_read,
	.write = sam0_i2s_write,
};

static int sam0_i2s_init(const struct device *dev)
{
	return 0;
}

#define SAM0_I2S_DMA_CHANNELS(n)					\
	.dma_dev = DEVICE_DT_GET(ATMEL_SAM0_DT_INST_DMA_CTLR(n, tx)),	\
	.tx_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, tx),	\
	.tx_dma_channel = ATMEL_SAM0_DT_INST_DMA_CHANNEL(n, tx),	\
	.rx_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, rx),	\
	.rx_dma_channel = ATMEL_SAM0_DT_INST_DMA_CHANNEL(n, rx),


#define SAM0_I2S_CONFIG_DEFN(n)					\
static const struct sam0_i2s_dev_cfg sam0_i2s_config_##n = {		\
	.regs = (I2s *)DT_INST_REG_ADDR(n),			\
	.gclk_gen = ATMEL_SAM0_DT_INST_ASSIGNED_CLOCKS_CELL_BY_NAME(n, gclk, gen),		\
	.gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id),		\
	.mclk = ATMEL_SAM0_DT_INST_MCLK_PM_REG_ADDR_OFFSET(n),		\
	.mclk_mask = ATMEL_SAM0_DT_INST_MCLK_PM_PERIPH_MASK(n, bit),	\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	SAM0_I2S_DMA_CHANNELS(n)					\
}

#define SAM0_I2S_DEVICE_INIT(n)					\
PINCTRL_DT_INST_DEFINE(n);						\
static struct sam0_i2s_dev_data sam0_i2s_data_##n;			\
SAM0_I2S_CONFIG_DEFN(n);						\
DEVICE_DT_INST_DEFINE(n, sam0_i2s_init, NULL,				\
		    &sam0_i2s_data_##n,				\
		    &sam0_i2s_config_##n, PRE_KERNEL_1,		\
		    CONFIG_I2S_INIT_PRIORITY,			\
		    &sam0_i2s_driver_api);				\

DT_INST_FOREACH_STATUS_OKAY(SAM0_I2S_DEVICE_INIT)
