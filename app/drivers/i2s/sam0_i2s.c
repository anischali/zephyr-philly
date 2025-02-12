/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam0_i2s

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <string.h>

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
struct i2s_sam_dev_cfg {
	const struct device *dev_dma;
	I2s *regs;
	void (*irq_config)(void);
	const struct atmel_sam_pmc_config clock_cfg;
	const struct pinctrl_dev_config *pcfg;
	uint8_t irq_id;
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
	int (*set_data_format)(const struct i2s_sam_dev_cfg *const,
			       const struct i2s_config *);
};

/* Device run time data */
struct i2s_sam_dev_data {
	struct stream rx;
	struct stream tx;
};

static int sam0_i2s_configure(const struct device *dev, enum i2s_dir dir,
			     const struct i2s_config *i2s_cfg)
{
    const struct i2s_sam_dev_cfg *const dev_cfg = dev->config;
	struct i2s_sam_dev_data *const dev_data = dev->data;
	I2s *const i2s = dev_cfg->regs;
	uint8_t num_words = i2s_cfg->channels;
	uint8_t word_size_bits = i2s_cfg->word_size;
	uint32_t bit_clk_freq;
	struct stream *stream;
	int ret;

	if (dir == I2S_DIR_RX) {
		stream = &dev_data->rx;
	} else if (dir == I2S_DIR_TX) {
		stream = &dev_data->tx;
	} else if (dir == I2S_DIR_BOTH) {
		return -ENOSYS;
	} else {
		LOG_ERR("Either RX or TX direction must be selected");
		return -EINVAL;
	}

	if (stream->state != I2S_STATE_NOT_READY &&
	    stream->state != I2S_STATE_READY) {
		LOG_ERR("invalid state");
		return -EINVAL;
	}

	if (i2s_cfg->frame_clk_freq == 0U) {
		stream->queue_drop(stream);
		(void)memset(&stream->cfg, 0, sizeof(struct i2s_config));
		stream->state = I2S_STATE_NOT_READY;
		return 0;
	}

	//if (i2s_cfg->format & I2S_FMT_FRAME_CLK_INV) {
	//	LOG_ERR("Frame clock inversion is not implemented");
	//	LOG_ERR("Please submit a patch");
	//	return -EINVAL;
	//}

	//if (i2s_cfg->format & I2S_FMT_BIT_CLK_INV) {
	//	LOG_ERR("Bit clock inversion is not implemented");
	//	LOG_ERR("Please submit a patch");
	//	return -EINVAL;
	//}

	//if (word_size_bits < SAM_SSC_WORD_SIZE_BITS_MIN ||
	//    word_size_bits > SAM_SSC_WORD_SIZE_BITS_MAX) {
	//	LOG_ERR("Unsupported I2S word size");
	//	return -EINVAL;
	//}

	//if (num_words < SAM_SSC_WORD_PER_FRAME_MIN ||
	//    num_words > SAM_SSC_WORD_PER_FRAME_MAX) {
	//	LOG_ERR("Unsupported words per frame number");
	//	return -EINVAL;
	//}

	memcpy(&stream->cfg, i2s_cfg, sizeof(struct i2s_config));

	bit_clk_freq = i2s_cfg->frame_clk_freq * word_size_bits * num_words;
	ret = bit_clock_set(i2s, bit_clk_freq);
	if (ret < 0) {
		return ret;
	}

	ret = stream->set_data_format(dev_cfg, i2s_cfg);
	if (ret < 0) {
		return ret;
	}

	/* Set up DMA channel parameters */
	stream->word_size_bytes = get_word_size_bytes(word_size_bits);

	//if (i2s_cfg->options & I2S_OPT_LOOPBACK) {
	//	ssc->SSC_RFMR |= SSC_RFMR_LOOP;
	//}

	stream->state = I2S_STATE_READY;

	return -ENOTSUP;
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

#define sam0_i2s_INIT(index)                                                                        \
	DEVICE_DT_INST_DEFINE(index, &sam0_i2s_init, NULL, NULL, NULL, POST_KERNEL,                 \
			      CONFIG_I2S_INIT_PRIORITY, &sam0_i2s_driver_api);

DT_INST_FOREACH_STATUS_OKAY(sam0_i2s_INIT)





#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_SAM0_ASYNC

#define SAM0_UART_IRQ_CONNECT(n, m)					\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq),		\
			    DT_INST_IRQ_BY_IDX(n, m, priority),		\
			    uart_sam0_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));		\
	} while (false)

#define UART_SAM0_IRQ_HANDLER_DECL(n)					\
	static void uart_sam0_irq_config_##n(const struct device *dev)
#define UART_SAM0_IRQ_HANDLER_FUNC(n)					\
	.irq_config_func = uart_sam0_irq_config_##n,

#if DT_INST_IRQ_HAS_IDX(0, 3)
#define UART_SAM0_IRQ_HANDLER(n)					\
static void uart_sam0_irq_config_##n(const struct device *dev)		\
{									\
	SAM0_UART_IRQ_CONNECT(n, 0);					\
	SAM0_UART_IRQ_CONNECT(n, 1);					\
	SAM0_UART_IRQ_CONNECT(n, 2);					\
	SAM0_UART_IRQ_CONNECT(n, 3);					\
}
#else
#define UART_SAM0_IRQ_HANDLER(n)					\
static void uart_sam0_irq_config_##n(const struct device *dev)		\
{									\
	SAM0_UART_IRQ_CONNECT(n, 0);					\
}
#endif
#else
#define UART_SAM0_IRQ_HANDLER_DECL(n)
#define UART_SAM0_IRQ_HANDLER_FUNC(n)
#define UART_SAM0_IRQ_HANDLER(n)
#endif

#if CONFIG_UART_SAM0_ASYNC
#define UART_SAM0_DMA_CHANNELS(n)					\
	.dma_dev = DEVICE_DT_GET(ATMEL_SAM0_DT_INST_DMA_CTLR(n, tx)),	\
	.tx_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, tx),	\
	.tx_dma_channel = ATMEL_SAM0_DT_INST_DMA_CHANNEL(n, tx),	\
	.rx_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, rx),	\
	.rx_dma_channel = ATMEL_SAM0_DT_INST_DMA_CHANNEL(n, rx),
#else
#define UART_SAM0_DMA_CHANNELS(n)
#endif

#define UART_SAM0_SERCOM_PADS(n) \
	(DT_INST_PROP(n, rxpo) << SERCOM_USART_CTRLA_RXPO_Pos) |	\
	(DT_INST_PROP(n, txpo) << SERCOM_USART_CTRLA_TXPO_Pos)

#define UART_SAM0_SERCOM_COLLISION_DETECT(n) \
	(DT_INST_PROP(n, collision_detection))

#define ASSIGNED_CLOCKS_CELL_BY_NAME					\
	ATMEL_SAM0_DT_INST_ASSIGNED_CLOCKS_CELL_BY_NAME

#define UART_SAM0_CONFIG_DEFN(n)					\
static const struct uart_sam0_dev_cfg uart_sam0_config_##n = {		\
	.regs = (SercomUsart *)DT_INST_REG_ADDR(n),			\
	.baudrate = DT_INST_PROP(n, current_speed),			\
	.gclk_gen = ASSIGNED_CLOCKS_CELL_BY_NAME(n, gclk, gen),		\
	.gclk_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, id),		\
	.mclk = ATMEL_SAM0_DT_INST_MCLK_PM_REG_ADDR_OFFSET(n),		\
	.mclk_mask = ATMEL_SAM0_DT_INST_MCLK_PM_PERIPH_MASK(n, bit),	\
	.pads = UART_SAM0_SERCOM_PADS(n),				\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	UART_SAM0_IRQ_HANDLER_FUNC(n)					\
	UART_SAM0_DMA_CHANNELS(n)					\
}

#define SAM0_I2S_DEVICE_INIT(n)					\
PINCTRL_DT_INST_DEFINE(n);						\
static struct uart_sam0_dev_data uart_sam0_data_##n;			\
UART_SAM0_IRQ_HANDLER_DECL(n);						\
UART_SAM0_CONFIG_DEFN(n);						\
DEVICE_DT_INST_DEFINE(n, uart_sam0_init, NULL,				\
		    &uart_sam0_data_##n,				\
		    &uart_sam0_config_##n, PRE_KERNEL_1,		\
		    CONFIG_SERIAL_INIT_PRIORITY,			\
		    &uart_sam0_driver_api);				\
UART_SAM0_IRQ_HANDLER(n)

DT_INST_FOREACH_STATUS_OKAY(UART_SAM0_DEVICE_INIT)
