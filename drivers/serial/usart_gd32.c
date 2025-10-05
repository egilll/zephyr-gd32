/*
 * Copyright (c) 2021, ATL Electronics
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_usart

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/dma.h>

#include <gd32_usart.h>

#ifdef CONFIG_UART_ASYNC_API
static void usart_gd32_dma_rx_cb(const struct device *dma_dev, void *user_data,
				 uint32_t channel, int status);
static void usart_gd32_dma_tx_cb(const struct device *dma_dev, void *user_data,
				 uint32_t channel, int status);

#define GD32_USART_DMA_STREAM_INIT(index, dir, ch_dir)                              \
	.dma_##dir = {                                                               \
		.dma_dev = COND_CODE_1(DT_INST_DMAS_HAS_NAME(index, dir),              \
			(DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, dir))), (NULL)), \
		.dma_channel = COND_CODE_1(                                         \
			DT_INST_DMAS_HAS_NAME(index, dir),                              \
			(DT_INST_DMAS_CELL_BY_NAME(index, dir, channel)), (0)),         \
		.dma_cfg = {                                                       \
			.dma_slot = COND_CODE_1(                                    \
				DT_INST_DMAS_HAS_NAME(index, dir),                    \
				(DT_INST_DMAS_CELL_BY_NAME(index, dir, slot)), (0)),   \
			.channel_direction = ch_dir,                               \
			.channel_priority = 0,                                    \
			.source_data_size = 1,                                    \
			.dest_data_size = 1,                                      \
			.source_burst_length = 1,                                 \
			.dest_burst_length = 1,                                   \
			.block_count = 1,                                         \
			.dma_callback = usart_gd32_dma_##dir##_cb,                \
		},                                                          \
	}

#define GD32_USART_DMA_INIT_FIELDS(index)                                          \
	GD32_USART_DMA_STREAM_INIT(index, rx, PERIPHERAL_TO_MEMORY),              \
	GD32_USART_DMA_STREAM_INIT(index, tx, MEMORY_TO_PERIPHERAL)
#else
#define GD32_USART_DMA_INIT_FIELDS(index)
#endif

/* Unify GD32 HAL USART status register name to USART_STAT */
#ifndef USART_STAT
#define USART_STAT USART_STAT0
#endif

struct gd32_usart_config {
	uint32_t reg;
	uint16_t clkid;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
	uint32_t parity;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */
#ifdef CONFIG_UART_ASYNC_API
	bool rx_dma_circular;
#endif
};

struct gd32_usart_data {
	uint32_t baud_rate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_ASYNC_API
	uart_callback_t async_cb;
	void *async_user_data;
	const struct device *dev;
	struct {
		const struct device *dma_dev;
		uint32_t dma_channel;
		struct dma_config dma_cfg;
		struct dma_block_config blk_cfg;
		uint8_t *buffer;
		size_t buffer_length;
		size_t offset;
		size_t counter;
	} dma_rx, dma_tx;
	uint8_t *rx_next_buffer;
	size_t rx_next_buffer_len;
#endif /* CONFIG_UART_ASYNC_API */
};

#ifdef CONFIG_UART_ASYNC_API
#define GD32_USART_DATA_REG_ADDR(base) ((uint32_t)((base) + 0x04U))

static inline void async_user_callback(struct gd32_usart_data *data,
				       struct uart_event *evt)
{
	if (data->async_cb) {
		data->async_cb(data->dev, evt, data->async_user_data);
	}
}

static inline void async_evt_rx_buf_request(struct gd32_usart_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_REQUEST,
	};

	async_user_callback(data, &evt);
}

static inline void async_evt_rx_buf_released(struct gd32_usart_data *data,
				           uint8_t *buf)
{
	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf.buf = buf,
	};

	async_user_callback(data, &evt);
}

static inline void async_evt_rx_disabled(struct gd32_usart_data *data)
{
	struct uart_event evt = {
		.type = UART_RX_DISABLED,
	};

	async_user_callback(data, &evt);
}

static inline void async_evt_rx_stopped(struct gd32_usart_data *data, int reason)
{
	struct uart_event evt = {
		.type = UART_RX_STOPPED,
		.data.rx_stop = {
			.reason = reason,
			.data = {
				.buf = data->dma_rx.buffer,
				.len = data->dma_rx.counter,
				.offset = 0,
			},
		},
	};

	async_user_callback(data, &evt);
}

static inline void async_evt_tx_done(struct gd32_usart_data *data)
{
	struct uart_event evt = {
		.type = UART_TX_DONE,
		.data.tx = {
			.buf = data->dma_tx.buffer,
			.len = data->dma_tx.counter,
		},
	};

	data->dma_tx.buffer = NULL;
	data->dma_tx.buffer_length = 0U;
	data->dma_tx.offset = 0U;
	data->dma_tx.counter = 0U;

	async_user_callback(data, &evt);
}

static inline void async_evt_tx_abort(struct gd32_usart_data *data)
{
	struct uart_event evt = {
		.type = UART_TX_ABORTED,
		.data.tx = {
			.buf = data->dma_tx.buffer,
			.len = data->dma_tx.counter,
		},
	};

	data->dma_tx.buffer = NULL;
	data->dma_tx.buffer_length = 0U;
	data->dma_tx.offset = 0U;
	data->dma_tx.counter = 0U;

	async_user_callback(data, &evt);
}
#endif /* CONFIG_UART_ASYNC_API */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
static void usart_gd32_isr(const struct device *dev)
{
	struct gd32_usart_data *const data = dev->data;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
#else
	ARG_UNUSED(data);
#endif
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */

#ifdef CONFIG_UART_ASYNC_API
static void usart_gd32_dma_rx_cb(const struct device *dma_dev, void *user_data,
				 uint32_t channel, int status)
{
	const struct device *dev = user_data;
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		async_evt_rx_stopped(data, UART_ERROR_OVERRUN);
		dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
		data->dma_rx.buffer_length = 0U;
		async_evt_rx_disabled(data);
		return;
	}

	if (data->dma_rx.buffer_length == 0U) {
		return;
	}

	if (data->dma_rx.dma_cfg.cyclic) {
		size_t pos = (status == DMA_STATUS_BLOCK) ?
			(data->dma_rx.buffer_length / 2U) : data->dma_rx.buffer_length;

		data->dma_rx.counter = pos;

		struct uart_event evt = {
			.type = UART_RX_RDY,
			.data.rx = {
				.buf = data->dma_rx.buffer,
				.len = data->dma_rx.counter - data->dma_rx.offset,
				.offset = data->dma_rx.offset,
			},
		};

		async_user_callback(data, &evt);
		data->dma_rx.offset = (status == DMA_STATUS_BLOCK) ? pos : 0U;
		return;
	}

	struct dma_status stat;

	if (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat) == 0) {
		data->dma_rx.counter = data->dma_rx.buffer_length - stat.pending_length;
	} else {
		data->dma_rx.counter = data->dma_rx.buffer_length;
	}

	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx = {
			.buf = data->dma_rx.buffer,
			.len = data->dma_rx.counter - data->dma_rx.offset,
			.offset = data->dma_rx.offset,
		},
	};

	data->dma_rx.offset = data->dma_rx.counter;
	async_user_callback(data, &evt);

	uint8_t *released = data->dma_rx.buffer;

	if (data->rx_next_buffer_len) {
		data->dma_rx.buffer = data->rx_next_buffer;
		data->dma_rx.buffer_length = data->rx_next_buffer_len;
		data->rx_next_buffer = NULL;
		data->rx_next_buffer_len = 0U;
		data->dma_rx.offset = 0U;
		data->dma_rx.counter = 0U;
		dma_reload(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
			  GD32_USART_DATA_REG_ADDR(cfg->reg),
			  (uint32_t)data->dma_rx.buffer,
			  data->dma_rx.buffer_length);
		dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
		async_evt_rx_buf_released(data, released);
		async_evt_rx_buf_request(data);
	} else {
		dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
		async_evt_rx_buf_released(data, released);
		data->dma_rx.buffer = NULL;
		data->dma_rx.buffer_length = 0U;
		data->dma_rx.offset = 0U;
		data->dma_rx.counter = 0U;
		async_evt_rx_disabled(data);
	}
}

static void usart_gd32_dma_tx_cb(const struct device *dma_dev, void *user_data,
				 uint32_t channel, int status)
{
	const struct device *dev = user_data;
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (status != 0) {
		return;
	}

	struct dma_status stat;
	size_t tx_len = data->dma_tx.buffer_length;

	if (tx_len > 0U) {
		if (dma_get_status(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &stat) == 0) {
			data->dma_tx.counter = tx_len - stat.pending_length;
		} else {
			data->dma_tx.counter = tx_len;
		}
	}

	async_evt_tx_done(data);
	if (data->dma_tx.buffer == NULL) {
		dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
		usart_dma_transmit_config(cfg->reg, USART_DENT_DISABLE);
	}
}

static int usart_gd32_async_callback_set(const struct device *dev,
					  uart_callback_t callback,
					  void *user_data)
{
	struct gd32_usart_data *data = dev->data;

	data->async_cb = callback;
	data->async_user_data = user_data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS)
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	data->user_cb = NULL;
	data->user_data = NULL;
#endif
#endif

	return 0;
}

static int usart_gd32_async_tx(const struct device *dev, const uint8_t *tx_data,
				 size_t buf_size, int32_t timeout)
{
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;
	int ret;

	ARG_UNUSED(timeout);

	if (data->dma_tx.dma_dev == NULL) {
		return -ENODEV;
	}

	if ((tx_data == NULL) || (buf_size == 0U)) {
		return -EINVAL;
	}

	if (data->dma_tx.buffer_length != 0U) {
		return -EBUSY;
	}

	data->dma_tx.buffer = (uint8_t *)tx_data;
	data->dma_tx.buffer_length = buf_size;
	data->dma_tx.offset = 0U;
	data->dma_tx.counter = 0U;
	data->dma_tx.blk_cfg.source_address = (uint32_t)data->dma_tx.buffer;
	data->dma_tx.blk_cfg.block_size = data->dma_tx.buffer_length;

	ret = dma_config(data->dma_tx.dma_dev, data->dma_tx.dma_channel,
			       &data->dma_tx.dma_cfg);
	if (ret < 0) {
		data->dma_tx.buffer = NULL;
		data->dma_tx.buffer_length = 0U;
		return ret;
	}

	usart_dma_transmit_config(cfg->reg, USART_DENT_ENABLE);

	return dma_start(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
}

static int usart_gd32_async_tx_abort(const struct device *dev)
{
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;
	struct dma_status stat;

	if (data->dma_tx.buffer_length == 0U) {
		return -EINVAL;
	}

	if (dma_get_status(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &stat) == 0) {
		data->dma_tx.counter = data->dma_tx.buffer_length - stat.pending_length;
	}

	dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
	usart_dma_transmit_config(cfg->reg, USART_DENT_DISABLE);
	async_evt_tx_abort(data);

	return 0;
}

static int usart_gd32_async_rx_enable(const struct device *dev, uint8_t *buf,
					 size_t len, int32_t timeout)
{
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;
	int ret;

	ARG_UNUSED(timeout);

	if (data->dma_rx.dma_dev == NULL) {
		return -ENODEV;
	}

	if (data->dma_rx.buffer_length != 0U) {
		return -EBUSY;
	}

	if ((buf == NULL) || (len == 0U)) {
		return -EINVAL;
	}

	data->dma_rx.buffer = buf;
	data->dma_rx.buffer_length = len;
	data->dma_rx.offset = 0U;
	data->dma_rx.counter = 0U;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0U;

	data->dma_rx.dma_cfg.cyclic = cfg->rx_dma_circular ? 1U : 0U;
	data->dma_rx.blk_cfg.dest_address = (uint32_t)data->dma_rx.buffer;
	data->dma_rx.blk_cfg.block_size = data->dma_rx.buffer_length;

	ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
			       &data->dma_rx.dma_cfg);
	if (ret < 0) {
		data->dma_rx.buffer = NULL;
		data->dma_rx.buffer_length = 0U;
		return ret;
	}

	usart_dma_receive_config(cfg->reg, USART_DENR_ENABLE);

	return dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
}

static int usart_gd32_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	struct gd32_usart_data *data = dev->data;
	unsigned int key;
	int ret = 0;

	key = irq_lock();

	if (data->dma_rx.dma_cfg.cyclic || (data->dma_rx.buffer_length == 0U)) {
		ret = -EACCES;
		goto out;
	}

	if (data->rx_next_buffer_len != 0U) {
		ret = -EBUSY;
		goto out;
	}

	data->rx_next_buffer = buf;
	data->rx_next_buffer_len = len;

out:
	irq_unlock(key);
	return ret;
}

static int usart_gd32_async_rx_disable(const struct device *dev)
{
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;
	struct dma_status stat;
	unsigned int key;

	key = irq_lock();

	if (data->dma_rx.buffer_length == 0U) {
		irq_unlock(key);
		return -EINVAL;
	}

	dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
	usart_dma_receive_config(cfg->reg, USART_DENR_DISABLE);

	if (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat) == 0) {
		size_t rx_len = data->dma_rx.buffer_length - stat.pending_length;

		if (rx_len > data->dma_rx.offset) {
			data->dma_rx.counter = rx_len;
			struct uart_event evt = {
				.type = UART_RX_RDY,
				.data.rx = {
					.buf = data->dma_rx.buffer,
					.len = data->dma_rx.counter - data->dma_rx.offset,
					.offset = data->dma_rx.offset,
				},
			};

			data->dma_rx.offset = data->dma_rx.counter;
			async_user_callback(data, &evt);
		}
	}

	async_evt_rx_buf_released(data, data->dma_rx.buffer);

	if (data->rx_next_buffer_len) {
		async_evt_rx_buf_released(data, data->rx_next_buffer);
		data->rx_next_buffer = NULL;
		data->rx_next_buffer_len = 0U;
	}

	data->dma_rx.buffer = NULL;
	data->dma_rx.buffer_length = 0U;
	data->dma_rx.offset = 0U;
	data->dma_rx.counter = 0U;

	async_evt_rx_disabled(data);
	irq_unlock(key);

	return 0;
}
#endif /* CONFIG_UART_ASYNC_API */

static int usart_gd32_init(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;
	struct gd32_usart_data *const data = dev->data;
	uint32_t word_length;
	uint32_t parity;
	int ret;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/**
	 * In order to keep the transfer data size to 8 bits(1 byte),
	 * append word length to 9BIT if parity bit enabled.
	 */
	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		parity = USART_PM_NONE;
		word_length = USART_WL_8BIT;
		break;
	case UART_CFG_PARITY_ODD:
		parity = USART_PM_ODD;
		word_length = USART_WL_9BIT;
		break;
	case UART_CFG_PARITY_EVEN:
		parity = USART_PM_EVEN;
		word_length = USART_WL_9BIT;
		break;
	default:
		return -ENOTSUP;
	}

	(void)clock_control_on(GD32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&cfg->clkid);

	(void)reset_line_toggle_dt(&cfg->reset);

	usart_baudrate_set(cfg->reg, data->baud_rate);
	usart_parity_config(cfg->reg, parity);
	usart_word_length_set(cfg->reg, word_length);
	/* Default to 1 stop bit */
	usart_stop_bit_set(cfg->reg, USART_STB_1BIT);
	usart_receive_config(cfg->reg, USART_RECEIVE_ENABLE);
	usart_transmit_config(cfg->reg, USART_TRANSMIT_ENABLE);
	usart_enable(cfg->reg);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	cfg->irq_config_func(dev);
#endif

#ifdef CONFIG_UART_ASYNC_API
	data->dev = dev;
	data->dma_rx.dma_cfg.head_block = &data->dma_rx.blk_cfg;
	data->dma_tx.dma_cfg.head_block = &data->dma_tx.blk_cfg;
	data->dma_rx.dma_cfg.user_data = (void *)dev;
	data->dma_tx.dma_cfg.user_data = (void *)dev;
	data->dma_rx.blk_cfg.source_address = GD32_USART_DATA_REG_ADDR(cfg->reg);
	data->dma_rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_rx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->dma_tx.blk_cfg.dest_address = GD32_USART_DATA_REG_ADDR(cfg->reg);
	data->dma_tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->dma_tx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_rx.buffer = NULL;
	data->dma_rx.buffer_length = 0U;
	data->dma_rx.offset = 0U;
	data->dma_rx.counter = 0U;
	data->dma_tx.buffer = NULL;
	data->dma_tx.buffer_length = 0U;
	data->dma_tx.offset = 0U;
	data->dma_tx.counter = 0U;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0U;
#endif

	return 0;
}

static int usart_gd32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct gd32_usart_config *const cfg = dev->config;
	uint32_t status;

	status = usart_flag_get(cfg->reg, USART_FLAG_RBNE);

	if (!status) {
		return -EPERM;
	}

	*c = usart_data_receive(cfg->reg);

	return 0;
}

static void usart_gd32_poll_out(const struct device *dev, unsigned char c)
{
	const struct gd32_usart_config *const cfg = dev->config;

	usart_data_transmit(cfg->reg, c);

	while (usart_flag_get(cfg->reg, USART_FLAG_TBE) == RESET) {
		;
	}
}

static int usart_gd32_err_check(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;
	uint32_t status = USART_STAT(cfg->reg);
	int errors = 0;

	if (status & USART_FLAG_ORERR) {
		usart_flag_clear(cfg->reg, USART_FLAG_ORERR);

		errors |= UART_ERROR_OVERRUN;
	}

	if (status & USART_FLAG_PERR) {
		usart_flag_clear(cfg->reg, USART_FLAG_PERR);

		errors |= UART_ERROR_PARITY;
	}

	if (status & USART_FLAG_FERR) {
		usart_flag_clear(cfg->reg, USART_FLAG_FERR);

		errors |= UART_ERROR_FRAMING;
	}

	usart_flag_clear(cfg->reg, USART_FLAG_NERR);

	return errors;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
int usart_gd32_fifo_fill(const struct device *dev, const uint8_t *tx_data,
			 int len)
{
	const struct gd32_usart_config *const cfg = dev->config;
	int num_tx = 0U;

	while ((len - num_tx > 0) &&
	       usart_flag_get(cfg->reg, USART_FLAG_TBE)) {
		usart_data_transmit(cfg->reg, tx_data[num_tx++]);
	}

	return num_tx;
}

int usart_gd32_fifo_read(const struct device *dev, uint8_t *rx_data,
			 const int size)
{
	const struct gd32_usart_config *const cfg = dev->config;
	int num_rx = 0U;

	while ((size - num_rx > 0) &&
	       usart_flag_get(cfg->reg, USART_FLAG_RBNE)) {
		rx_data[num_rx++] = usart_data_receive(cfg->reg);
	}

	return num_rx;
}

void usart_gd32_irq_tx_enable(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;

	usart_interrupt_enable(cfg->reg, USART_INT_TC);
}

void usart_gd32_irq_tx_disable(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;

	usart_interrupt_disable(cfg->reg, USART_INT_TC);
}

int usart_gd32_irq_tx_ready(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;

	return usart_flag_get(cfg->reg, USART_FLAG_TBE) &&
	       usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_TC);
}

int usart_gd32_irq_tx_complete(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;

	return usart_flag_get(cfg->reg, USART_FLAG_TC);
}

void usart_gd32_irq_rx_enable(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;

	usart_interrupt_enable(cfg->reg, USART_INT_RBNE);
}

void usart_gd32_irq_rx_disable(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;

	usart_interrupt_disable(cfg->reg, USART_INT_RBNE);
}

int usart_gd32_irq_rx_ready(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;

	return usart_flag_get(cfg->reg, USART_FLAG_RBNE);
}

void usart_gd32_irq_err_enable(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;

	usart_interrupt_enable(cfg->reg, USART_INT_ERR);
	usart_interrupt_enable(cfg->reg, USART_INT_PERR);
}

void usart_gd32_irq_err_disable(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;

	usart_interrupt_disable(cfg->reg, USART_INT_ERR);
	usart_interrupt_disable(cfg->reg, USART_INT_PERR);
}

int usart_gd32_irq_is_pending(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;

	return ((usart_flag_get(cfg->reg, USART_FLAG_RBNE) &&
		 usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_RBNE)) ||
		(usart_flag_get(cfg->reg, USART_FLAG_TC) &&
		 usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_TC)));
}

int usart_gd32_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 1;
}

void usart_gd32_irq_callback_set(const struct device *dev,
				 uart_irq_callback_user_data_t cb,
				 void *user_data)
{
	struct gd32_usart_data *const data = dev->data;

	data->user_cb = cb;
	data->user_data = user_data;
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static DEVICE_API(uart, usart_gd32_driver_api) = {
	.poll_in = usart_gd32_poll_in,
	.poll_out = usart_gd32_poll_out,
	.err_check = usart_gd32_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = usart_gd32_fifo_fill,
	.fifo_read = usart_gd32_fifo_read,
	.irq_tx_enable = usart_gd32_irq_tx_enable,
	.irq_tx_disable = usart_gd32_irq_tx_disable,
	.irq_tx_ready = usart_gd32_irq_tx_ready,
	.irq_tx_complete = usart_gd32_irq_tx_complete,
	.irq_rx_enable = usart_gd32_irq_rx_enable,
	.irq_rx_disable = usart_gd32_irq_rx_disable,
	.irq_rx_ready = usart_gd32_irq_rx_ready,
	.irq_err_enable = usart_gd32_irq_err_enable,
	.irq_err_disable = usart_gd32_irq_err_disable,
	.irq_is_pending = usart_gd32_irq_is_pending,
	.irq_update = usart_gd32_irq_update,
	.irq_callback_set = usart_gd32_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = usart_gd32_async_callback_set,
	.tx = usart_gd32_async_tx,
	.tx_abort = usart_gd32_async_tx_abort,
	.rx_enable = usart_gd32_async_rx_enable,
	.rx_buf_rsp = usart_gd32_rx_buf_rsp,
	.rx_disable = usart_gd32_async_rx_disable,
#endif
};

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define GD32_USART_IRQ_HANDLER(n)						\
	static void usart_gd32_config_func_##n(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			    DT_INST_IRQ(n, priority),				\
			    usart_gd32_isr,					\
			    DEVICE_DT_INST_GET(n),				\
			    0);							\
		irq_enable(DT_INST_IRQN(n));					\
	}
#define GD32_USART_IRQ_HANDLER_FUNC_INIT(n)					\
	.irq_config_func = usart_gd32_config_func_##n,
#else
#define GD32_USART_IRQ_HANDLER(n)
#define GD32_USART_IRQ_HANDLER_FUNC_INIT(n)
#endif

#define GD32_USART_INIT(n)							\
	PINCTRL_DT_INST_DEFINE(n);						\
	GD32_USART_IRQ_HANDLER(n)						\
	static struct gd32_usart_data usart_gd32_data_##n = {			\
		.baud_rate = DT_INST_PROP(n, current_speed),			\
		IF_ENABLED(CONFIG_UART_ASYNC_API,			\
			(GD32_USART_DMA_INIT_FIELDS(n),))	\
	};									\
	static const struct gd32_usart_config usart_gd32_config_##n = {		\
		.reg = DT_INST_REG_ADDR(n),					\
		.clkid = DT_INST_CLOCKS_CELL(n, id),				\
		.reset = RESET_DT_SPEC_INST_GET(n),				\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		.parity = DT_INST_ENUM_IDX(n, parity),				\
		 GD32_USART_IRQ_HANDLER_FUNC_INIT(n)				\
		IF_ENABLED(CONFIG_UART_ASYNC_API,				\
			(.rx_dma_circular =			\
				DT_INST_PROP_OR(n, gd_rx_dma_circular, false),))\
	};									\
	DEVICE_DT_INST_DEFINE(n, usart_gd32_init,				\
			      NULL,						\
			      &usart_gd32_data_##n,				\
			      &usart_gd32_config_##n, PRE_KERNEL_1,		\
			      CONFIG_SERIAL_INIT_PRIORITY,			\
			      &usart_gd32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GD32_USART_INIT)
