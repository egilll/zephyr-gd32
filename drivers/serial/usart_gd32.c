/*
 * Copyright (c) 2021, ATL Electronics
 * Copyright (c) 2025 Aleksandr Senin <al@meshium.net>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_usart

#include <errno.h>

#include <zephyr/cache.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#ifdef CONFIG_DMA
#include <zephyr/drivers/dma.h>
#endif

#include <gd32_usart.h>

#if defined(CONFIG_UART_ASYNC_API) && defined(CONFIG_DMA)
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
	GD32_USART_DMA_STREAM_INIT(index, tx, MEMORY_TO_PERIPHERAL),
#else
#define GD32_USART_DMA_INIT_FIELDS(index)
#endif

/* Unify GD32 HAL USART status register name to USART_STAT */
#ifndef USART_STAT
#define USART_STAT USART_STAT0
#endif

/* Keep ISR bounded even if a status flag sticks high. */
#define GD32_USART_MAX_RX_PER_ISR	16U
#define GD32_USART_MAX_TX_PER_ISR	16U

/* Polling wait timeout for a single character transfer (bounded, baud-scaled). */
#define GD32_USART_POLL_MIN_US		100U
#define GD32_USART_POLL_MAX_US		1000000U

enum gd32_usart_tx_state {
	GD32_USART_TX_IDLE,
	GD32_USART_TX_IRQ,
	GD32_USART_TX_DMA,
	GD32_USART_TX_WAIT_TC,
	GD32_USART_TX_TERMINATING,
};

struct gd32_usart_data;

struct gd32_usart_tx_timeout {
	struct k_work_delayable work;
	struct gd32_usart_data *data;
	atomic_t generation;
};

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
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	enum uart_config_parity parity;
	enum uart_config_stop_bits stop_bits;
	enum uart_config_data_bits data_bits;
	enum uart_config_flow_control flow_ctrl;
	bool initialized;
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
#ifdef CONFIG_UART_ASYNC_API
	uart_callback_t async_cb;
	void *async_user_data;
	const struct device *dev;
	struct {
		uint8_t *buffer;
		size_t buffer_length;
		size_t offset;
		size_t counter;
	} rx;
	struct {
		const uint8_t *buffer;
		size_t buffer_length;
		size_t counter;
	} tx;
	atomic_t tx_state;
	atomic_t tx_generation;
	int32_t rx_timeout;
	struct k_work_delayable rx_timeout_work;
	struct gd32_usart_tx_timeout tx_timeout[2];
	uint8_t tx_timeout_index;
	bool rx_hw_timeout_active;
	struct k_mutex async_lock;
#ifdef CONFIG_DMA
	struct {
		const struct device *dma_dev;
		uint32_t dma_channel;
		struct dma_config dma_cfg;
		struct dma_block_config blk_cfg;
	} dma_rx, dma_tx;
#endif
	bool rx_dma_active;
	bool tx_dma_active;
	uint8_t *rx_next_buffer;
	size_t rx_next_buffer_len;
#endif /* CONFIG_UART_ASYNC_API */
};

static uint32_t usart_gd32_calc_char_timeout_us(const struct device *dev)
{
	const struct gd32_usart_data *data = dev->data;
	uint32_t baud = data->baud_rate;
	uint64_t char_us;
	uint64_t timeout_us;

	if (baud == 0U) {
		return GD32_USART_POLL_MAX_US;
	}

	/* Start + data(9 max with parity) + parity + stop ~= <= 12 bit-times. */
	char_us = DIV_ROUND_UP(12ULL * 1000000ULL, (uint64_t)baud);
	timeout_us = (char_us * 4ULL) + 50ULL;

	return (uint32_t)CLAMP(timeout_us, (uint64_t)GD32_USART_POLL_MIN_US,
			       (uint64_t)GD32_USART_POLL_MAX_US);
}

#ifdef CONFIG_UART_ASYNC_API
#define GD32_USART_DATA_REG_ADDR(base) ((uint32_t)((base) + 0x04U))

static int usart_gd32_cache_result(int rc)
{
	return rc == -ENOTSUP ? 0 : rc;
}

static void usart_gd32_async_isr(const struct device *dev);
static int usart_gd32_async_tx_abort_generation(const struct device *dev,
						atomic_val_t expected_generation);

static inline bool usart_gd32_has_receiver_timeout(uint32_t reg)
{
	/*
	 * On GD32F4xx, USART_RT/STAT1/CTL3 are not available for UART3/4/6/7.
	 * Only enable receiver timeout for peripherals that are documented to have it.
	 * TODO: Verify for other chips
	 */
#ifdef USART0
	if (reg == USART0) {
		return true;
	}
#endif
#ifdef USART1
	if (reg == USART1) {
		return true;
	}
#endif
#ifdef USART2
	if (reg == USART2) {
		return true;
	}
#endif
#ifdef USART5
	if (reg == USART5) {
		return true;
	}
#endif

	return false;
}

static inline uint32_t usart_gd32_rt_baud_clocks_from_timeout_us(uint32_t baud, int32_t timeout_us)
{
	if ((timeout_us <= 0) || (timeout_us == SYS_FOREVER_US) || (baud == 0U)) {
		return 0U;
	}

	uint64_t rt = DIV_ROUND_UP((uint64_t)timeout_us * (uint64_t)baud, 1000000ULL);

	return (uint32_t)CLAMP(rt, 1ULL, 0x00FFFFFFULL);
}

static inline void usart_gd32_clear_idle_and_errors(uint32_t reg)
{
	volatile uint32_t dummy;

	/*
	 * GD32 clears IDLEF/ORERR/NERR/FERR/PERR by reading STAT0 then DATA.
	 * This sequence is also used in the manufacturer's IDLE receive example.
	 */
	dummy = USART_STAT(reg);
	dummy = USART_DATA(reg);
	ARG_UNUSED(dummy);
}

static inline void async_user_callback(struct gd32_usart_data *data,
				       struct uart_event *evt)
{
	if ((evt->type == UART_RX_RDY) && (evt->data.rx.buf != NULL) &&
	    (evt->data.rx.len != 0U)) {
		(void)sys_cache_data_invd_range(evt->data.rx.buf + evt->data.rx.offset,
						evt->data.rx.len);
	}

	if (data->async_cb) {
		data->async_cb(data->dev, evt, data->async_user_data);
	}
}

static inline void async_evt_rx_rdy_cyclic(struct gd32_usart_data *data, size_t new_pos)
{
	const size_t buf_len = data->rx.buffer_length;
	size_t start = data->rx.offset;

	if ((data->rx.buffer == NULL) || (buf_len == 0U)) {
		return;
	}

	if (start >= buf_len) {
		start = 0U;
	}
	if (new_pos > buf_len) {
		new_pos = buf_len;
	}

	if (new_pos == start) {
		return;
	}

	if (new_pos == buf_len) {
		struct uart_event evt = {
			.type = UART_RX_RDY,
			.data.rx = {
				.buf = data->rx.buffer,
				.len = buf_len - start,
				.offset = start,
			},
		};

		if (evt.data.rx.len != 0U) {
			async_user_callback(data, &evt);
		}
		data->rx.offset = 0U;
		return;
	}

	if (new_pos > start) {
		struct uart_event evt = {
			.type = UART_RX_RDY,
			.data.rx = {
				.buf = data->rx.buffer,
				.len = new_pos - start,
				.offset = start,
			},
		};

		async_user_callback(data, &evt);
		data->rx.offset = new_pos;
		return;
	}

	/* Wrap-around: deliver tail then head. */
	struct uart_event evt1 = {
		.type = UART_RX_RDY,
		.data.rx = {
			.buf = data->rx.buffer,
			.len = buf_len - start,
			.offset = start,
		},
	};
	if (evt1.data.rx.len != 0U) {
		async_user_callback(data, &evt1);
	}

	if (new_pos != 0U) {
		struct uart_event evt2 = {
			.type = UART_RX_RDY,
			.data.rx = {
				.buf = data->rx.buffer,
				.len = new_pos,
				.offset = 0U,
			},
		};
		async_user_callback(data, &evt2);
	}

	data->rx.offset = new_pos;
}

static inline void async_evt_rx_rdy(struct gd32_usart_data *data)
{
	if (data->rx.counter <= data->rx.offset) {
		return;
	}

	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx = {
			.buf = data->rx.buffer,
			.len = data->rx.counter - data->rx.offset,
			.offset = data->rx.offset,
		},
	};

	data->rx.offset = data->rx.counter;
	async_user_callback(data, &evt);
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
				.buf = data->rx.buffer,
				.len = data->rx.counter,
				.offset = 0,
			},
		},
	};

	async_user_callback(data, &evt);
}

static inline void async_evt_tx_done(struct gd32_usart_data *data)
{
	(void)k_work_cancel_delayable(&data->tx_timeout[data->tx_timeout_index].work);

	struct uart_event evt = {
		.type = UART_TX_DONE,
		.data.tx = {
			.buf = data->tx.buffer,
			.len = data->tx.counter,
		},
	};

	data->tx.buffer = NULL;
	data->tx.buffer_length = 0U;
	data->tx.counter = 0U;
	data->tx_dma_active = false;
	atomic_inc(&data->tx_generation);
	atomic_set(&data->tx_state, GD32_USART_TX_IDLE);

	async_user_callback(data, &evt);
}

static inline void async_evt_tx_aborted(struct gd32_usart_data *data)
{
	(void)k_work_cancel_delayable(&data->tx_timeout[data->tx_timeout_index].work);

	struct uart_event evt = {
		.type = UART_TX_ABORTED,
		.data.tx = {
			.buf = data->tx.buffer,
			.len = data->tx.counter,
		},
	};

	data->tx.buffer = NULL;
	data->tx.buffer_length = 0U;
	data->tx.counter = 0U;
	data->tx_dma_active = false;
	atomic_inc(&data->tx_generation);
	atomic_set(&data->tx_state, GD32_USART_TX_IDLE);

	async_user_callback(data, &evt);
}

static bool usart_gd32_tx_claim_terminal(struct gd32_usart_data *data)
{
	atomic_val_t state = atomic_get(&data->tx_state);

	while ((state == GD32_USART_TX_IRQ) || (state == GD32_USART_TX_DMA) ||
	       (state == GD32_USART_TX_WAIT_TC)) {
		if (atomic_cas(&data->tx_state, state, GD32_USART_TX_TERMINATING)) {
			return true;
		}
		state = atomic_get(&data->tx_state);
	}

	return false;
}

static inline bool build_rx_rdy_event(struct gd32_usart_data *data, struct uart_event *evt)
{
	if (data->rx.counter <= data->rx.offset) {
		return false;
	}

	evt->type = UART_RX_RDY;
	evt->data.rx.buf = data->rx.buffer;
	evt->data.rx.len = data->rx.counter - data->rx.offset;
	evt->data.rx.offset = data->rx.offset;
	data->rx.offset = data->rx.counter;
	return true;
}

static inline bool async_evt_rx_rdy_if_pending(struct gd32_usart_data *data)
{
	struct uart_event evt = {};

	if (!build_rx_rdy_event(data, &evt)) {
		return false;
	}

	async_user_callback(data, &evt);
	return true;
}

static inline void async_timer_start(struct k_work_delayable *work, int32_t timeout)
{
	if ((timeout != SYS_FOREVER_US) && (timeout != 0)) {
		k_work_reschedule(work, K_USEC(timeout));
	}
}

static inline void async_tx_timer_start(struct k_work_delayable *work, int32_t timeout)
{
	if (timeout != SYS_FOREVER_US) {
		(void)k_work_reschedule(work, K_USEC(timeout));
	}
}

#ifdef CONFIG_DMA
static inline bool usart_gd32_dma_rx_pos_get(struct gd32_usart_data *data, size_t *pos)
{
	struct dma_status stat;

	if ((data->dma_rx.dma_dev == NULL) ||
	    (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat) != 0)) {
		return false;
	}

	if (stat.pending_length >= data->rx.buffer_length) {
		*pos = 0U;
		return true;
	}

	*pos = data->rx.buffer_length - stat.pending_length;
	return true;
}

static void usart_gd32_async_dma_rx_flush(const struct device *dev, int dma_status)
{
	struct gd32_usart_data *data = dev->data;
	size_t pos = 0U;

	if (data->rx.buffer_length == 0U) {
		return;
	}

	if (!usart_gd32_dma_rx_pos_get(data, &pos)) {
		if (dma_status == DMA_STATUS_BLOCK) {
			pos = data->rx.buffer_length / 2U;
		} else if (dma_status == DMA_STATUS_COMPLETE) {
			pos = data->rx.buffer_length;
		} else {
			return;
		}
	}

	if (data->dma_rx.dma_cfg.cyclic) {
		async_evt_rx_rdy_cyclic(data, pos);
		return;
	}

	if (pos < data->rx.counter) {
		/* DMA was likely reloaded; ignore transient backwards position. */
		return;
	}

	data->rx.counter = pos;
	async_evt_rx_rdy(data);
}
#endif /* CONFIG_DMA */
#endif /* CONFIG_UART_ASYNC_API */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
static void usart_gd32_isr(const struct device *dev)
{
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	struct gd32_usart_data *const data = dev->data;

	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
		return;
	}
#endif

#ifdef CONFIG_UART_ASYNC_API
	usart_gd32_async_isr(dev);
#endif
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */

#ifdef CONFIG_UART_ASYNC_API
#ifdef CONFIG_DMA
static void usart_gd32_dma_rx_cb(const struct device *dma_dev, void *user_data,
				 uint32_t channel, int status)
{
	const struct device *dev = user_data;
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);

	if (!data->rx_dma_active || (data->rx.buffer == NULL) ||
	    (data->rx.buffer_length == 0U)) {
		return;
	}

	if (status < 0) {
		struct dma_status stat;

		(void)k_work_cancel_delayable(&data->rx_timeout_work);
		if (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat) == 0) {
			data->rx.counter = data->rx.buffer_length -
					   MIN(stat.pending_length, data->rx.buffer_length);
		} else {
			data->rx.counter = data->rx.buffer_length;
		}

		async_evt_rx_stopped(data, UART_ERROR_OVERRUN);
		dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
		usart_dma_receive_config(cfg->reg, USART_DENR_DISABLE);
		usart_interrupt_disable(cfg->reg, USART_INT_IDLE);
		usart_interrupt_disable(cfg->reg, USART_INT_ERR);
		usart_interrupt_disable(cfg->reg, USART_INT_PERR);
		async_evt_rx_rdy(data);
		async_evt_rx_buf_released(data, data->rx.buffer);
		if (data->rx_next_buffer_len) {
			async_evt_rx_buf_released(data, data->rx_next_buffer);
			data->rx_next_buffer = NULL;
			data->rx_next_buffer_len = 0U;
		}
		data->rx.buffer = NULL;
		data->rx.buffer_length = 0U;
		data->rx.offset = 0U;
		data->rx.counter = 0U;
		data->rx_dma_active = false;
		async_evt_rx_disabled(data);
		return;
	}

	if (data->rx.buffer_length == 0U) {
		return;
	}

	if (!data->dma_rx.dma_cfg.cyclic && (status == DMA_STATUS_COMPLETE)) {
		struct dma_status stat;

		if ((dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
				    &stat) != 0) || stat.busy || (stat.pending_length != 0U)) {
			/* A retired-buffer duplicate must not retire the next buffer. */
			return;
		}
	}

	usart_gd32_async_dma_rx_flush(dev, status);

	if (data->dma_rx.dma_cfg.cyclic) {
		return;
	}

	uint8_t *released = data->rx.buffer;

	if (data->rx_next_buffer_len) {
		int ret;

		data->rx.buffer = data->rx_next_buffer;
		data->rx.buffer_length = data->rx_next_buffer_len;
		data->rx_next_buffer = NULL;
		data->rx_next_buffer_len = 0U;
		data->rx.offset = 0U;
		data->rx.counter = 0U;
		usart_gd32_clear_idle_and_errors(cfg->reg);
		ret = dma_reload(data->dma_rx.dma_dev, data->dma_rx.dma_channel,
				 GD32_USART_DATA_REG_ADDR(cfg->reg),
				 (uint32_t)data->rx.buffer,
				 data->rx.buffer_length);
		if (ret == 0) {
			ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
		}
		async_evt_rx_buf_released(data, released);
		if (ret != 0) {
			async_evt_rx_stopped(data, UART_ERROR_OVERRUN);
			(void)dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
			usart_dma_receive_config(cfg->reg, USART_DENR_DISABLE);
			usart_interrupt_disable(cfg->reg, USART_INT_IDLE);
			usart_interrupt_disable(cfg->reg, USART_INT_ERR);
			usart_interrupt_disable(cfg->reg, USART_INT_PERR);
			async_evt_rx_buf_released(data, data->rx.buffer);
			data->rx.buffer = NULL;
			data->rx.buffer_length = 0U;
			data->rx.offset = 0U;
			data->rx.counter = 0U;
			data->rx_dma_active = false;
			async_evt_rx_disabled(data);
			return;
		}
		async_evt_rx_buf_request(data);
	} else {
		dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
		async_evt_rx_buf_released(data, released);
		usart_dma_receive_config(cfg->reg, USART_DENR_DISABLE);
		usart_interrupt_disable(cfg->reg, USART_INT_IDLE);
		usart_interrupt_disable(cfg->reg, USART_INT_ERR);
		usart_interrupt_disable(cfg->reg, USART_INT_PERR);
		data->rx.buffer = NULL;
		data->rx.buffer_length = 0U;
		data->rx.offset = 0U;
		data->rx.counter = 0U;
		data->rx_dma_active = false;
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

	if (atomic_get(&data->tx_state) != GD32_USART_TX_DMA) {
		return;
	}

	struct dma_status stat;
	size_t tx_len = data->tx.buffer_length;

	if ((status == DMA_STATUS_COMPLETE) &&
	    ((dma_get_status(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &stat) != 0) ||
	     stat.busy || (stat.pending_length != 0U))) {
		/* Ignore a completion that does not describe the currently configured block. */
		return;
	}

	if (status != DMA_STATUS_COMPLETE) {
		if (!atomic_cas(&data->tx_state, GD32_USART_TX_DMA,
				GD32_USART_TX_TERMINATING)) {
			return;
		}
		if ((tx_len > 0U) &&
		    (dma_get_status(data->dma_tx.dma_dev, data->dma_tx.dma_channel,
				    &stat) == 0)) {
			data->tx.counter = tx_len - MIN(stat.pending_length, tx_len);
		}
		(void)dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
		usart_dma_transmit_config(cfg->reg, USART_DENT_DISABLE);
		usart_interrupt_disable(cfg->reg, USART_INT_TC);
		async_evt_tx_aborted(data);
		return;
	}

	if (!atomic_cas(&data->tx_state, GD32_USART_TX_DMA,
			GD32_USART_TX_WAIT_TC)) {
		return;
	}

	data->tx.counter = tx_len;
	(void)dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
	usart_dma_transmit_config(cfg->reg, USART_DENT_DISABLE);
	data->tx_dma_active = false;
	/* DMA completion only filled USART_DATA; TC is the wire-idle condition. */
	usart_interrupt_enable(cfg->reg, USART_INT_TC);
}
#endif /* CONFIG_DMA */

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

static void usart_gd32_async_rx_timeout_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct gd32_usart_data *data = CONTAINER_OF(dwork, struct gd32_usart_data, rx_timeout_work);
	const struct device *dev = data->dev;
	const struct gd32_usart_config *cfg = dev->config;
	struct uart_event evt = {};
	bool send_evt = false;

	__ASSERT_NO_MSG(!k_is_in_isr());
	k_mutex_lock(&data->async_lock, K_FOREVER);

	if (data->rx_hw_timeout_active) {
		k_mutex_unlock(&data->async_lock);
		return;
	}

	if (data->rx_dma_active) {
		k_mutex_unlock(&data->async_lock);
		return;
	}

	usart_interrupt_disable(cfg->reg, USART_INT_RBNE);
	usart_interrupt_disable(cfg->reg, USART_INT_ERR);
	usart_interrupt_disable(cfg->reg, USART_INT_PERR);

	if (data->rx.counter > data->rx.offset) {
		evt.type = UART_RX_RDY;
		evt.data.rx.buf = data->rx.buffer;
		evt.data.rx.len = data->rx.counter - data->rx.offset;
		evt.data.rx.offset = data->rx.offset;
		data->rx.offset = data->rx.counter;
		send_evt = true;
	}

	usart_interrupt_enable(cfg->reg, USART_INT_ERR);
	usart_interrupt_enable(cfg->reg, USART_INT_PERR);
	if (data->rx.buffer_length != 0U) {
		usart_interrupt_enable(cfg->reg, USART_INT_RBNE);
	}
	k_mutex_unlock(&data->async_lock);

	if (send_evt) {
		async_user_callback(data, &evt);
	}
}

static void usart_gd32_async_tx_timeout_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct gd32_usart_tx_timeout *timeout =
		CONTAINER_OF(dwork, struct gd32_usart_tx_timeout, work);
	struct gd32_usart_data *data = timeout->data;
	atomic_val_t generation = atomic_get(&timeout->generation);

	(void)usart_gd32_async_tx_abort_generation(data->dev, generation);
}

static void usart_gd32_async_rx_buf_done(const struct device *dev, bool request_next_buf)
{
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;
	uint8_t *released = data->rx.buffer;

	(void)k_work_cancel_delayable(&data->rx_timeout_work);
	async_evt_rx_rdy(data);
	async_evt_rx_buf_released(data, released);

	if (data->rx_next_buffer_len) {
		data->rx.buffer = data->rx_next_buffer;
		data->rx.buffer_length = data->rx_next_buffer_len;
		data->rx.offset = 0U;
		data->rx.counter = 0U;
		data->rx_next_buffer = NULL;
		data->rx_next_buffer_len = 0U;
		if (request_next_buf) {
			async_evt_rx_buf_request(data);
		}
		return;
	}

	usart_interrupt_disable(cfg->reg, USART_INT_RBNE);
	usart_interrupt_disable(cfg->reg, USART_INT_ERR);
	usart_interrupt_disable(cfg->reg, USART_INT_PERR);
	usart_interrupt_disable(cfg->reg, USART_INT_IDLE);
	if (data->rx_hw_timeout_active && usart_gd32_has_receiver_timeout(cfg->reg)) {
		usart_interrupt_disable(cfg->reg, USART_INT_RT);
		usart_receiver_timeout_disable(cfg->reg);
		usart_flag_clear(cfg->reg, USART_FLAG_RT);
		data->rx_hw_timeout_active = false;
	}
	data->rx.buffer = NULL;
	data->rx.buffer_length = 0U;
	data->rx.offset = 0U;
	data->rx.counter = 0U;
	async_evt_rx_disabled(data);
}

static void usart_gd32_async_isr(const struct device *dev)
{
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;
	uint32_t stat0 = USART_STAT(cfg->reg);

	if (data->rx_dma_active &&
	    (stat0 & (USART_STAT0_ORERR | USART_STAT0_NERR | USART_STAT0_FERR | USART_STAT0_PERR))) {
		/*
		 * Clear line error flags to avoid interrupt storms in DMA mode.
		 * (Errors are also observable via uart_err_check().)
		 */
		usart_gd32_clear_idle_and_errors(cfg->reg);
		stat0 = USART_STAT(cfg->reg);
	}

	if (!data->rx_dma_active &&
	    (stat0 & (USART_STAT0_ORERR | USART_STAT0_NERR | USART_STAT0_FERR | USART_STAT0_PERR)) &&
	    !usart_flag_get(cfg->reg, USART_FLAG_RBNE)) {
		/*
		 * Clear line errors that can latch without RBNE, to avoid repeated IRQ entry.
		 * When RBNE is set, the loop below clears errors as part of draining DATA.
		 */
		usart_gd32_clear_idle_and_errors(cfg->reg);
	}

	if (!data->rx_dma_active && (data->rx.buffer_length != 0U) &&
	    usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_RBNE)) {
		for (uint32_t i = 0; i < GD32_USART_MAX_RX_PER_ISR &&
				     usart_flag_get(cfg->reg, USART_FLAG_RBNE);
		     i++) {
			stat0 = USART_STAT(cfg->reg);
			uint8_t byte = (uint8_t)USART_DATA(cfg->reg);

			if (stat0 & (USART_STAT0_ORERR | USART_STAT0_NERR |
				     USART_STAT0_FERR | USART_STAT0_PERR)) {
				/* Discard bytes received with line errors. */
				continue;
			}

			if (data->rx.counter >= data->rx.buffer_length) {
				usart_gd32_async_rx_buf_done(dev, true);
				if (data->rx.buffer_length == 0U) {
					break;
				}
			}

			data->rx.buffer[data->rx.counter++] = byte;
			if (!data->rx_hw_timeout_active) {
				async_timer_start(&data->rx_timeout_work, data->rx_timeout);
			}

			if (data->rx.counter >= data->rx.buffer_length) {
				usart_gd32_async_rx_buf_done(dev, true);
				if (data->rx.buffer_length == 0U) {
					break;
				}
			}
		}
	}

	if (!data->rx_dma_active && data->rx_hw_timeout_active &&
	    usart_gd32_has_receiver_timeout(cfg->reg) &&
	    (data->rx.buffer_length != 0U) &&
	    usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_RT)) {
		usart_flag_clear(cfg->reg, USART_FLAG_RT);
		(void)async_evt_rx_rdy_if_pending(data);
	}

	if (data->rx_dma_active && (data->rx.buffer_length != 0U) &&
	    usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_IDLE)) {
		usart_gd32_clear_idle_and_errors(cfg->reg);
		usart_gd32_async_dma_rx_flush(dev, 0);
	}

	if ((atomic_get(&data->tx_state) == GD32_USART_TX_IRQ) &&
	    (data->tx.buffer_length != 0U)) {
		if (usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_TBE)) {
			for (uint32_t i = 0; i < GD32_USART_MAX_TX_PER_ISR &&
					     usart_flag_get(cfg->reg, USART_FLAG_TBE) &&
					     (data->tx.counter < data->tx.buffer_length);
			     i++) {
				usart_data_transmit(cfg->reg, data->tx.buffer[data->tx.counter++]);
			}

			if (data->tx.counter >= data->tx.buffer_length) {
				usart_interrupt_disable(cfg->reg, USART_INT_TBE);
				usart_flag_clear(cfg->reg, USART_FLAG_TC);
				usart_interrupt_enable(cfg->reg, USART_INT_TC);
			}
		}

		if (usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_TC) &&
		    (data->tx.counter >= data->tx.buffer_length)) {
			usart_interrupt_disable(cfg->reg, USART_INT_TC);
			usart_flag_clear(cfg->reg, USART_FLAG_TC);
			if (atomic_cas(&data->tx_state, GD32_USART_TX_IRQ,
				       GD32_USART_TX_TERMINATING)) {
				data->tx.counter = data->tx.buffer_length;
				async_evt_tx_done(data);
			}
		}
	}

	if ((atomic_get(&data->tx_state) == GD32_USART_TX_WAIT_TC) &&
	    usart_interrupt_flag_get(cfg->reg, USART_INT_FLAG_TC)) {
		usart_interrupt_disable(cfg->reg, USART_INT_TC);
		usart_flag_clear(cfg->reg, USART_FLAG_TC);
		if (atomic_cas(&data->tx_state, GD32_USART_TX_WAIT_TC,
			       GD32_USART_TX_TERMINATING)) {
			data->tx.counter = data->tx.buffer_length;
			async_evt_tx_done(data);
		}
	}
}

static int usart_gd32_async_tx(const struct device *dev, const uint8_t *tx_data,
				 size_t buf_size, int32_t timeout)
{
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;
	int timeout_index = -1;

	if ((tx_data == NULL) || (buf_size == 0U) ||
	    ((timeout < 0) && (timeout != SYS_FOREVER_US))) {
		return -EINVAL;
	}

	__ASSERT_NO_MSG(!k_is_in_isr());
	k_mutex_lock(&data->async_lock, K_FOREVER);

	if ((data->tx.buffer_length != 0U) || (atomic_get(&data->tx_state) != GD32_USART_TX_IDLE)) {
		k_mutex_unlock(&data->async_lock);
		return -EBUSY;
	}
	for (size_t i = 0U; i < ARRAY_SIZE(data->tx_timeout); i++) {
		if (k_work_delayable_busy_get(&data->tx_timeout[i].work) == 0) {
			timeout_index = (int)i;
			break;
		}
	}
	if (timeout_index < 0) {
		k_mutex_unlock(&data->async_lock);
		return -EBUSY;
	}

	usart_interrupt_disable(cfg->reg, USART_INT_TBE);
	usart_interrupt_disable(cfg->reg, USART_INT_TC);

	data->tx.buffer = tx_data;
	data->tx.buffer_length = buf_size;
	data->tx.counter = 0U;
	atomic_inc(&data->tx_generation);
	data->tx_timeout_index = (uint8_t)timeout_index;
	atomic_set(&data->tx_timeout[timeout_index].generation, atomic_get(&data->tx_generation));

#ifdef CONFIG_DMA
	if ((data->dma_tx.dma_dev != NULL) && (buf_size <= UINT16_MAX)) {
		int ret;

		data->tx_dma_active = true;
		data->dma_tx.blk_cfg.source_address = (uint32_t)data->tx.buffer;
		data->dma_tx.blk_cfg.block_size = data->tx.buffer_length;

		ret = dma_config(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &data->dma_tx.dma_cfg);
		if (ret < 0) {
			goto dma_start_failed;
		}

		ret = usart_gd32_cache_result(sys_cache_data_flush_range(
			(void *)data->tx.buffer, data->tx.buffer_length));
		if (ret < 0) {
			goto dma_start_failed;
		}

		usart_flag_clear(cfg->reg, USART_FLAG_TC);
		atomic_set(&data->tx_state, GD32_USART_TX_DMA);
		async_tx_timer_start(&data->tx_timeout[timeout_index].work, timeout);
		usart_dma_transmit_config(cfg->reg, USART_DENT_ENABLE);
		ret = dma_start(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
		if (ret < 0) {
			(void)k_work_cancel_delayable(&data->tx_timeout[timeout_index].work);
			usart_dma_transmit_config(cfg->reg, USART_DENT_DISABLE);
dma_start_failed:
			atomic_inc(&data->tx_generation);
			atomic_set(&data->tx_state, GD32_USART_TX_IDLE);
			data->tx.buffer = NULL;
			data->tx.buffer_length = 0U;
			data->tx.counter = 0U;
			data->tx_dma_active = false;
		}

		k_mutex_unlock(&data->async_lock);
		return ret;
	}
#endif

	data->tx_dma_active = false;
	atomic_set(&data->tx_state, GD32_USART_TX_IRQ);
	async_tx_timer_start(&data->tx_timeout[timeout_index].work, timeout);
	if (usart_flag_get(cfg->reg, USART_FLAG_TBE)) {
		usart_data_transmit(cfg->reg, data->tx.buffer[data->tx.counter++]);
	}

	if (data->tx.counter < data->tx.buffer_length) {
		usart_interrupt_enable(cfg->reg, USART_INT_TBE);
	} else {
		usart_flag_clear(cfg->reg, USART_FLAG_TC);
		usart_interrupt_enable(cfg->reg, USART_INT_TC);
	}

	k_mutex_unlock(&data->async_lock);
	return 0;
}

static int usart_gd32_async_tx_abort_generation(const struct device *dev,
						atomic_val_t expected_generation)
{
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;
	struct uart_event evt;

	__ASSERT_NO_MSG(!k_is_in_isr());
	k_mutex_lock(&data->async_lock, K_FOREVER);

	if ((atomic_get(&data->tx_generation) != expected_generation) ||
	    !usart_gd32_tx_claim_terminal(data)) {
		k_mutex_unlock(&data->async_lock);
		return -EFAULT;
	}

	usart_interrupt_disable(cfg->reg, USART_INT_TBE);
	usart_interrupt_disable(cfg->reg, USART_INT_TC);

#ifdef CONFIG_DMA
	if (data->tx_dma_active) {
		struct dma_status stat;

		if (dma_get_status(data->dma_tx.dma_dev, data->dma_tx.dma_channel, &stat) == 0) {
			data->tx.counter = data->tx.buffer_length -
					   MIN(stat.pending_length, data->tx.buffer_length);
		}

		(void)dma_stop(data->dma_tx.dma_dev, data->dma_tx.dma_channel);
		usart_dma_transmit_config(cfg->reg, USART_DENT_DISABLE);
	}
#endif

	(void)k_work_cancel_delayable(&data->tx_timeout[data->tx_timeout_index].work);
	evt = (struct uart_event){
		.type = UART_TX_ABORTED,
		.data.tx = {
			.buf = data->tx.buffer,
			.len = data->tx.counter,
		},
	};
	data->tx.buffer = NULL;
	data->tx.buffer_length = 0U;
	data->tx.counter = 0U;
	data->tx_dma_active = false;
	atomic_inc(&data->tx_generation);
	atomic_set(&data->tx_state, GD32_USART_TX_IDLE);
	k_mutex_unlock(&data->async_lock);
	async_user_callback(data, &evt);

	return 0;
}

static int usart_gd32_async_tx_abort(const struct device *dev)
{
	struct gd32_usart_data *data = dev->data;

	return usart_gd32_async_tx_abort_generation(dev, atomic_get(&data->tx_generation));
}

static int usart_gd32_async_rx_enable(const struct device *dev, uint8_t *buf,
					 size_t len, int32_t timeout)
{
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;
	int ret = 0;
	bool request_next = true;

	__ASSERT_NO_MSG(!k_is_in_isr());
	k_mutex_lock(&data->async_lock, K_FOREVER);

	if (data->rx.buffer_length != 0U) {
		k_mutex_unlock(&data->async_lock);
		return -EBUSY;
	}

	if ((buf == NULL) || (len == 0U) ||
	    ((timeout < 0) && (timeout != SYS_FOREVER_US))) {
		k_mutex_unlock(&data->async_lock);
		return -EINVAL;
	}

	usart_interrupt_disable(cfg->reg, USART_INT_RBNE);
	usart_interrupt_disable(cfg->reg, USART_INT_ERR);
	usart_interrupt_disable(cfg->reg, USART_INT_PERR);
	usart_interrupt_disable(cfg->reg, USART_INT_IDLE);
	if (usart_gd32_has_receiver_timeout(cfg->reg)) {
		usart_interrupt_disable(cfg->reg, USART_INT_RT);
		usart_receiver_timeout_disable(cfg->reg);
		usart_flag_clear(cfg->reg, USART_FLAG_RT);
	}

	data->rx.buffer = buf;
	data->rx.buffer_length = len;
	data->rx.offset = 0U;
	data->rx.counter = 0U;
	data->rx_timeout = timeout;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0U;
	data->rx_dma_active = false;
	data->rx_hw_timeout_active = false;

#ifdef CONFIG_DMA
	if ((data->dma_rx.dma_dev != NULL) && (len <= UINT16_MAX)) {
		ret = usart_gd32_cache_result(sys_cache_data_flush_and_invd_range(buf, len));
		if (ret < 0) {
			data->rx.buffer = NULL;
			data->rx.buffer_length = 0U;
			k_mutex_unlock(&data->async_lock);
			return ret;
		}

		data->rx_dma_active = true;
		data->dma_rx.dma_cfg.cyclic = cfg->rx_dma_circular ? 1U : 0U;
		data->dma_rx.blk_cfg.dest_address = (uint32_t)data->rx.buffer;
		data->dma_rx.blk_cfg.block_size = data->rx.buffer_length;

		ret = dma_config(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &data->dma_rx.dma_cfg);
		if (ret < 0) {
			data->rx.buffer = NULL;
			data->rx.buffer_length = 0U;
			data->rx.offset = 0U;
			data->rx.counter = 0U;
			data->rx_dma_active = false;
			usart_interrupt_enable(cfg->reg, USART_INT_ERR);
			usart_interrupt_enable(cfg->reg, USART_INT_PERR);
			k_mutex_unlock(&data->async_lock);
			return ret;
		}

		usart_dma_receive_config(cfg->reg, USART_DENR_ENABLE);
		ret = dma_start(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
		if (ret < 0) {
			usart_dma_receive_config(cfg->reg, USART_DENR_DISABLE);
			data->rx.buffer = NULL;
			data->rx.buffer_length = 0U;
			data->rx.offset = 0U;
			data->rx.counter = 0U;
			data->rx_dma_active = false;
			usart_interrupt_enable(cfg->reg, USART_INT_ERR);
			usart_interrupt_enable(cfg->reg, USART_INT_PERR);
			k_mutex_unlock(&data->async_lock);
			return ret;
		}
	}
#endif

	usart_interrupt_enable(cfg->reg, USART_INT_ERR);
	usart_interrupt_enable(cfg->reg, USART_INT_PERR);

	if (data->rx_dma_active) {
		usart_gd32_clear_idle_and_errors(cfg->reg);
		usart_interrupt_enable(cfg->reg, USART_INT_IDLE);
	} else {
		usart_interrupt_enable(cfg->reg, USART_INT_RBNE);

		if (usart_gd32_has_receiver_timeout(cfg->reg)) {
			uint32_t rt = usart_gd32_rt_baud_clocks_from_timeout_us(data->baud_rate, timeout);

			if (rt != 0U) {
				usart_receiver_timeout_threshold_config(cfg->reg, rt);
				usart_flag_clear(cfg->reg, USART_FLAG_RT);
				usart_receiver_timeout_enable(cfg->reg);
				usart_interrupt_enable(cfg->reg, USART_INT_RT);
				data->rx_hw_timeout_active = true;
			}
		}
	}

	k_mutex_unlock(&data->async_lock);

#ifdef CONFIG_DMA
	if (data->rx_dma_active && data->dma_rx.dma_cfg.cyclic) {
		request_next = false;
	}
#endif

	if (request_next) {
		async_evt_rx_buf_request(data);
	}
	return ret;
}

static int usart_gd32_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	struct gd32_usart_data *data = dev->data;
	int ret = 0;

	__ASSERT_NO_MSG(!k_is_in_isr());
	k_mutex_lock(&data->async_lock, K_FOREVER);

	if ((buf == NULL) || (len == 0U)) {
		ret = -EINVAL;
		goto out;
	}

	if (data->rx.buffer_length == 0U) {
		ret = -EACCES;
		goto out;
	}

#ifdef CONFIG_DMA
	if (data->rx_dma_active && data->dma_rx.dma_cfg.cyclic) {
		ret = -EACCES;
		goto out;
	}
#endif

	if (data->rx_next_buffer_len != 0U) {
		ret = -EBUSY;
		goto out;
	}

#ifdef CONFIG_DMA
	if (data->rx_dma_active) {
		if (len > UINT16_MAX) {
			ret = -EINVAL;
			goto out;
		}
		ret = usart_gd32_cache_result(sys_cache_data_flush_and_invd_range(buf, len));
		if (ret < 0) {
			goto out;
		}
	}
#endif

	unsigned int key = irq_lock();

	data->rx_next_buffer = buf;
	data->rx_next_buffer_len = len;
	irq_unlock(key);

out:
	k_mutex_unlock(&data->async_lock);
	return ret;
}

static int usart_gd32_async_rx_disable(const struct device *dev)
{
	struct gd32_usart_data *data = dev->data;
	const struct gd32_usart_config *cfg = dev->config;
	uint8_t *released_first = NULL;
	struct uart_event rdy_events[2] = {};
	int rdy_count = 0;
	uint8_t *release_bufs[3] = {};
	int release_count = 0;
	bool send_disabled = false;

	__ASSERT_NO_MSG(!k_is_in_isr());
	k_mutex_lock(&data->async_lock, K_FOREVER);

	if (data->rx.buffer_length == 0U) {
		k_mutex_unlock(&data->async_lock);
		return -EFAULT;
	}

	usart_interrupt_disable(cfg->reg, USART_INT_RBNE);
	usart_interrupt_disable(cfg->reg, USART_INT_ERR);
	usart_interrupt_disable(cfg->reg, USART_INT_PERR);
	usart_interrupt_disable(cfg->reg, USART_INT_IDLE);

	if (usart_gd32_has_receiver_timeout(cfg->reg)) {
		usart_interrupt_disable(cfg->reg, USART_INT_RT);
		usart_receiver_timeout_disable(cfg->reg);
		usart_flag_clear(cfg->reg, USART_FLAG_RT);
	}
	data->rx_hw_timeout_active = false;

	struct k_work_sync sync;

	(void)k_work_cancel_delayable_sync(&data->rx_timeout_work, &sync);

#ifdef CONFIG_DMA
	if (data->rx_dma_active) {
		struct dma_status stat;
		size_t pos = 0U;

		dma_stop(data->dma_rx.dma_dev, data->dma_rx.dma_channel);
		usart_dma_receive_config(cfg->reg, USART_DENR_DISABLE);
		if (dma_get_status(data->dma_rx.dma_dev, data->dma_rx.dma_channel, &stat) == 0) {
			pos = (stat.pending_length >= data->rx.buffer_length) ?
				0U : (data->rx.buffer_length - stat.pending_length);
		} else {
			pos = data->rx.buffer_length;
		}

		if (data->dma_rx.dma_cfg.cyclic) {
			size_t start = data->rx.offset;
			const size_t buf_len = data->rx.buffer_length;

			if (start >= buf_len) {
				start = 0U;
			}
			if (pos > buf_len) {
				pos = buf_len;
			}

			if ((pos != start) && (rdy_count < (int)ARRAY_SIZE(rdy_events))) {
				if (pos >= start) {
					rdy_events[rdy_count++] = (struct uart_event){
						.type = UART_RX_RDY,
						.data.rx = {
							.buf = data->rx.buffer,
							.len = pos - start,
							.offset = start,
						},
					};
				} else {
					rdy_events[rdy_count++] = (struct uart_event){
						.type = UART_RX_RDY,
						.data.rx = {
							.buf = data->rx.buffer,
							.len = buf_len - start,
							.offset = start,
						},
					};
					if ((pos != 0U) && (rdy_count < (int)ARRAY_SIZE(rdy_events))) {
						rdy_events[rdy_count++] = (struct uart_event){
							.type = UART_RX_RDY,
							.data.rx = {
								.buf = data->rx.buffer,
								.len = pos,
								.offset = 0U,
							},
						};
					}
				}
			}

			data->rx.offset = pos;
			data->rx.counter = pos;
		} else {
			data->rx.counter = pos;
		}

		data->rx_dma_active = false;
	} else
#endif
	{
		const size_t max_reads = MIN(data->rx.buffer_length + data->rx_next_buffer_len + 32U,
					     512U);

		for (size_t i = 0; i < max_reads && usart_flag_get(cfg->reg, USART_FLAG_RBNE); i++) {
			uint32_t stat0 = USART_STAT(cfg->reg);
			uint8_t byte = (uint8_t)USART_DATA(cfg->reg);

			if (stat0 & (USART_STAT0_ORERR | USART_STAT0_NERR |
				     USART_STAT0_FERR | USART_STAT0_PERR)) {
				continue;
			}

			if (data->rx.counter >= data->rx.buffer_length) {
				if (data->rx_next_buffer_len == 0U) {
					continue;
				}

				if (build_rx_rdy_event(data, &rdy_events[rdy_count])) {
					++rdy_count;
				}
				released_first = data->rx.buffer;
				data->rx.buffer = data->rx_next_buffer;
				data->rx.buffer_length = data->rx_next_buffer_len;
				data->rx.offset = 0U;
				data->rx.counter = 0U;
				data->rx_next_buffer = NULL;
				data->rx_next_buffer_len = 0U;
			}

			data->rx.buffer[data->rx.counter++] = byte;
		}
	}

	if (build_rx_rdy_event(data, &rdy_events[rdy_count])) {
		++rdy_count;
	}
	if (released_first != NULL) {
		release_bufs[release_count++] = released_first;
	}
	if (data->rx.buffer != NULL) {
		release_bufs[release_count++] = data->rx.buffer;
	}

	if (data->rx_next_buffer_len) {
		release_bufs[release_count++] = data->rx_next_buffer;
	}

	data->rx.buffer = NULL;
	data->rx.buffer_length = 0U;
	data->rx.offset = 0U;
	data->rx.counter = 0U;
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0U;
	send_disabled = true;

	k_mutex_unlock(&data->async_lock);

	for (int i = 0; i < rdy_count; ++i) {
		async_user_callback(data, &rdy_events[i]);
	}
	for (int i = 0; i < release_count; ++i) {
		struct uart_event evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf.buf = release_bufs[i],
		};
		async_user_callback(data, &evt);
	}
	if (send_disabled) {
		struct uart_event evt = {
			.type = UART_RX_DISABLED,
		};
		async_user_callback(data, &evt);
	}

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

	ret = clock_control_on(GD32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&cfg->clkid);
	if (ret < 0) {
		return ret;
	}

	ret = reset_line_toggle_dt(&cfg->reset);
	if (ret < 0) {
		return ret;
	}

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

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	/* Initialize runtime configuration from Devicetree defaults */
	data->parity = cfg->parity;
	data->data_bits = UART_CFG_DATA_BITS_8;
	data->stop_bits = UART_CFG_STOP_BITS_1;
	data->flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	data->initialized = true;
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_ASYNC_API
	data->dev = dev;
	data->rx.buffer = NULL;
	data->rx.buffer_length = 0U;
	data->rx.offset = 0U;
	data->rx.counter = 0U;
	data->tx.buffer = NULL;
	data->tx.buffer_length = 0U;
	data->tx.counter = 0U;
	atomic_set(&data->tx_state, GD32_USART_TX_IDLE);
	data->rx_timeout = 0;
	data->rx_dma_active = false;
	data->tx_dma_active = false;
	data->rx_hw_timeout_active = false;
	if (usart_gd32_has_receiver_timeout(cfg->reg)) {
		usart_interrupt_disable(cfg->reg, USART_INT_RT);
		usart_receiver_timeout_disable(cfg->reg);
		usart_flag_clear(cfg->reg, USART_FLAG_RT);
	}
	k_mutex_init(&data->async_lock);
	k_work_init_delayable(&data->rx_timeout_work, usart_gd32_async_rx_timeout_handler);
	for (size_t i = 0U; i < ARRAY_SIZE(data->tx_timeout); i++) {
		data->tx_timeout[i].data = data;
		k_work_init_delayable(&data->tx_timeout[i].work,
				      usart_gd32_async_tx_timeout_handler);
	}
#ifdef CONFIG_DMA
	if (((data->dma_rx.dma_dev != NULL) && !device_is_ready(data->dma_rx.dma_dev)) ||
	    ((data->dma_tx.dma_dev != NULL) && !device_is_ready(data->dma_tx.dma_dev))) {
		return -ENODEV;
	}

	data->dma_rx.dma_cfg.head_block = &data->dma_rx.blk_cfg;
	data->dma_tx.dma_cfg.head_block = &data->dma_tx.blk_cfg;
	data->dma_rx.dma_cfg.complete_callback_en = 1U;
	data->dma_tx.dma_cfg.complete_callback_en = 1U;
	data->dma_rx.dma_cfg.user_data = (void *)dev;
	data->dma_tx.dma_cfg.user_data = (void *)dev;
	data->dma_rx.blk_cfg.source_address = GD32_USART_DATA_REG_ADDR(cfg->reg);
	data->dma_rx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	data->dma_rx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->dma_tx.blk_cfg.dest_address = GD32_USART_DATA_REG_ADDR(cfg->reg);
	data->dma_tx.blk_cfg.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	data->dma_tx.blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
#endif
	data->rx_next_buffer = NULL;
	data->rx_next_buffer_len = 0U;
#endif
	return 0;
}

static int usart_gd32_poll_in(const struct device *dev, unsigned char *c)
{
	const struct gd32_usart_config *const cfg = dev->config;
	uint32_t status;

#ifdef CONFIG_UART_ASYNC_API
	struct gd32_usart_data *data = dev->data;

	if (data->rx.buffer_length != 0U) {
		return -EBUSY;
	}
#endif

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
	uint32_t timeout_us = usart_gd32_calc_char_timeout_us(dev);

	if (!WAIT_FOR(usart_flag_get(cfg->reg, USART_FLAG_TBE) != RESET,
		      timeout_us, k_busy_wait(1))) {
		/* Avoid blocking logging paths forever if the peripheral is wedged. */
		return;
	}

	usart_data_transmit(cfg->reg, c);
}

static int usart_gd32_err_check(const struct device *dev)
{
	const struct gd32_usart_config *const cfg = dev->config;
	uint32_t status = USART_STAT(cfg->reg);
	int errors = 0;

	if (status & USART_STAT0_ORERR) {
		errors |= UART_ERROR_OVERRUN;
	}

	if (status & USART_STAT0_PERR) {
		errors |= UART_ERROR_PARITY;
	}

	if (status & USART_STAT0_FERR) {
		errors |= UART_ERROR_FRAMING;
	}

	if (status & USART_STAT0_NERR) {
		errors |= UART_ERROR_NOISE;
	}

	if (errors != 0) {
		volatile uint32_t dummy = USART_DATA(cfg->reg);
		ARG_UNUSED(dummy);
	}

	return errors;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int usart_gd32_configure(const struct device *dev, const struct uart_config *cfg_new)
{
	const struct gd32_usart_config *const cfg = dev->config;
	struct gd32_usart_data *const data = dev->data;
	uint32_t parity_bits;
	uint32_t word_length;
	uint32_t stop_bits_hw;

	if (cfg_new == NULL) {
		return -EINVAL;
	}

	if (cfg_new->baudrate == 0U) {
		return -EINVAL;
	}

	if (cfg_new->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
		return -ENOTSUP;
	}

	switch (cfg_new->parity) {
	case UART_CFG_PARITY_NONE:
		parity_bits = USART_PM_NONE;
		break;
	case UART_CFG_PARITY_ODD:
		parity_bits = USART_PM_ODD;
		break;
	case UART_CFG_PARITY_EVEN:
		parity_bits = USART_PM_EVEN;
		break;
	default:
		return -EINVAL;
	}

	switch (cfg_new->data_bits) {
	case UART_CFG_DATA_BITS_8:
	case UART_CFG_DATA_BITS_7:
		break;
	default:
		return -EINVAL;
	}

	if (cfg_new->data_bits == UART_CFG_DATA_BITS_7 && cfg_new->parity == UART_CFG_PARITY_NONE) {
		return -EINVAL;
	}

	/* Map word length depending on requested data bits and parity */
	if (cfg_new->parity == UART_CFG_PARITY_NONE) {
		/* 8N* uses 8-bit word length */
		word_length = USART_WL_8BIT;
	} else {
		/* With parity: 8 data bits -> 9-bit word length, 7 data bits -> 8-bit */
		word_length = (cfg_new->data_bits == UART_CFG_DATA_BITS_8) ? USART_WL_9BIT
									   : USART_WL_8BIT;
	}

	switch (cfg_new->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		stop_bits_hw = USART_STB_1BIT;
		break;
	case UART_CFG_STOP_BITS_2:
		stop_bits_hw = USART_STB_2BIT;
		break;
	default:
		return -EINVAL;
	}

	if (data->baud_rate == cfg_new->baudrate && data->parity == cfg_new->parity &&
	    data->data_bits == cfg_new->data_bits && data->stop_bits == cfg_new->stop_bits &&
	    data->flow_ctrl == cfg_new->flow_ctrl) {
		return 0;
	}

#ifdef CONFIG_UART_ASYNC_API
	k_mutex_lock(&data->async_lock, K_FOREVER);
	if ((atomic_get(&data->tx_state) != GD32_USART_TX_IDLE) ||
	    (data->rx.buffer_length != 0U)) {
		k_mutex_unlock(&data->async_lock);
		return -EBUSY;
	}
#endif

	unsigned int key = irq_lock();

	usart_disable(cfg->reg);

	usart_parity_config(cfg->reg, parity_bits);
	usart_word_length_set(cfg->reg, word_length);
	usart_stop_bit_set(cfg->reg, stop_bits_hw);
	usart_baudrate_set(cfg->reg, cfg_new->baudrate);

	usart_receive_config(cfg->reg, USART_RECEIVE_ENABLE);
	usart_transmit_config(cfg->reg, USART_TRANSMIT_ENABLE);
	usart_enable(cfg->reg);

	irq_unlock(key);

	data->baud_rate = cfg_new->baudrate;
	data->parity = cfg_new->parity;
	data->data_bits = cfg_new->data_bits;
	data->stop_bits = cfg_new->stop_bits;
	data->flow_ctrl = cfg_new->flow_ctrl;

#ifdef CONFIG_UART_ASYNC_API
	k_mutex_unlock(&data->async_lock);
#endif

	return 0;
}

static int usart_gd32_config_get(const struct device *dev, struct uart_config *cfg_out)
{
	struct gd32_usart_data *const data = dev->data;

	if (cfg_out == NULL) {
		return -EINVAL;
	}

	if (!data->initialized) {
		return -ENODEV;
	}

	cfg_out->baudrate = data->baud_rate;
	cfg_out->parity = data->parity;
	cfg_out->stop_bits = data->stop_bits;
	cfg_out->data_bits = data->data_bits;
	cfg_out->flow_ctrl = data->flow_ctrl;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

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
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = usart_gd32_configure,
	.config_get = usart_gd32_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
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
		IF_ENABLED(CONFIG_UART_ASYNC_API,				\
			(GD32_USART_DMA_INIT_FIELDS(n)))			\
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
