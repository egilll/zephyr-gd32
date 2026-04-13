/*
 * Copyright 2017, 2024 NXP
 * Copyright (c) 2020 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kinetis_uart

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <fsl_uart.h>
#include <soc.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/pinctrl.h>
#if defined(CONFIG_UART_ASYNC_API) && defined(CONFIG_DMA)
#include <zephyr/drivers/dma.h>
#endif

#if defined(CONFIG_UART_ASYNC_API) && defined(CONFIG_DMA) && \
	defined(FSL_FEATURE_UART_HAS_DMA_SELECT) && FSL_FEATURE_UART_HAS_DMA_SELECT
#define UART_MCUX_ASYNC_DMA 1
#endif

#ifdef UART_MCUX_ASYNC_DMA
struct uart_mcux_dma_config {
	const struct device *dev;
	uint32_t channel;
	struct dma_config cfg;
};
#endif

struct uart_mcux_config {
	UART_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	void (*irq_config_func)(const struct device *dev);
#endif
	const struct pinctrl_dev_config *pincfg;
#ifdef UART_MCUX_ASYNC_DMA
	const struct uart_mcux_dma_config tx_dma;
	const struct uart_mcux_dma_config rx_dma;
#endif
};

struct uart_mcux_data {
	struct uart_config uart_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
#ifdef CONFIG_UART_ASYNC_API
	struct {
		const struct device *dev;
		uart_callback_t callback;
		void *user_data;
		struct {
			const uint8_t *buf;
			size_t len;
			size_t counter;
			int32_t timeout;
			struct k_work_delayable timeout_work;
#ifdef UART_MCUX_ASYNC_DMA
			struct dma_block_config dma_block;
			bool dma_active;
			bool dma_wait_tc;
#endif
		} tx;
		struct {
			uint8_t *buf;
			size_t len;
			size_t offset;
			size_t counter;
			uint8_t *next_buf;
			size_t next_len;
			int32_t timeout;
			struct k_work_delayable timeout_work;
#ifdef UART_MCUX_ASYNC_DMA
			struct dma_block_config dma_block;
			bool dma_active;
#endif
		} rx;
	} async;
#endif
};

#ifdef CONFIG_UART_ASYNC_API
#define UART_MCUX_ASYNC_TX_IRQS \
	(kUART_TxDataRegEmptyInterruptEnable | kUART_TransmissionCompleteInterruptEnable)

#define UART_MCUX_ASYNC_RX_IRQS \
	(kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable | \
	 kUART_NoiseErrorInterruptEnable | kUART_FramingErrorInterruptEnable | \
	 kUART_ParityErrorInterruptEnable)

#define UART_MCUX_ASYNC_RX_TIMER_IRQS kUART_IdleLineInterruptEnable

static inline bool uart_mcux_async_timeout_enabled(int32_t timeout)
{
	return (timeout != SYS_FOREVER_US) && (timeout != 0);
}

static inline void uart_mcux_async_timer_start(struct k_work_delayable *work, int32_t timeout)
{
	if (uart_mcux_async_timeout_enabled(timeout)) {
		k_work_reschedule(work, K_USEC(timeout));
	}
}

static inline void uart_mcux_async_user_callback(const struct device *dev,
						 struct uart_event *event)
{
	struct uart_mcux_data *data = dev->data;

	if (data->async.callback) {
		data->async.callback(dev, event, data->async.user_data);
	}
}

static inline void uart_mcux_async_clear_idle_flag(UART_Type *base)
{
	volatile uint8_t dummy;

	while ((base->S1 & UART_S1_IDLE_MASK) != 0U) {
		dummy = base->S1;
		dummy = base->D;
	}

	ARG_UNUSED(dummy);

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
	base->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
#endif
}

#ifdef UART_MCUX_ASYNC_DMA
static inline bool uart_mcux_async_tx_uses_dma(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;

	return config->tx_dma.dev != NULL;
}

static inline bool uart_mcux_async_rx_uses_dma(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;

	return config->rx_dma.dev != NULL;
}

static void __maybe_unused uart_mcux_dma_tx_cb(const struct device *dma_dev, void *user_data,
					       uint32_t channel, int status);
static void __maybe_unused uart_mcux_dma_rx_cb(const struct device *dma_dev, void *user_data,
					       uint32_t channel, int status);

static void uart_mcux_async_rx_dma_update_counter(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	struct dma_status status;
	size_t counter;
	unsigned int key;

	if (!data->async.rx.dma_active || (data->async.rx.buf == NULL)) {
		return;
	}

	if (dma_get_status(config->rx_dma.dev, config->rx_dma.channel, &status) != 0) {
		return;
	}

	if (status.pending_length > data->async.rx.len) {
		return;
	}

	counter = data->async.rx.len - status.pending_length;
	key = irq_lock();
	if ((data->async.rx.buf != NULL) && (counter > data->async.rx.counter)) {
		data->async.rx.counter = counter;
	}
	irq_unlock(key);
}

static int uart_mcux_async_tx_dma_start(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	struct dma_config dma_cfg = config->tx_dma.cfg;
	int ret;

	data->async.tx.dma_block = (struct dma_block_config) {
		.source_address = (uint32_t)data->async.tx.buf,
		.dest_address = UART_GetDataRegisterAddress(config->base),
		.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
		.block_size = data->async.tx.len,
	};
	dma_cfg.head_block = &data->async.tx.dma_block;

	UART_EnableTxDMA(config->base, false);

	ret = dma_config(config->tx_dma.dev, config->tx_dma.channel, &dma_cfg);
	if (ret != 0) {
		return ret;
	}

	ret = dma_start(config->tx_dma.dev, config->tx_dma.channel);
	if (ret != 0) {
		return ret;
	}

	data->async.tx.dma_active = true;
	data->async.tx.dma_wait_tc = false;
	UART_EnableTxDMA(config->base, true);

	return 0;
}

static int uart_mcux_async_rx_dma_start(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	struct dma_config dma_cfg = config->rx_dma.cfg;
	int ret;

	data->async.rx.dma_block = (struct dma_block_config) {
		.source_address = UART_GetDataRegisterAddress(config->base),
		.dest_address = (uint32_t)data->async.rx.buf,
		.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
		.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT,
		.block_size = data->async.rx.len,
	};
	dma_cfg.head_block = &data->async.rx.dma_block;

	UART_EnableRxDMA(config->base, false);

	ret = dma_config(config->rx_dma.dev, config->rx_dma.channel, &dma_cfg);
	if (ret != 0) {
		return ret;
	}

	ret = dma_start(config->rx_dma.dev, config->rx_dma.channel);
	if (ret != 0) {
		return ret;
	}

	data->async.rx.dma_active = true;
	UART_EnableRxDMA(config->base, true);

	return 0;
}
#else
static inline bool uart_mcux_async_tx_uses_dma(const struct device *dev)
{
	ARG_UNUSED(dev);

	return false;
}

static inline bool uart_mcux_async_rx_uses_dma(const struct device *dev)
{
	ARG_UNUSED(dev);

	return false;
}
#endif

static size_t uart_mcux_async_tx_count(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	size_t count = data->async.tx.counter;

#ifdef UART_MCUX_ASYNC_DMA
	if (uart_mcux_async_tx_uses_dma(dev)) {
		if (data->async.tx.dma_wait_tc) {
			return data->async.tx.len;
		}

		if (data->async.tx.dma_active) {
			struct dma_status status;

			if ((dma_get_status(config->tx_dma.dev, config->tx_dma.channel, &status) == 0) &&
			    (status.pending_length <= data->async.tx.len)) {
				return data->async.tx.len - status.pending_length;
			}
		}
	}
#endif

#if defined(FSL_FEATURE_UART_HAS_FIFO) && FSL_FEATURE_UART_HAS_FIFO
	count -= MIN(count, (size_t)UART_GetTxFifoCount(config->base));
#else
	if ((count > 0U) &&
	    ((UART_GetStatusFlags(config->base) & kUART_TxDataRegEmptyFlag) == 0U)) {
		count--;
	}
#endif

	return count;
}

static void uart_mcux_async_update_busy(const struct device *dev)
{
	struct uart_mcux_data *data = dev->data;

	if ((data->async.tx.buf != NULL) || (data->async.rx.buf != NULL)) {
		pm_device_busy_set(dev);
	} else {
		pm_device_busy_clear(dev);
	}
}

static void uart_mcux_async_rx_disable_locked(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;

	UART_DisableInterrupts(config->base, UART_MCUX_ASYNC_RX_IRQS | UART_MCUX_ASYNC_RX_TIMER_IRQS);
#ifdef UART_MCUX_ASYNC_DMA
	UART_EnableRxDMA(config->base, false);
	data->async.rx.dma_active = false;
#endif
	(void)k_work_cancel_delayable(&data->async.rx.timeout_work);
	uart_mcux_async_update_busy(dev);
}

static void uart_mcux_async_tx_disable_locked(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;

	UART_DisableInterrupts(config->base, UART_MCUX_ASYNC_TX_IRQS);
#ifdef UART_MCUX_ASYNC_DMA
	UART_EnableTxDMA(config->base, false);
	data->async.tx.dma_active = false;
	data->async.tx.dma_wait_tc = false;
#endif
	(void)k_work_cancel_delayable(&data->async.tx.timeout_work);
	uart_mcux_async_update_busy(dev);
}

static void uart_mcux_async_rx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_mcux_data *data =
		CONTAINER_OF(dwork, struct uart_mcux_data, async.rx.timeout_work);
	struct uart_event event;
	const struct device *dev = data->async.dev;
	unsigned int key = irq_lock();
	size_t len;
	size_t offset;
	uint8_t *buf;

#ifdef UART_MCUX_ASYNC_DMA
	if (uart_mcux_async_rx_uses_dma(dev)) {
		uart_mcux_async_rx_dma_update_counter(dev);
	}
#endif

	if (data->async.rx.buf == NULL) {
		irq_unlock(key);
		return;
	}

	if (data->async.rx.counter <= data->async.rx.offset) {
		irq_unlock(key);
		return;
	}

	buf = data->async.rx.buf;
	offset = data->async.rx.offset;
	len = data->async.rx.counter - data->async.rx.offset;
	data->async.rx.offset = data->async.rx.counter;
	irq_unlock(key);

	event = (struct uart_event) {
		.type = UART_RX_RDY,
		.data.rx.buf = buf,
		.data.rx.offset = offset,
		.data.rx.len = len,
	};

	uart_mcux_async_user_callback(dev, &event);
}

static void uart_mcux_async_tx_timeout(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct uart_mcux_data *data =
		CONTAINER_OF(dwork, struct uart_mcux_data, async.tx.timeout_work);
	const struct device *dev = data->async.dev;

	(void)uart_tx_abort(dev);
}

static int uart_mcux_callback_set(const struct device *dev, uart_callback_t callback,
				  void *user_data)
{
	struct uart_mcux_data *data = dev->data;

	data->async.callback = callback;
	data->async.user_data = user_data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS) && defined(CONFIG_UART_INTERRUPT_DRIVEN)
	data->callback = NULL;
	data->cb_data = NULL;
#endif

	return 0;
}

static int uart_mcux_tx(const struct device *dev, const uint8_t *buf, size_t len,
			int32_t timeout)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	unsigned int key = irq_lock();

	if ((buf == NULL) || (len == 0U)) {
		irq_unlock(key);
		return -EINVAL;
	}

	if (data->async.tx.buf != NULL) {
		irq_unlock(key);
		return -EBUSY;
	}

	data->async.tx.buf = buf;
	data->async.tx.len = len;
	data->async.tx.counter = 0U;
	data->async.tx.timeout = timeout;
#ifdef UART_MCUX_ASYNC_DMA
	data->async.tx.dma_active = false;
	data->async.tx.dma_wait_tc = false;
#endif

	uart_mcux_async_update_busy(dev);
	UART_DisableInterrupts(config->base, UART_MCUX_ASYNC_TX_IRQS);

#ifdef UART_MCUX_ASYNC_DMA
	if (uart_mcux_async_tx_uses_dma(dev)) {
		int ret;

		ret = uart_mcux_async_tx_dma_start(dev);
		if (ret != 0) {
			data->async.tx.buf = NULL;
			data->async.tx.len = 0U;
			uart_mcux_async_update_busy(dev);
			irq_unlock(key);
			return ret;
		}
	} else
#endif
	{
		UART_EnableInterrupts(config->base, kUART_TxDataRegEmptyInterruptEnable);
	}

	uart_mcux_async_timer_start(&data->async.tx.timeout_work, timeout);
	irq_unlock(key);

	return 0;
}

static int uart_mcux_tx_abort(const struct device *dev)
{
	struct uart_mcux_data *data = dev->data;
	struct uart_event event;
	const uint8_t *buf;
	size_t len;
	unsigned int key = irq_lock();

#ifdef UART_MCUX_ASYNC_DMA
	const struct uart_mcux_config *config = dev->config;
#endif

	if (data->async.tx.buf == NULL) {
		irq_unlock(key);
		return -EFAULT;
	}

#ifdef UART_MCUX_ASYNC_DMA
	if (data->async.tx.dma_active && uart_mcux_async_tx_uses_dma(dev)) {
		UART_EnableTxDMA(config->base, false);
		(void)dma_stop(config->tx_dma.dev, config->tx_dma.channel);
	}
#endif

	buf = data->async.tx.buf;
	len = uart_mcux_async_tx_count(dev);
	data->async.tx.buf = NULL;
	data->async.tx.len = 0U;
	data->async.tx.counter = 0U;
	uart_mcux_async_tx_disable_locked(dev);
	irq_unlock(key);

	event = (struct uart_event) {
		.type = UART_TX_ABORTED,
		.data.tx.buf = buf,
		.data.tx.len = len,
	};

	uart_mcux_async_user_callback(dev, &event);

	return 0;
}

static int uart_mcux_rx_enable(const struct device *dev, uint8_t *buf, size_t len,
			       int32_t timeout)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	struct uart_event event = {
		.type = UART_RX_BUF_REQUEST,
	};
	unsigned int key = irq_lock();

	if ((buf == NULL) || (len == 0U)) {
		irq_unlock(key);
		return -EINVAL;
	}

	if (data->async.rx.buf != NULL) {
		irq_unlock(key);
		return -EBUSY;
	}

	data->async.rx.buf = buf;
	data->async.rx.len = len;
	data->async.rx.offset = 0U;
	data->async.rx.counter = 0U;
	data->async.rx.next_buf = NULL;
	data->async.rx.next_len = 0U;
	data->async.rx.timeout = timeout;
#ifdef UART_MCUX_ASYNC_DMA
	data->async.rx.dma_active = false;
#endif

	(void)k_work_cancel_delayable(&data->async.rx.timeout_work);
	UART_ClearStatusFlags(config->base, kUART_RxOverrunFlag |
					    kUART_ParityErrorFlag |
					    kUART_FramingErrorFlag |
					    kUART_NoiseErrorFlag);
	uart_mcux_async_clear_idle_flag(config->base);
	uart_mcux_async_update_busy(dev);

#ifdef UART_MCUX_ASYNC_DMA
	if (uart_mcux_async_rx_uses_dma(dev)) {
		int ret = uart_mcux_async_rx_dma_start(dev);

		if (ret != 0) {
			data->async.rx.buf = NULL;
			data->async.rx.len = 0U;
			uart_mcux_async_update_busy(dev);
			irq_unlock(key);
			return ret;
		}

		UART_EnableInterrupts(config->base, kUART_RxOverrunInterruptEnable |
						      kUART_NoiseErrorInterruptEnable |
						      kUART_FramingErrorInterruptEnable |
						      kUART_ParityErrorInterruptEnable);
		if (uart_mcux_async_timeout_enabled(timeout)) {
			UART_EnableInterrupts(config->base, kUART_IdleLineInterruptEnable);
		}
	} else
#endif
	{
		UART_EnableInterrupts(config->base, UART_MCUX_ASYNC_RX_IRQS);
	}
	irq_unlock(key);

	uart_mcux_async_user_callback(dev, &event);

	return 0;
}

static int uart_mcux_rx_buf_rsp(const struct device *dev, uint8_t *buf, size_t len)
{
	struct uart_mcux_data *data = dev->data;
	unsigned int key = irq_lock();

	if ((buf == NULL) || (len == 0U)) {
		irq_unlock(key);
		return -EINVAL;
	}

	if (data->async.rx.buf == NULL) {
		irq_unlock(key);
		return -EACCES;
	}

	if (data->async.rx.next_buf != NULL) {
		irq_unlock(key);
		return -EBUSY;
	}

	data->async.rx.next_buf = buf;
	data->async.rx.next_len = len;
	irq_unlock(key);

	return 0;
}

static int uart_mcux_rx_disable(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	struct uart_event event;
	uint8_t *buf;
	uint8_t *next_buf;
	size_t len;
	size_t offset;
	size_t next_len;
	unsigned int key = irq_lock();

	if (data->async.rx.buf == NULL) {
		irq_unlock(key);
		return -EFAULT;
	}

#ifdef UART_MCUX_ASYNC_DMA
	if (data->async.rx.dma_active && uart_mcux_async_rx_uses_dma(dev)) {
		UART_EnableRxDMA(config->base, false);
		uart_mcux_async_rx_dma_update_counter(dev);
		(void)dma_stop(config->rx_dma.dev, config->rx_dma.channel);
	}
#endif

	buf = data->async.rx.buf;
	len = data->async.rx.counter - data->async.rx.offset;
	offset = data->async.rx.offset;
	next_buf = data->async.rx.next_buf;
	next_len = data->async.rx.next_len;
	data->async.rx.buf = NULL;
	data->async.rx.len = 0U;
	data->async.rx.offset = 0U;
	data->async.rx.counter = 0U;
	data->async.rx.next_buf = NULL;
	data->async.rx.next_len = 0U;
	uart_mcux_async_rx_disable_locked(dev);
	UART_ClearStatusFlags(config->base, kUART_RxOverrunFlag |
					    kUART_ParityErrorFlag |
					    kUART_FramingErrorFlag |
					    kUART_NoiseErrorFlag);
	uart_mcux_async_clear_idle_flag(config->base);
	irq_unlock(key);

	if (len > 0U) {
		event = (struct uart_event) {
			.type = UART_RX_RDY,
			.data.rx.buf = buf,
			.data.rx.offset = offset,
			.data.rx.len = len,
		};
		uart_mcux_async_user_callback(dev, &event);
	}

	event = (struct uart_event) {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf.buf = buf,
	};
	uart_mcux_async_user_callback(dev, &event);

	if ((next_buf != NULL) && (next_len > 0U)) {
		event = (struct uart_event) {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf.buf = next_buf,
		};
		uart_mcux_async_user_callback(dev, &event);
	}

	event = (struct uart_event) {
		.type = UART_RX_DISABLED,
	};
	uart_mcux_async_user_callback(dev, &event);

	return 0;
}

static size_t uart_mcux_async_rx_stop_locked(const struct device *dev,
					     enum uart_rx_stop_reason reason,
					     struct uart_event *events)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	uint8_t *buf = data->async.rx.buf;
	uint8_t *next_buf = data->async.rx.next_buf;
	size_t offset = data->async.rx.offset;
	size_t len = data->async.rx.counter - data->async.rx.offset;
	size_t event_count = 0U;

#ifdef UART_MCUX_ASYNC_DMA
	if (data->async.rx.dma_active && uart_mcux_async_rx_uses_dma(dev)) {
		UART_EnableRxDMA(config->base, false);
		(void)dma_stop(config->rx_dma.dev, config->rx_dma.channel);
	}
#endif

	data->async.rx.buf = NULL;
	data->async.rx.len = 0U;
	data->async.rx.offset = 0U;
	data->async.rx.counter = 0U;
	data->async.rx.next_buf = NULL;
	data->async.rx.next_len = 0U;
	uart_mcux_async_rx_disable_locked(dev);
	UART_ClearStatusFlags(config->base, kUART_RxOverrunFlag |
					    kUART_ParityErrorFlag |
					    kUART_FramingErrorFlag |
					    kUART_NoiseErrorFlag);
	uart_mcux_async_clear_idle_flag(config->base);

	events[event_count++] = (struct uart_event) {
		.type = UART_RX_STOPPED,
		.data.rx_stop.reason = reason,
		.data.rx_stop.data.buf = buf,
		.data.rx_stop.data.offset = offset,
		.data.rx_stop.data.len = len,
	};

	if (len > 0U) {
		events[event_count++] = (struct uart_event) {
			.type = UART_RX_RDY,
			.data.rx.buf = buf,
			.data.rx.offset = offset,
			.data.rx.len = len,
		};
	}

	if (buf != NULL) {
		events[event_count++] = (struct uart_event) {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf.buf = buf,
		};
	}

	if (next_buf != NULL) {
		events[event_count++] = (struct uart_event) {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf.buf = next_buf,
		};
	}

	events[event_count++] = (struct uart_event) {
		.type = UART_RX_DISABLED,
	};

	return event_count;
}

static size_t uart_mcux_async_tx_handle(const struct device *dev, uint32_t flags, uint32_t enabled,
					struct uart_event *events)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;

#ifdef UART_MCUX_ASYNC_DMA
	if (data->async.tx.dma_active) {
		return 0U;
	}
#endif

	if ((data->async.tx.buf == NULL) || (data->async.tx.counter >= data->async.tx.len)) {
		goto check_done;
	}

	if ((enabled & kUART_TxDataRegEmptyInterruptEnable) != 0U) {
		while ((data->async.tx.counter < data->async.tx.len) &&
		       ((UART_GetStatusFlags(config->base) & kUART_TxDataRegEmptyFlag) != 0U)) {
			UART_WriteByte(config->base, data->async.tx.buf[data->async.tx.counter++]);
		}

		if (data->async.tx.counter == data->async.tx.len) {
			UART_DisableInterrupts(config->base, kUART_TxDataRegEmptyInterruptEnable);
			UART_EnableInterrupts(config->base, kUART_TransmissionCompleteInterruptEnable);
		}
	}

check_done:
	if ((data->async.tx.buf == NULL) ||
	    ((enabled & kUART_TransmissionCompleteInterruptEnable) == 0U) ||
	    ((flags & kUART_TransmissionCompleteFlag) == 0U)) {
		return 0U;
	}

	events[0] = (struct uart_event) {
		.type = UART_TX_DONE,
		.data.tx.buf = data->async.tx.buf,
		.data.tx.len = data->async.tx.len,
	};

	data->async.tx.buf = NULL;
	data->async.tx.len = 0U;
	data->async.tx.counter = 0U;
	uart_mcux_async_tx_disable_locked(dev);

	return 1U;
}

static size_t uart_mcux_async_rx_handle(const struct device *dev, uint32_t flags, uint32_t enabled,
					struct uart_event *events)
{
	struct uart_mcux_data *data = dev->data;
	size_t event_count = 0U;
	enum uart_rx_stop_reason reason = 0U;

	if ((flags & kUART_RxOverrunFlag) != 0U) {
		reason |= UART_ERROR_OVERRUN;
	}
	if ((flags & kUART_ParityErrorFlag) != 0U) {
		reason |= UART_ERROR_PARITY;
	}
	if ((flags & kUART_FramingErrorFlag) != 0U) {
		reason |= UART_ERROR_FRAMING;
	}
	if ((flags & kUART_NoiseErrorFlag) != 0U) {
		reason |= UART_ERROR_NOISE;
	}

	if (reason != 0U) {
#ifdef UART_MCUX_ASYNC_DMA
		if (data->async.rx.dma_active) {
			uart_mcux_async_rx_dma_update_counter(dev);
		}
#endif
		return uart_mcux_async_rx_stop_locked(dev, reason, events);
	}

#ifdef UART_MCUX_ASYNC_DMA
	if (data->async.rx.dma_active) {
		if (((enabled & kUART_IdleLineInterruptEnable) != 0U) &&
		    ((flags & kUART_IdleLineFlag) != 0U)) {
			uart_mcux_async_clear_idle_flag(((const struct uart_mcux_config *)
							 dev->config)->base);
			uart_mcux_async_timer_start(&data->async.rx.timeout_work,
						    data->async.rx.timeout);
		}

		return 0U;
	}
#endif

	while ((data->async.rx.buf != NULL) &&
	       ((UART_GetStatusFlags(((const struct uart_mcux_config *)dev->config)->base) &
		 kUART_RxDataRegFullFlag) != 0U)) {
		if (data->async.rx.counter < data->async.rx.len) {
			data->async.rx.buf[data->async.rx.counter++] =
				UART_ReadByte(((const struct uart_mcux_config *)dev->config)->base);
			uart_mcux_async_timer_start(&data->async.rx.timeout_work,
						    data->async.rx.timeout);
		} else {
			(void)UART_ReadByte(((const struct uart_mcux_config *)dev->config)->base);
		}

		if (data->async.rx.counter == data->async.rx.len) {
			uint8_t *full_buf = data->async.rx.buf;
			uint8_t *next_buf = data->async.rx.next_buf;
			size_t next_len = data->async.rx.next_len;
			size_t offset = data->async.rx.offset;
			size_t len = data->async.rx.counter - data->async.rx.offset;

			events[event_count++] = (struct uart_event) {
				.type = UART_RX_RDY,
				.data.rx.buf = full_buf,
				.data.rx.offset = offset,
				.data.rx.len = len,
			};
			events[event_count++] = (struct uart_event) {
				.type = UART_RX_BUF_RELEASED,
				.data.rx_buf.buf = full_buf,
			};

			data->async.rx.buf = next_buf;
			data->async.rx.len = next_len;
			data->async.rx.offset = 0U;
			data->async.rx.counter = 0U;
			data->async.rx.next_buf = NULL;
			data->async.rx.next_len = 0U;
			(void)k_work_cancel_delayable(&data->async.rx.timeout_work);

			if (next_buf != NULL) {
				events[event_count++] = (struct uart_event) {
					.type = UART_RX_BUF_REQUEST,
				};
			} else {
				uart_mcux_async_rx_disable_locked(dev);
				events[event_count++] = (struct uart_event) {
					.type = UART_RX_DISABLED,
				};
			}

			break;
		}
	}

	if (event_count == 0U) {
		return 0U;
	}

	return event_count;
}

#ifdef UART_MCUX_ASYNC_DMA
static void __maybe_unused uart_mcux_dma_tx_cb(const struct device *dma_dev, void *user_data,
					       uint32_t channel, int status)
{
	const struct device *dev = user_data;
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	struct uart_event event;
	unsigned int key;
	size_t len;

	ARG_UNUSED(dma_dev);

	if (channel != config->tx_dma.channel) {
		return;
	}

	key = irq_lock();
	if (!data->async.tx.dma_active || (data->async.tx.buf == NULL)) {
		irq_unlock(key);
		return;
	}

	if (status == 0) {
		data->async.tx.counter = data->async.tx.len;
		data->async.tx.dma_active = false;
		data->async.tx.dma_wait_tc = true;
		UART_EnableTxDMA(config->base, false);
		UART_EnableInterrupts(config->base, kUART_TransmissionCompleteInterruptEnable);
		irq_unlock(key);
		return;
	}

	len = uart_mcux_async_tx_count(dev);
	event = (struct uart_event) {
		.type = UART_TX_ABORTED,
		.data.tx.buf = data->async.tx.buf,
		.data.tx.len = len,
	};

	data->async.tx.buf = NULL;
	data->async.tx.len = 0U;
	data->async.tx.counter = 0U;
	uart_mcux_async_tx_disable_locked(dev);
	irq_unlock(key);

	uart_mcux_async_user_callback(dev, &event);
}

static void __maybe_unused uart_mcux_dma_rx_cb(const struct device *dma_dev, void *user_data,
					       uint32_t channel, int status)
{
	const struct device *dev = user_data;
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	struct uart_event events[5];
	size_t event_count = 0U;
	unsigned int key;

	ARG_UNUSED(dma_dev);

	if (channel != config->rx_dma.channel) {
		return;
	}

	key = irq_lock();
	if (!data->async.rx.dma_active || (data->async.rx.buf == NULL)) {
		irq_unlock(key);
		return;
	}

	data->async.rx.counter = data->async.rx.len;
	data->async.rx.dma_active = false;

	if (status != 0) {
		event_count = uart_mcux_async_rx_stop_locked(dev, UART_ERROR_OVERRUN, events);
		irq_unlock(key);

		for (size_t i = 0; i < event_count; i++) {
			uart_mcux_async_user_callback(dev, &events[i]);
		}

		return;
	}

	events[event_count++] = (struct uart_event) {
		.type = UART_RX_RDY,
		.data.rx.buf = data->async.rx.buf,
		.data.rx.offset = data->async.rx.offset,
		.data.rx.len = data->async.rx.counter - data->async.rx.offset,
	};
	events[event_count++] = (struct uart_event) {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf.buf = data->async.rx.buf,
	};

	(void)k_work_cancel_delayable(&data->async.rx.timeout_work);

	if (data->async.rx.next_buf != NULL) {
		data->async.rx.buf = data->async.rx.next_buf;
		data->async.rx.len = data->async.rx.next_len;
		data->async.rx.offset = 0U;
		data->async.rx.counter = 0U;
		data->async.rx.next_buf = NULL;
		data->async.rx.next_len = 0U;

		if (uart_mcux_async_rx_dma_start(dev) == 0) {
			if (uart_mcux_async_timeout_enabled(data->async.rx.timeout)) {
				UART_EnableInterrupts(config->base, kUART_IdleLineInterruptEnable);
			}
			events[event_count++] = (struct uart_event) {
				.type = UART_RX_BUF_REQUEST,
			};
		} else {
			uint8_t *buf = data->async.rx.buf;

			data->async.rx.buf = NULL;
			data->async.rx.len = 0U;
			data->async.rx.offset = 0U;
			data->async.rx.counter = 0U;
			uart_mcux_async_rx_disable_locked(dev);
			events[event_count++] = (struct uart_event) {
				.type = UART_RX_BUF_RELEASED,
				.data.rx_buf.buf = buf,
			};
			events[event_count++] = (struct uart_event) {
				.type = UART_RX_DISABLED,
			};
		}
	} else {
		data->async.rx.buf = NULL;
		data->async.rx.len = 0U;
		data->async.rx.offset = 0U;
		data->async.rx.counter = 0U;
		uart_mcux_async_rx_disable_locked(dev);
		events[event_count++] = (struct uart_event) {
			.type = UART_RX_DISABLED,
		};
	}

	irq_unlock(key);

	for (size_t i = 0; i < event_count; i++) {
		uart_mcux_async_user_callback(dev, &events[i]);
	}
}
#endif
#endif /* CONFIG_UART_ASYNC_API */

static int uart_mcux_configure(const struct device *dev,
			       const struct uart_config *cfg)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	uart_config_t uart_config;
	uint32_t clock_freq;
	status_t retval;

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	if (clock_control_get_rate(config->clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}

	UART_GetDefaultConfig(&uart_config);

	uart_config.enableTx = true;
	uart_config.enableRx = true;
	uart_config.baudRate_Bps = cfg->baudrate;

	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
#if defined(FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT) && \
FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT
		uart_config.stopBitCount = kUART_OneStopBit;
		break;
	case UART_CFG_STOP_BITS_2:
		uart_config.stopBitCount = kUART_TwoStopBit;
#endif
		break;
	default:
		return -ENOTSUP;
	}

#if defined(FSL_FEATURE_UART_HAS_MODEM_SUPPORT) && FSL_FEATURE_UART_HAS_MODEM_SUPPORT
	switch (cfg->flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE:
		uart_config.enableRxRTS = false;
		uart_config.enableTxCTS = false;
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		uart_config.enableRxRTS = true;
		uart_config.enableTxCTS = true;
		break;
	default:
		return -ENOTSUP;
	}
#endif

	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		uart_config.parityMode = kUART_ParityDisabled;
		break;
	case UART_CFG_PARITY_EVEN:
		uart_config.parityMode = kUART_ParityEven;
		break;
	case UART_CFG_PARITY_ODD:
		uart_config.parityMode = kUART_ParityOdd;
		break;
	default:
		return -ENOTSUP;
	}

	retval = UART_Init(config->base, &uart_config, clock_freq);
	if (retval != kStatus_Success) {
		return -EINVAL;
	}

	data->uart_cfg = *cfg;

	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_mcux_config_get(const struct device *dev,
				struct uart_config *cfg)
{
	struct uart_mcux_data *data = dev->data;

	*cfg = data->uart_cfg;

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static int uart_mcux_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t flags = UART_GetStatusFlags(config->base);
	int ret = -1;

	if (flags & kUART_RxDataRegFullFlag) {
		*c = UART_ReadByte(config->base);
		ret = 0;
	}

	return ret;
}

static void uart_mcux_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_mcux_config *config = dev->config;

	while (!(UART_GetStatusFlags(config->base) & kUART_TxDataRegEmptyFlag)) {
	}

	UART_WriteByte(config->base, c);
}

static int uart_mcux_err_check(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t flags = UART_GetStatusFlags(config->base);
	int err = 0;

	if (flags & kUART_RxOverrunFlag) {
		err |= UART_ERROR_OVERRUN;
	}

	if (flags & kUART_ParityErrorFlag) {
		err |= UART_ERROR_PARITY;
	}

	if (flags & kUART_FramingErrorFlag) {
		err |= UART_ERROR_FRAMING;
	}

	if (flags & kUART_NoiseErrorFlag) {
		err |= UART_ERROR_NOISE;
	}

	UART_ClearStatusFlags(config->base, kUART_RxOverrunFlag |
					    kUART_ParityErrorFlag |
					    kUART_FramingErrorFlag |
					    kUART_NoiseErrorFlag);

	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_mcux_fifo_fill(const struct device *dev,
			       const uint8_t *tx_data,
			       int len)
{
	const struct uart_mcux_config *config = dev->config;
	int num_tx = 0U;

	while ((len - num_tx > 0) &&
	       (UART_GetStatusFlags(config->base) & kUART_TxDataRegEmptyFlag)) {

		UART_WriteByte(config->base, tx_data[num_tx++]);
	}

	return num_tx;
}

static int uart_mcux_fifo_read(const struct device *dev, uint8_t *rx_data,
			       const int len)
{
	const struct uart_mcux_config *config = dev->config;
	int num_rx = 0U;

	while ((len - num_rx > 0) &&
	       (UART_GetStatusFlags(config->base) & kUART_RxDataRegFullFlag)) {

		rx_data[num_rx++] = UART_ReadByte(config->base);
	}

	return num_rx;
}

static void uart_mcux_irq_tx_enable(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t mask = kUART_TxDataRegEmptyInterruptEnable;
	pm_device_busy_set(dev);
	UART_EnableInterrupts(config->base, mask);
}

static void uart_mcux_irq_tx_disable(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t mask = kUART_TxDataRegEmptyInterruptEnable;
	pm_device_busy_clear(dev);
	UART_DisableInterrupts(config->base, mask);
}

static int uart_mcux_irq_tx_complete(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t flags = UART_GetStatusFlags(config->base);

	return (flags & kUART_TransmissionCompleteFlag) != 0U;
}

static int uart_mcux_irq_tx_ready(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t mask = kUART_TxDataRegEmptyInterruptEnable;
	uint32_t flags = UART_GetStatusFlags(config->base);

	return (UART_GetEnabledInterrupts(config->base) & mask)
		&& (flags & kUART_TxDataRegEmptyFlag);
}

static void uart_mcux_irq_rx_enable(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t mask = kUART_RxDataRegFullInterruptEnable;

	UART_EnableInterrupts(config->base, mask);
}

static void uart_mcux_irq_rx_disable(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t mask = kUART_RxDataRegFullInterruptEnable;

	UART_DisableInterrupts(config->base, mask);
}

static int uart_mcux_irq_rx_full(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t flags = UART_GetStatusFlags(config->base);

	return (flags & kUART_RxDataRegFullFlag) != 0U;
}

static int uart_mcux_irq_rx_pending(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t mask = kUART_RxDataRegFullInterruptEnable;

	return (UART_GetEnabledInterrupts(config->base) & mask)
		&& uart_mcux_irq_rx_full(dev);
}

static void uart_mcux_irq_err_enable(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t mask = kUART_NoiseErrorInterruptEnable |
			kUART_FramingErrorInterruptEnable |
			kUART_ParityErrorInterruptEnable;

	UART_EnableInterrupts(config->base, mask);
}

static void uart_mcux_irq_err_disable(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	uint32_t mask = kUART_NoiseErrorInterruptEnable |
			kUART_FramingErrorInterruptEnable |
			kUART_ParityErrorInterruptEnable;

	UART_DisableInterrupts(config->base, mask);
}

static int uart_mcux_irq_is_pending(const struct device *dev)
{
	return uart_mcux_irq_tx_ready(dev) || uart_mcux_irq_rx_pending(dev);
}

static int uart_mcux_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_mcux_irq_callback_set(const struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_mcux_data *data = dev->data;

	data->callback = cb;
	data->cb_data = cb_data;

#if defined(CONFIG_UART_EXCLUSIVE_API_CALLBACKS) && defined(CONFIG_UART_ASYNC_API)
	data->async.callback = NULL;
	data->async.user_data = NULL;
#endif
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
static void uart_mcux_isr(const struct device *dev)
{
	struct uart_mcux_data *data = dev->data;
	const struct uart_mcux_config *config = dev->config;
#ifdef CONFIG_UART_ASYNC_API
	struct uart_event events[6];
	size_t event_count = 0U;
	unsigned int key = irq_lock();
	uint32_t enabled = UART_GetEnabledInterrupts(config->base);
	uint32_t flags = UART_GetStatusFlags(config->base);

	if ((data->async.tx.buf != NULL) || (data->async.rx.buf != NULL)) {
		if (data->async.rx.buf != NULL) {
			event_count += uart_mcux_async_rx_handle(dev, flags, enabled, events);
		}

		if (data->async.tx.buf != NULL) {
			flags = UART_GetStatusFlags(config->base);
			enabled = UART_GetEnabledInterrupts(config->base);
			event_count += uart_mcux_async_tx_handle(dev, flags, enabled,
								 &events[event_count]);
		}

		irq_unlock(key);

		for (size_t i = 0; i < event_count; i++) {
			uart_mcux_async_user_callback(dev, &events[i]);
		}

		return;
	}

	irq_unlock(key);
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (data->callback) {
		data->callback(dev, data->cb_data);
	}
#endif
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */

static int uart_mcux_init(const struct device *dev)
{
	const struct uart_mcux_config *config = dev->config;
	struct uart_mcux_data *data = dev->data;
	int err;

	err = uart_mcux_configure(dev, &data->uart_cfg);
	if (err != 0) {
		return err;
	}

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

#ifdef UART_MCUX_ASYNC_DMA
	if (uart_mcux_async_tx_uses_dma(dev) && !device_is_ready(config->tx_dma.dev)) {
		return -ENODEV;
	}

	if (uart_mcux_async_rx_uses_dma(dev) && !device_is_ready(config->rx_dma.dev)) {
		return -ENODEV;
	}
#endif

#ifdef CONFIG_UART_ASYNC_API
	data->async.dev = dev;
	k_work_init_delayable(&data->async.tx.timeout_work, uart_mcux_async_tx_timeout);
	k_work_init_delayable(&data->async.rx.timeout_work, uart_mcux_async_rx_timeout);
#endif

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	config->irq_config_func(dev);
#endif

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int uart_mcux_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct uart_mcux_config *config = dev->config;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		clock_control_on(config->clock_dev, config->clock_subsys);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		clock_control_off(config->clock_dev, config->clock_subsys);
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		return 0;
	case PM_DEVICE_ACTION_TURN_ON:
		return 0;
	default:
		return -ENOTSUP;
	}
	return 0;
}
#endif /*CONFIG_PM_DEVICE*/

static DEVICE_API(uart, uart_mcux_driver_api) = {
	.poll_in = uart_mcux_poll_in,
	.poll_out = uart_mcux_poll_out,
	.err_check = uart_mcux_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_mcux_configure,
	.config_get = uart_mcux_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_mcux_fifo_fill,
	.fifo_read = uart_mcux_fifo_read,
	.irq_tx_enable = uart_mcux_irq_tx_enable,
	.irq_tx_disable = uart_mcux_irq_tx_disable,
	.irq_tx_complete = uart_mcux_irq_tx_complete,
	.irq_tx_ready = uart_mcux_irq_tx_ready,
	.irq_rx_enable = uart_mcux_irq_rx_enable,
	.irq_rx_disable = uart_mcux_irq_rx_disable,
	.irq_rx_ready = uart_mcux_irq_rx_full,
	.irq_err_enable = uart_mcux_irq_err_enable,
	.irq_err_disable = uart_mcux_irq_err_disable,
	.irq_is_pending = uart_mcux_irq_is_pending,
	.irq_update = uart_mcux_irq_update,
	.irq_callback_set = uart_mcux_irq_callback_set,
#endif
#ifdef CONFIG_UART_ASYNC_API
	.callback_set = uart_mcux_callback_set,
	.tx = uart_mcux_tx,
	.tx_abort = uart_mcux_tx_abort,
	.rx_enable = uart_mcux_rx_enable,
	.rx_buf_rsp = uart_mcux_rx_buf_rsp,
	.rx_disable = uart_mcux_rx_disable,
#endif
};

#ifdef UART_MCUX_ASYNC_DMA
#define UART_MCUX_DMA_CHANNEL_INIT(n, dir, ch_dir, cb)				\
	.dir##_dma = COND_CODE_1(DT_INST_DMAS_HAS_NAME(n, dir),		\
		((struct uart_mcux_dma_config) {				\
			.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, dir)),\
			.channel = DT_INST_DMAS_CELL_BY_NAME(n, dir, channel),	\
			.cfg = {						\
				.dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, dir, mux),\
				.channel_direction = ch_dir,			\
				.source_data_size = 1,				\
				.dest_data_size = 1,				\
				.source_burst_length = 1,			\
				.dest_burst_length = 1,				\
				.block_count = 1,				\
				.user_data = (void *)DEVICE_DT_INST_GET(n),	\
				.dma_callback = cb,				\
			},							\
		}), ((struct uart_mcux_dma_config) { 0 })),

#define UART_MCUX_DMA_CFG_INIT(n)						\
	UART_MCUX_DMA_CHANNEL_INIT(n, tx, MEMORY_TO_PERIPHERAL, uart_mcux_dma_tx_cb) \
	UART_MCUX_DMA_CHANNEL_INIT(n, rx, PERIPHERAL_TO_MEMORY, uart_mcux_dma_rx_cb)
#else
#define UART_MCUX_DMA_CFG_INIT(n)
#endif

#define UART_MCUX_DECLARE_CFG(n, IRQ_FUNC_INIT)				\
static const struct uart_mcux_config uart_mcux_##n##_config = {		\
	.base = (UART_Type *)DT_INST_REG_ADDR(n),			\
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		\
	.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),\
	.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	UART_MCUX_DMA_CFG_INIT(n)					\
	IRQ_FUNC_INIT							\
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
#define UART_MCUX_CONFIG_FUNC(n)					\
	static void uart_mcux_config_func_##n(const struct device *dev)	\
	{								\
		UART_MCUX_IRQ(n, status);	\
		UART_MCUX_IRQ(n, error);	\
	}

#define UART_MCUX_IRQ_INIT(n, name)					\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, name, irq),	\
			    DT_INST_IRQ_BY_NAME(n, name, priority),	\
			    uart_mcux_isr, DEVICE_DT_INST_GET(n), 0);	\
									\
		irq_enable(DT_INST_IRQ_BY_NAME(n, name, irq));	\
	} while (false)

#define UART_MCUX_IRQ(n, name)						\
	COND_CODE_1(DT_INST_IRQ_HAS_NAME(n, name),		\
		    (UART_MCUX_IRQ_INIT(n, name)), ())

#define UART_MCUX_IRQ_CFG_FUNC_INIT(n)					\
	.irq_config_func = uart_mcux_config_func_##n
#define UART_MCUX_INIT_CFG(n)						\
	UART_MCUX_DECLARE_CFG(n, UART_MCUX_IRQ_CFG_FUNC_INIT(n))
#else
#define UART_MCUX_CONFIG_FUNC(n)
#define UART_MCUX_IRQ_CFG_FUNC_INIT
#define UART_MCUX_INIT_CFG(n)						\
	UART_MCUX_DECLARE_CFG(n, UART_MCUX_IRQ_CFG_FUNC_INIT)
#endif

#define UART_MCUX_INIT(n)						\
	PINCTRL_DT_INST_DEFINE(n);					\
									\
	static struct uart_mcux_data uart_mcux_##n##_data = {		\
		.uart_cfg = {						\
			.stop_bits = UART_CFG_STOP_BITS_1,		\
			.data_bits = UART_CFG_DATA_BITS_8,		\
			.baudrate  = DT_INST_PROP(n, current_speed),	\
			.parity    = UART_CFG_PARITY_NONE,		\
			.flow_ctrl = DT_INST_PROP(n, hw_flow_control) ?	\
				UART_CFG_FLOW_CTRL_RTS_CTS : UART_CFG_FLOW_CTRL_NONE,\
		},							\
	};								\
									\
	static const struct uart_mcux_config uart_mcux_##n##_config;	\
	PM_DEVICE_DT_INST_DEFINE(n, uart_mcux_pm_action);\
									\
	DEVICE_DT_INST_DEFINE(n,					\
			    uart_mcux_init,				\
			    PM_DEVICE_DT_INST_GET(n),			\
			    &uart_mcux_##n##_data,			\
			    &uart_mcux_##n##_config,			\
			    PRE_KERNEL_1,				\
			    CONFIG_SERIAL_INIT_PRIORITY,		\
			    &uart_mcux_driver_api);			\
									\
	UART_MCUX_CONFIG_FUNC(n)					\
									\
	UART_MCUX_INIT_CFG(n);

DT_INST_FOREACH_STATUS_OKAY(UART_MCUX_INIT)
