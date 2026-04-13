/*
 * Copyright (c) 2021 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_i2c

#include <errno.h>
#include <stdint.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/i2c.h>
#ifdef CONFIG_I2C_RTIO
#include <zephyr/drivers/i2c/rtio.h>
#endif
#include <zephyr/sys/util.h>

#ifdef CONFIG_I2C_GD32_DMA
#include <string.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_gd32.h>
#endif

#include <gd32_i2c.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2c_gd32, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

/* Bus error */
#define I2C_GD32_ERR_BERR    BIT(0)
/* Arbitration lost */
#define I2C_GD32_ERR_LARB    BIT(1)
/* No ACK received */
#define I2C_GD32_ERR_AERR    BIT(2)
/* Over-run or under-run */
#define I2C_GD32_ERR_OUERR   BIT(3)
/* PEC error */
#define I2C_GD32_ERR_PECERR  BIT(4)
/* SMBus timeout */
#define I2C_GD32_ERR_SMBTO   BIT(5)
/* SMBus alert */
#define I2C_GD32_ERR_SMBALT  BIT(6)
/* DMA transfer error */
#define I2C_GD32_ERR_DMA     BIT(7)
/* I2C bus busy */
#define I2C_GD32_ERR_BUSY    BIT(8)
/* Transfer timeout */
#define I2C_GD32_ERR_TIMEOUT BIT(9)
/* Unexpected controller state/flag combination */
#define I2C_GD32_ERR_STATE   BIT(10)

/*
 * Conservative timeout: allow slow devices/clock stretching, but still recover
 * if the peripheral/IRQs become unresponsive.
 */
#define I2C_GD32_SYNC_TIMEOUT_BASE_US         200000U
#define I2C_GD32_SYNC_TIMEOUT_PER_BYTE_FACTOR 20U
#define I2C_GD32_SYNC_TIMEOUT_PER_BYTE_MIN_US 1000U
#define I2C_GD32_SYNC_TIMEOUT_MAX_MS          20000U
#define I2C_GD32_STOP_TIMEOUT_MS              100U
#define I2C_GD32_RECOVER_PULSE_DELAY_MS       1U

#ifdef CONFIG_I2C_GD32_DMA
struct i2c_gd32_dma_config {
	const struct device *dev;
	uint32_t channel;
	uint32_t slot;
	uint32_t config;
	uint32_t fifo_threshold;
};
#endif

struct i2c_gd32_config {
	uint32_t reg;
	uint32_t bitrate;
	uint16_t clkid;
	struct reset_dt_spec reset;
	struct gpio_dt_spec scl_gpios;
	struct gpio_dt_spec sda_gpios;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_cfg_func)(void);
#ifdef CONFIG_I2C_GD32_DMA
	struct i2c_gd32_dma_config dma_rx;
	struct i2c_gd32_dma_config dma_tx;
#endif
};

enum i2c_gd32_phase {
	I2C_GD32_PHASE_IDLE = 0,
	I2C_GD32_PHASE_START,
	I2C_GD32_PHASE_ADDR,
	I2C_GD32_PHASE_DATA_TX,
	I2C_GD32_PHASE_DATA_RX,
	I2C_GD32_PHASE_RESTART,
	I2C_GD32_PHASE_DMA,
	I2C_GD32_PHASE_COMPLETE,
	I2C_GD32_PHASE_ERROR,
	I2C_GD32_PHASE_RECOVERING,
};

enum i2c_gd32_frontend {
	I2C_GD32_FRONTEND_NONE = 0,
	I2C_GD32_FRONTEND_SYNC,
	I2C_GD32_FRONTEND_CALLBACK,
	I2C_GD32_FRONTEND_RTIO,
};

#ifdef CONFIG_I2C_TARGET
enum i2c_gd32_target_state {
	I2C_GD32_TARGET_IDLE = 0,
	I2C_GD32_TARGET_RECV,
	I2C_GD32_TARGET_SEND,
};
#endif

struct i2c_gd32_data {
	const struct device *dev;
	struct k_sem bus_mutex;
	struct k_sem sync_sem;
	struct k_spinlock lock;
	uint32_t dev_config;
	uint16_t addr1;
	uint16_t addr2;
	uint32_t xfer_len;
	struct i2c_msg *msgs;
	uint8_t num_msgs;
	uint8_t seg_next;
	struct i2c_msg *current;
	uint16_t errs;
	enum i2c_gd32_phase phase;
	enum i2c_gd32_frontend frontend;
	uint32_t xfer_cookie;
	bool xfer_addr_10_bits;
#ifdef CONFIG_I2C_GD32_DMA
	uint32_t dma_cookie;
#endif
	bool transfer_done;
	bool addr_is_read;
	bool ten_bit_read_restart;
#ifdef CONFIG_I2C_TARGET
	struct i2c_target_config *target_cfg;
	enum i2c_gd32_target_state target_state;
	uint8_t target_tx_byte;
	bool target_attached;
	bool target_first_tx;
	bool target_data_seen;
	bool target_pending_rx_valid;
	uint8_t target_pending_rx_byte;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	const uint8_t *target_tx_buf;
	size_t target_tx_len;
	size_t target_tx_pos;
	size_t target_rx_len;
	bool target_rx_overflow;
	uint8_t target_rx_buf[CONFIG_I2C_GD32_TARGET_BUFFER_SIZE];
#endif
#endif
#ifdef CONFIG_I2C_CALLBACK
	struct k_work async_work;
	struct k_work_delayable async_timeout_work;
	i2c_callback_t async_cb;
	void *async_userdata;
	bool async_active;
#endif
#ifdef CONFIG_I2C_RTIO
	struct i2c_rtio *ctx;
	struct k_work rtio_work;
	struct i2c_msg *rtio_msgs;
	uint8_t rtio_msg_capacity;
	uint8_t rtio_msg_count;
	bool rtio_complete_pending;
#endif
#ifdef CONFIG_I2C_GD32_DMA
	/* DMA is only used for single-message transfers. */
#endif
#ifdef CONFIG_I2C_GD32_DMA
	bool dma_active;
	bool dma_is_read;
	bool dma_done;
	bool dma_tx_done;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk;
	const struct device *dma_dev;
	uint32_t dma_channel;
#endif
};

enum i2c_gd32_seg_end {
	I2C_GD32_SEG_END_STOP = 0,
	I2C_GD32_SEG_END_RESTART,
};

#ifdef CONFIG_I2C_GD32_DMA
static void i2c_gd32_dma_cleanup(const struct device *dev);
#endif
static void i2c_gd32_finish_transfer(const struct device *dev, bool issue_stop);
static void i2c_gd32_abort_transfer(const struct device *dev, uint16_t errs, bool issue_stop);
static int i2c_gd32_xfer_end(const struct device *dev);
static int i2c_gd32_configure_locked(const struct device *dev, uint32_t dev_config);
static void i2c_gd32_transfer_release(const struct device *dev);
#ifdef CONFIG_I2C_CALLBACK
static int i2c_gd32_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				uint16_t addr, i2c_callback_t cb, void *userdata);
#endif
#ifdef CONFIG_I2C_RTIO
static void i2c_gd32_rtio_work_handler(struct k_work *work);
#endif
#ifdef CONFIG_I2C_TARGET
static void i2c_gd32_target_event_isr(const struct device *dev, uint32_t stat);
static void i2c_gd32_target_error_isr(const struct device *dev, uint32_t stat);
static int i2c_gd32_target_register(const struct device *dev, struct i2c_target_config *config);
static int i2c_gd32_target_unregister(const struct device *dev, struct i2c_target_config *config);
static void i2c_gd32_target_deliver_rx_byte(const struct device *dev, uint8_t val);
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
static inline bool i2c_gd32_target_use_read_buffer(const struct i2c_gd32_data *data)
{
	const struct i2c_target_callbacks *cb = data->target_cfg->callbacks;

	return (cb != NULL) && (cb->buf_read_requested != NULL);
}

static inline bool i2c_gd32_target_use_write_buffer(const struct i2c_gd32_data *data)
{
	const struct i2c_target_callbacks *cb = data->target_cfg->callbacks;

	return (cb != NULL) && (cb->buf_write_received != NULL);
}
#endif
#endif
static inline bool i2c_gd32_xfer_10_bit_addr(const struct i2c_gd32_data *data);

#ifdef CONFIG_I2C_RTIO
static inline bool i2c_gd32_rtio_pending(const struct i2c_gd32_data *data)
{
	struct i2c_rtio *ctx = data->ctx;
	k_spinlock_key_t key;
	bool pending;

	key = k_spin_lock(&ctx->slock);
	pending = (ctx->txn_head != NULL);
	k_spin_unlock(&ctx->slock, key);

	return pending;
}

static void i2c_gd32_rtio_kick(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;

	if (i2c_gd32_rtio_pending(data)) {
		(void)k_work_submit(&data->rtio_work);
	}
}
#endif

static void i2c_gd32_bus_unlock(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;

	k_sem_give(&data->bus_mutex);

#ifdef CONFIG_I2C_RTIO
	if (data->frontend == I2C_GD32_FRONTEND_NONE) {
		i2c_gd32_rtio_kick(dev);
	}
#endif
}

static uint32_t i2c_gd32_sync_timeout_ms_xfer(const struct device *dev, struct i2c_msg *msgs,
					      uint8_t num_msgs)
{
	const struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t bitrate_bps;
	uint32_t addr_bytes;
	uint32_t seg_cnt;
	uint64_t bytes_total;
	uint64_t per_byte_us;
	uint64_t per_byte_budget_us;
	uint64_t timeout_us;
	uint64_t timeout_ms;

	switch (I2C_SPEED_GET(data->dev_config)) {
	case I2C_SPEED_STANDARD:
		bitrate_bps = I2C_BITRATE_STANDARD;
		break;
	case I2C_SPEED_FAST:
		bitrate_bps = I2C_BITRATE_FAST;
		break;
	case I2C_SPEED_FAST_PLUS:
		bitrate_bps = I2C_BITRATE_FAST_PLUS;
		break;
	case I2C_SPEED_HIGH:
		bitrate_bps = I2C_BITRATE_HIGH;
		break;
	case I2C_SPEED_ULTRA:
		bitrate_bps = I2C_BITRATE_ULTRA;
		break;
	default:
		bitrate_bps = cfg->bitrate ? cfg->bitrate : I2C_BITRATE_STANDARD;
		break;
	}

	if (i2c_gd32_xfer_10_bit_addr(data)) {
		/* Worst-case: 10-bit master receive requires a repeated header. */
		addr_bytes = 3U;
	} else {
		addr_bytes = 1U;
	}

	seg_cnt = 1U;
	for (uint8_t i = 1U; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_RESTART) != 0U) {
			seg_cnt++;
		}
	}

	bytes_total = 0U;
	for (uint8_t i = 0U; i < num_msgs; i++) {
		bytes_total += msgs[i].len;
	}
	bytes_total += (uint64_t)addr_bytes * seg_cnt;
	per_byte_us = (9ULL * 1000000ULL + (bitrate_bps - 1U)) / bitrate_bps;
	per_byte_budget_us = MAX(I2C_GD32_SYNC_TIMEOUT_PER_BYTE_MIN_US,
				 per_byte_us * I2C_GD32_SYNC_TIMEOUT_PER_BYTE_FACTOR);
	timeout_us = I2C_GD32_SYNC_TIMEOUT_BASE_US + (per_byte_budget_us * (bytes_total + 2U));
	timeout_ms = (timeout_us + 999U) / 1000U;

	if (timeout_ms > I2C_GD32_SYNC_TIMEOUT_MAX_MS) {
		timeout_ms = I2C_GD32_SYNC_TIMEOUT_MAX_MS;
	}

	return (uint32_t)timeout_ms;
}

static inline bool i2c_gd32_phase_is_terminal(enum i2c_gd32_phase phase)
{
	return (phase == I2C_GD32_PHASE_COMPLETE) || (phase == I2C_GD32_PHASE_ERROR) ||
	       (phase == I2C_GD32_PHASE_RECOVERING);
}

static inline enum i2c_gd32_seg_end i2c_gd32_current_seg_end(const struct i2c_gd32_data *data)
{
	return (data->seg_next >= data->num_msgs) ? I2C_GD32_SEG_END_STOP : I2C_GD32_SEG_END_RESTART;
}

static inline void i2c_gd32_configure_interrupts(const struct i2c_gd32_config *cfg,
						 uint32_t irq_mask)
{
	uint32_t ctl1 = I2C_CTL1(cfg->reg);

	ctl1 &= ~(I2C_CTL1_ERRIE | I2C_CTL1_EVIE | I2C_CTL1_BUFIE);
	ctl1 |= irq_mask;
	I2C_CTL1(cfg->reg) = ctl1;
}

static inline void i2c_gd32_enable_start_interrupts(const struct i2c_gd32_config *cfg)
{
	i2c_gd32_configure_interrupts(cfg, I2C_CTL1_ERRIE | I2C_CTL1_EVIE);
}

#ifdef CONFIG_I2C_TARGET
static inline void i2c_gd32_enable_target_address_interrupts(const struct i2c_gd32_config *cfg)
{
	i2c_gd32_configure_interrupts(cfg, I2C_CTL1_ERRIE | I2C_CTL1_EVIE);
}

static inline void i2c_gd32_enable_target_data_interrupts(const struct i2c_gd32_config *cfg)
{
	i2c_gd32_configure_interrupts(cfg, I2C_CTL1_ERRIE | I2C_CTL1_EVIE | I2C_CTL1_BUFIE);
}
#endif

static inline void i2c_gd32_disable_interrupts(const struct i2c_gd32_config *cfg)
{
	i2c_gd32_configure_interrupts(cfg, 0U);
}

static inline void i2c_gd32_enable_data_interrupts(const struct device *dev)
{
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t irq_mask = I2C_CTL1_ERRIE | I2C_CTL1_EVIE;

#ifdef CONFIG_I2C_GD32_DMA
	struct i2c_gd32_data *data = dev->data;

	if (!data->dma_active) {
		irq_mask |= I2C_CTL1_BUFIE;
	}
#else
	irq_mask |= I2C_CTL1_BUFIE;
#endif

	i2c_gd32_configure_interrupts(cfg, irq_mask);
}

static inline enum i2c_gd32_phase i2c_gd32_data_phase_for_segment(struct i2c_gd32_data *data)
{
	return (data->current->flags & I2C_MSG_READ) != 0U ? I2C_GD32_PHASE_DATA_RX
							   : I2C_GD32_PHASE_DATA_TX;
}

static inline bool i2c_gd32_xfer_10_bit_addr(const struct i2c_gd32_data *data)
{
	return data->xfer_addr_10_bits || ((data->dev_config & I2C_ADDR_10_BITS) != 0U);
}

static inline void i2c_gd32_clear_addsend(const struct i2c_gd32_config *cfg)
{
	(void)I2C_STAT0(cfg->reg);
	(void)I2C_STAT1(cfg->reg);
}

static int i2c_gd32_validate_msgs(struct i2c_gd32_data *data, struct i2c_msg *msgs, uint8_t num_msgs,
				  bool *xfer_addr_10_bits)
{
	bool any_msg_10_bits = false;
	bool all_msg_10_bits = true;

	if ((msgs == NULL) || (num_msgs == 0U)) {
		return -EINVAL;
	}

	for (uint8_t i = 0U; i < num_msgs; i++) {
		if ((msgs[i].buf == NULL) || (msgs[i].len == 0U)) {
			return -EINVAL;
		}

		any_msg_10_bits |= (msgs[i].flags & I2C_MSG_ADDR_10_BITS) != 0U;
		all_msg_10_bits &= (msgs[i].flags & I2C_MSG_ADDR_10_BITS) != 0U;

		if (i < (num_msgs - 1U)) {
			if ((msgs[i].flags & I2C_MSG_STOP) != 0U) {
				return -EINVAL;
			}

			if (((msgs[i].flags & I2C_MSG_RW_MASK) !=
			     (msgs[i + 1U].flags & I2C_MSG_RW_MASK)) &&
			    ((msgs[i + 1U].flags & I2C_MSG_RESTART) == 0U)) {
				return -EINVAL;
			}
		}
	}

	if (any_msg_10_bits && !all_msg_10_bits) {
		return -EINVAL;
	}

	*xfer_addr_10_bits = any_msg_10_bits ? true : ((data->dev_config & I2C_ADDR_10_BITS) != 0U);

	return 0;
}

static int i2c_gd32_set_target_address(struct i2c_gd32_data *data, uint16_t addr)
{
	if (i2c_gd32_xfer_10_bit_addr(data)) {
		if (addr > BIT_MASK(10)) {
			return -EINVAL;
		}

		data->addr1 = 0x78U | ((addr & BITS(8, 9)) >> 8U);
		data->addr2 = addr & BITS(0, 7);
	} else {
		if (addr > BIT_MASK(7)) {
			return -EINVAL;
		}

		data->addr1 = addr & BITS(0, 6);
		data->addr2 = 0U;
	}

	return 0;
}

static void i2c_gd32_clear_error_flags(const struct i2c_gd32_config *cfg, uint32_t stat)
{
	if (stat & I2C_STAT0_BERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_BERR;
	}

	if (stat & I2C_STAT0_LOSTARB) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_LOSTARB;
	}

	if (stat & I2C_STAT0_AERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_AERR;
	}

	if (stat & I2C_STAT0_OUERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_OUERR;
	}

	if (stat & I2C_STAT0_PECERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_PECERR;
	}

	if (stat & I2C_STAT0_SMBTO) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_SMBTO;
	}

	if (stat & I2C_STAT0_SMBALT) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_SMBALT;
	}
}

static void i2c_gd32_quiesce_controller(const struct device *dev)
{
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t stat;

	/* Start each transfer from a quiet controller state with stale flags consumed. */
	i2c_gd32_disable_interrupts(cfg);

	I2C_CTL1(cfg->reg) &= ~(I2C_CTL1_DMAON | I2C_CTL1_DMALST);

	stat = I2C_STAT0(cfg->reg);
	i2c_gd32_clear_error_flags(cfg, stat);

	if (stat & I2C_STAT0_ADDSEND) {
		i2c_gd32_clear_addsend(cfg);
	}

	for (int i = 0; (i < 2) && (I2C_STAT0(cfg->reg) & I2C_STAT0_RBNE); i++) {
		(void)I2C_DATA(cfg->reg);
	}

	if (stat & (I2C_STAT0_SBSEND | I2C_STAT0_ADD10SEND | I2C_STAT0_BTC)) {
		I2C_CTL0(cfg->reg) &= ~I2C_CTL0_I2CEN;
		I2C_CTL0(cfg->reg) |= I2C_CTL0_I2CEN;
		(void)I2C_STAT0(cfg->reg);
		(void)I2C_STAT1(cfg->reg);
	}
}

static void i2c_gd32_segment_prepare(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	bool is_read = (data->current->flags & I2C_MSG_READ) != 0U;

	I2C_CTL1(cfg->reg) &= ~(I2C_CTL1_DMAON | I2C_CTL1_DMALST);
	I2C_CTL0(cfg->reg) &= ~I2C_CTL0_POAP;
	I2C_CTL0(cfg->reg) |= I2C_CTL0_ACKEN;

	data->addr_is_read = is_read;
	data->ten_bit_read_restart = false;
	data->phase = I2C_GD32_PHASE_IDLE;

	if (is_read) {
#ifdef CONFIG_I2C_GD32_DMA
		if (!data->dma_active && (data->xfer_len == 2U)) {
#else
		if (data->xfer_len == 2U) {
#endif
			I2C_CTL0(cfg->reg) |= I2C_CTL0_POAP;
		}

		if (i2c_gd32_xfer_10_bit_addr(data)) {
			/* 10-bit reads require a write header before the repeated read header. */
			data->ten_bit_read_restart = true;
			data->addr_is_read = false;
		}
	}
}

static int i2c_gd32_segment_init(const struct device *dev, uint8_t start_idx, enum i2c_gd32_seg_end *seg_end)
{
	struct i2c_gd32_data *data = dev->data;
	struct i2c_msg *msgs = data->msgs;
	uint8_t idx = start_idx;

	data->current = &msgs[start_idx];
	data->xfer_len = 0U;

	while (idx < data->num_msgs) {
		data->xfer_len += msgs[idx].len;
		idx++;
		if (idx >= data->num_msgs) {
			break;
		}
		if ((msgs[idx].flags & I2C_MSG_RESTART) != 0U) {
			break;
		}
		if ((msgs[idx].flags & I2C_MSG_RW_MASK) != (msgs[start_idx].flags & I2C_MSG_RW_MASK)) {
			break;
		}
	}

	data->seg_next = idx;
	*seg_end = (idx >= data->num_msgs) ? I2C_GD32_SEG_END_STOP : I2C_GD32_SEG_END_RESTART;
	i2c_gd32_segment_prepare(dev);

	return 0;
}

static void i2c_gd32_start_address_phase(const struct device *dev, bool repeated_start)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;

	data->phase = repeated_start ? I2C_GD32_PHASE_RESTART : I2C_GD32_PHASE_START;
	i2c_gd32_enable_start_interrupts(cfg);
	I2C_CTL0(cfg->reg) |= I2C_CTL0_START;
}

static void i2c_gd32_enter_data_phase(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;

#ifdef CONFIG_I2C_GD32_DMA
	if (data->dma_active) {
		data->phase = I2C_GD32_PHASE_DMA;
	} else {
		data->phase = i2c_gd32_data_phase_for_segment(data);
	}
#else
	data->phase = i2c_gd32_data_phase_for_segment(data);
#endif

	i2c_gd32_enable_data_interrupts(dev);
}

static void i2c_gd32_segment_complete(const struct device *dev, enum i2c_gd32_seg_end seg_end)
{
	struct i2c_gd32_data *data = dev->data;

	if (data->transfer_done) {
		return;
	}

	if (data->errs != 0U) {
		i2c_gd32_finish_transfer(dev, true);
		return;
	}

	if (seg_end == I2C_GD32_SEG_END_STOP) {
		i2c_gd32_finish_transfer(dev, true);
		return;
	}

	if (data->seg_next >= data->num_msgs) {
		i2c_gd32_abort_transfer(dev, I2C_GD32_ERR_STATE, true);
		return;
	}

	enum i2c_gd32_seg_end next_end;
	(void)i2c_gd32_segment_init(dev, data->seg_next, &next_end);
	i2c_gd32_start_address_phase(dev, true);
}

#ifdef CONFIG_I2C_RTIO
static inline void i2c_gd32_msg_from_rx(const struct rtio_iodev_sqe *iodev_sqe, struct i2c_msg *msg)
{
	msg->buf = iodev_sqe->sqe.rx.buf;
	msg->len = iodev_sqe->sqe.rx.buf_len;
	msg->flags = I2C_MSG_READ | iodev_sqe->sqe.iodev_flags;
}

static inline void i2c_gd32_msg_from_tx(const struct rtio_iodev_sqe *iodev_sqe, struct i2c_msg *msg)
{
	msg->buf = (uint8_t *)iodev_sqe->sqe.tx.buf;
	msg->len = iodev_sqe->sqe.tx.buf_len;
	msg->flags = iodev_sqe->sqe.iodev_flags;
}

static inline void i2c_gd32_msg_from_tiny_tx(const struct rtio_iodev_sqe *iodev_sqe,
					     struct i2c_msg *msg)
{
	msg->buf = (uint8_t *)iodev_sqe->sqe.tiny_tx.buf;
	msg->len = iodev_sqe->sqe.tiny_tx.buf_len;
	msg->flags = iodev_sqe->sqe.iodev_flags;
}

static int i2c_gd32_rtio_prepare_msgs(const struct device *dev, uint16_t *addr)
{
	struct i2c_gd32_data *data = dev->data;
	struct i2c_rtio *ctx = data->ctx;
	struct rtio_iodev_sqe *txn = ctx->txn_curr;
	const struct rtio_iodev *iodev;
	const struct i2c_dt_spec *dt_spec;

	if (txn == NULL) {
		return -EINVAL;
	}

	iodev = txn->sqe.iodev;
	dt_spec = iodev->data;
	*addr = dt_spec->addr;
	data->rtio_msg_count = 0U;

	do {
		struct i2c_msg *msg;

		if (data->rtio_msg_count >= data->rtio_msg_capacity) {
			return -ENOMEM;
		}

		if ((txn->sqe.iodev != iodev) ||
		    (((const struct i2c_dt_spec *)txn->sqe.iodev->data)->addr != *addr)) {
			return -EINVAL;
		}

		msg = &data->rtio_msgs[data->rtio_msg_count];

		switch (txn->sqe.op) {
		case RTIO_OP_RX:
			i2c_gd32_msg_from_rx(txn, msg);
			break;
		case RTIO_OP_TX:
			i2c_gd32_msg_from_tx(txn, msg);
			break;
		case RTIO_OP_TINY_TX:
			i2c_gd32_msg_from_tiny_tx(txn, msg);
			break;
		default:
			return -EINVAL;
		}

		data->rtio_msg_count++;
		txn = rtio_txn_next(txn);
	} while (txn != NULL);

	return 0;
}

static void i2c_gd32_rtio_sqe_signaled(struct rtio_iodev_sqe *iodev_sqe, void *userdata)
{
	const struct device *dev = userdata;
	struct i2c_gd32_data *data = dev->data;

	ARG_UNUSED(iodev_sqe);

	if (i2c_rtio_complete(data->ctx, 0)) {
		i2c_gd32_rtio_kick(dev);
	}
}
#endif

static int i2c_gd32_wait_not_busy(const struct i2c_gd32_config *cfg, uint32_t timeout_ms)
{
	uint32_t deadline = k_uptime_get_32() + timeout_ms;

	while (I2C_STAT1(cfg->reg) & I2C_STAT1_I2CBSY) {
		if ((int32_t)(k_uptime_get_32() - deadline) >= 0) {
			return -ETIMEDOUT;
		}
		k_msleep(1);
	}

	return 0;
}

static inline void i2c_gd32_xfer_read(struct i2c_gd32_data *data, const struct i2c_gd32_config *cfg)
{
	data->current->len--;
	*data->current->buf = I2C_DATA(cfg->reg);
	data->current->buf++;

	if ((data->xfer_len > 0U) && (data->current->len == 0U)) {
		data->current++;
	}
}

static inline void i2c_gd32_xfer_write(struct i2c_gd32_data *data,
				       const struct i2c_gd32_config *cfg)
{
	data->current->len--;
	I2C_DATA(cfg->reg) = *data->current->buf;
	data->current->buf++;

	if ((data->xfer_len > 0U) && (data->current->len == 0U)) {
		data->current++;
	}
}

static void i2c_gd32_handle_rbne(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	enum i2c_gd32_seg_end seg_end = i2c_gd32_current_seg_end(data);

	switch (data->xfer_len) {
	case 0:
		/* Drain the stale byte so RBNE does not keep retriggering the IRQ. */
		(void)I2C_DATA(cfg->reg);
		i2c_gd32_segment_complete(dev, seg_end);
		break;
	case 1:
		/* If total_read_length == 1, read the data directly. */
		data->xfer_len--;
		i2c_gd32_xfer_read(data, cfg);
		i2c_gd32_segment_complete(dev, seg_end);

		break;
	case 2:
		__fallthrough;
	case 3:
		/*
		 * Master receive Solution B from the GD32 reference manual:
		 * once only the final 2 or 3 bytes remain, stop consuming RBNE
		 * events and wait for BTC instead. BTC stretches SCL while both
		 * the data register and shift register are full, which gives the
		 * CPU time to flip ACKEN/STOP without the classic STM32-v1/GD32
		 * receive race.
		 */
		I2C_CTL1(cfg->reg) &= ~I2C_CTL1_BUFIE;
		break;
	default:
		/*
		 * If total_read_length > 3 and remaining_read_length > 3,
		 * read the data directly.
		 */
		data->xfer_len--;
		i2c_gd32_xfer_read(data, cfg);
		break;
	}
}

static void i2c_gd32_handle_tbe(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	enum i2c_gd32_seg_end seg_end = i2c_gd32_current_seg_end(data);

	if (data->xfer_len > 0U) {
		data->xfer_len--;
		if (data->xfer_len == 0U) {
			/*
			 * This is the last data to transmit, disable the TBE interrupt.
			 * Use the BTC interrupt to indicate the write data complete state.
			 */
			I2C_CTL1(cfg->reg) &= ~I2C_CTL1_BUFIE;
		}
		i2c_gd32_xfer_write(data, cfg);

	} else {
		i2c_gd32_segment_complete(dev, seg_end);
	}
}

static void i2c_gd32_handle_btc(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	enum i2c_gd32_seg_end seg_end = i2c_gd32_current_seg_end(data);

	if (data->current->flags & I2C_MSG_READ) {
		uint32_t counter = 0U;

		switch (data->xfer_len) {
		case 2:
			if (seg_end == I2C_GD32_SEG_END_STOP) {
				/*
				 * Solution B, N = 2 terminal case: issue STOP while BTC is
				 * stretching SCL, then drain the final two bytes.
				 */
				I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
			}

			for (counter = 2U; counter > 0; counter--) {
				data->xfer_len--;
				i2c_gd32_xfer_read(data, cfg);
			}
			i2c_gd32_segment_complete(dev, seg_end);

			break;
		case 3:
			/*
			 * Solution B, N = 3 terminal case: BTC is stretching SCL with N-2
			 * in DATA and N-1 in the shift register, so clear ACKEN before
			 * releasing the clock by reading N-2.
			 */
			I2C_CTL0(cfg->reg) &= ~I2C_CTL0_ACKEN;

			data->xfer_len--;
			i2c_gd32_xfer_read(data, cfg);

			break;
		default:
			i2c_gd32_handle_rbne(dev);
			break;
		}
	} else {
		i2c_gd32_handle_tbe(dev);
	}
}

static void i2c_gd32_handle_addsend(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	enum i2c_gd32_seg_end seg_end = i2c_gd32_current_seg_end(data);

	if ((data->current->flags & I2C_MSG_READ) && (data->xfer_len <= 2U) &&
#ifdef CONFIG_I2C_GD32_DMA
	    !data->dma_active
#else
	    true
#endif
	) {
		/*
		 * For 1-byte reads, and for 2-byte reads without DMA, the manual requires
		 * ACKEN to be cleared before ADDSEND is cleared. ADDSEND itself stretches
		 * SCL, so this is the safe place to do it.
		 */
		I2C_CTL0(cfg->reg) &= ~I2C_CTL0_ACKEN;
	}

	i2c_gd32_clear_addsend(cfg);

	if (data->ten_bit_read_restart) {
		data->ten_bit_read_restart = false;
		data->addr_is_read = true;
		i2c_gd32_start_address_phase(dev, true);
		return;
	}

	if ((seg_end == I2C_GD32_SEG_END_STOP) && (data->current->flags & I2C_MSG_READ) &&
	    (data->xfer_len == 1U)) {
		/* Enter stop condition */
		I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
	}

#ifdef CONFIG_I2C_GD32_DMA
	if (data->dma_active) {
		if (data->dma_is_read && (data->xfer_len >= 2U)) {
			I2C_CTL1(cfg->reg) |= I2C_CTL1_DMALST;
		} else {
			I2C_CTL1(cfg->reg) &= ~I2C_CTL1_DMALST;
		}

		/*
		 * DMA must be connected only after ADDSEND is cleared so the data phase
		 * starts with the controller and DMA state machines aligned.
		 */
		I2C_CTL1(cfg->reg) |= I2C_CTL1_DMAON;
	}
#endif

	i2c_gd32_enter_data_phase(dev);
}

static void i2c_gd32_handle_unexpected_event(const struct device *dev, uint32_t stat)
{
	const struct i2c_gd32_config *cfg = dev->config;

	if (stat & I2C_STAT0_ADDSEND) {
		i2c_gd32_clear_addsend(cfg);
	}

	if (stat & I2C_STAT0_RBNE) {
		(void)I2C_DATA(cfg->reg);
	}

	i2c_gd32_abort_transfer(dev, I2C_GD32_ERR_STATE, true);
}

static void i2c_gd32_event_isr(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t stat;

#ifdef CONFIG_I2C_TARGET
	if (data->target_attached) {
		stat = I2C_STAT0(cfg->reg);
		i2c_gd32_target_event_isr(dev, stat);
		return;
	}
#endif

	if (data->transfer_done || i2c_gd32_phase_is_terminal(data->phase)) {
		i2c_gd32_disable_interrupts(cfg);
		return;
	}

	stat = I2C_STAT0(cfg->reg);

	switch (data->phase) {
	case I2C_GD32_PHASE_START:
	case I2C_GD32_PHASE_RESTART:
		if (stat & I2C_STAT0_SBSEND) {
			data->phase = I2C_GD32_PHASE_ADDR;
			I2C_DATA(cfg->reg) = (data->addr1 << 1U) | (data->addr_is_read ? 1U : 0U);
			return;
		}

		/*
		 * On a repeated-start after a write segment, BTC/TBE can remain asserted
		 * until the START condition takes effect. Treat them as stale while waiting
		 * for SBSEND instead of aborting the transfer.
		 */
		if (stat & (I2C_STAT0_ADD10SEND | I2C_STAT0_ADDSEND | I2C_STAT0_RBNE)) {
			i2c_gd32_handle_unexpected_event(dev, stat);
		}

		return;
	case I2C_GD32_PHASE_ADDR:
		if (stat & I2C_STAT0_ADD10SEND) {
			I2C_DATA(cfg->reg) = data->addr2;
			return;
		}

		if (stat & I2C_STAT0_ADDSEND) {
			i2c_gd32_handle_addsend(dev);
			return;
		}

		if (stat & (I2C_STAT0_SBSEND | I2C_STAT0_BTC | I2C_STAT0_RBNE | I2C_STAT0_TBE)) {
			i2c_gd32_handle_unexpected_event(dev, stat);
		}

		return;
	case I2C_GD32_PHASE_DATA_TX:
		if (stat & I2C_STAT0_BTC) {
			i2c_gd32_handle_btc(dev);
			return;
		}

		if (stat & I2C_STAT0_TBE) {
			i2c_gd32_handle_tbe(dev);
			return;
		}

		if (stat & (I2C_STAT0_SBSEND | I2C_STAT0_ADD10SEND | I2C_STAT0_ADDSEND |
			    I2C_STAT0_RBNE)) {
			i2c_gd32_handle_unexpected_event(dev, stat);
		}

		return;
	case I2C_GD32_PHASE_DATA_RX:
		/*
		 * BTC is a superset of RBNE/TBE on this controller, so the receive terminal
		 * sequence must look for it first once a received byte is actually pending.
		 * On 10-bit reads a stale BTC can remain set across the repeated-address phase,
		 * so BTC without RBNE must not be treated as data-ready.
		 */
		if ((stat & I2C_STAT0_BTC) && (stat & I2C_STAT0_RBNE)) {
			i2c_gd32_handle_btc(dev);
			return;
		}

		if (stat & I2C_STAT0_RBNE) {
			i2c_gd32_handle_rbne(dev);
			return;
		}

		if (stat & (I2C_STAT0_SBSEND | I2C_STAT0_ADD10SEND | I2C_STAT0_ADDSEND |
			    I2C_STAT0_TBE)) {
			i2c_gd32_handle_unexpected_event(dev, stat);
		}

		return;
	case I2C_GD32_PHASE_DMA:
#ifdef CONFIG_I2C_GD32_DMA
		if (stat & I2C_STAT0_BTC) {
			if (!data->dma_is_read && data->dma_tx_done) {
				i2c_gd32_finish_transfer(dev, true);
			}

			return;
		}
#endif

		if (stat & (I2C_STAT0_SBSEND | I2C_STAT0_ADD10SEND | I2C_STAT0_ADDSEND |
			    I2C_STAT0_RBNE | I2C_STAT0_TBE)) {
			i2c_gd32_handle_unexpected_event(dev, stat);
		}

		return;
	default:
		i2c_gd32_handle_unexpected_event(dev, stat);
		return;
	}
}

static void i2c_gd32_error_isr(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t stat;
	uint16_t errs = 0U;

	stat = I2C_STAT0(cfg->reg);

#ifdef CONFIG_I2C_TARGET
	if (data->target_attached) {
		i2c_gd32_target_error_isr(dev, stat);
		return;
	}
#endif

	if (data->transfer_done || i2c_gd32_phase_is_terminal(data->phase)) {
		i2c_gd32_clear_error_flags(cfg, stat);
		i2c_gd32_disable_interrupts(cfg);
		return;
	}

	if (stat & I2C_STAT0_BERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_BERR;
		errs |= I2C_GD32_ERR_BERR;
	}

	if (stat & I2C_STAT0_LOSTARB) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_LOSTARB;
		errs |= I2C_GD32_ERR_LARB;
	}

	if (stat & I2C_STAT0_AERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_AERR;
		errs |= I2C_GD32_ERR_AERR;
	}

	if (stat & I2C_STAT0_OUERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_OUERR;
		errs |= I2C_GD32_ERR_OUERR;
	}

	if (stat & I2C_STAT0_PECERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_PECERR;
		errs |= I2C_GD32_ERR_PECERR;
	}

	if (stat & I2C_STAT0_SMBTO) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_SMBTO;
		errs |= I2C_GD32_ERR_SMBTO;
	}

	if (stat & I2C_STAT0_SMBALT) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_SMBALT;
		errs |= I2C_GD32_ERR_SMBALT;
	}

	if (errs != 0U) {
		data->errs |= errs;
		i2c_gd32_finish_transfer(dev, true);
	}
}

static void i2c_gd32_log_err(struct i2c_gd32_data *data)
{
	if (data->errs & I2C_GD32_ERR_BERR) {
		LOG_ERR("Bus error");
	}

	if (data->errs & I2C_GD32_ERR_LARB) {
		LOG_ERR("Arbitration lost");
	}

	if (data->errs & I2C_GD32_ERR_AERR) {
		LOG_ERR("No ACK received");
	}

	if (data->errs & I2C_GD32_ERR_OUERR) {
		LOG_ERR("Over-run or under-run");
	}

	if (data->errs & I2C_GD32_ERR_PECERR) {
		LOG_ERR("PEC error");
	}

	if (data->errs & I2C_GD32_ERR_SMBTO) {
		LOG_ERR("SMBus timeout");
	}

	if (data->errs & I2C_GD32_ERR_SMBALT) {
		LOG_ERR("SMBus alert");
	}

	if (data->errs & I2C_GD32_ERR_DMA) {
		LOG_ERR("DMA error");
	}

	if (data->errs & I2C_GD32_ERR_BUSY) {
		LOG_ERR("I2C bus busy");
	}

	if (data->errs & I2C_GD32_ERR_TIMEOUT) {
		LOG_ERR("Transfer timeout");
	}

	if (data->errs & I2C_GD32_ERR_STATE) {
		LOG_ERR("Unexpected controller state");
	}
}

static int i2c_gd32_recover_bus_bitbang(const struct device *dev)
{
	const struct i2c_gd32_config *cfg = dev->config;
	int ret;
	int sda;

	if ((cfg->scl_gpios.port == NULL) || (cfg->sda_gpios.port == NULL)) {
		return -ENOTSUP;
	}

	if (!device_is_ready(cfg->scl_gpios.port) || !device_is_ready(cfg->sda_gpios.port)) {
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->scl_gpios,
				    GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN | GPIO_PULL_UP);
	if (ret != 0) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&cfg->sda_gpios,
				    GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN | GPIO_PULL_UP);
	if (ret != 0) {
		return ret;
	}

	sda = gpio_pin_get_dt(&cfg->sda_gpios);
	if (sda < 0) {
		return sda;
	}

	/*
	 * Recover a wedged target by toggling SCL a few cycles while releasing SDA,
	 * then forcing a STOP condition.
	 */
	for (int i = 0; (i < 9) && (sda == 0); ++i) {
		(void)gpio_pin_set_dt(&cfg->scl_gpios, 0);
		k_msleep(I2C_GD32_RECOVER_PULSE_DELAY_MS);
		(void)gpio_pin_set_dt(&cfg->scl_gpios, 1);
		k_msleep(I2C_GD32_RECOVER_PULSE_DELAY_MS);

		sda = gpio_pin_get_dt(&cfg->sda_gpios);
		if (sda < 0) {
			return sda;
		}
	}

	(void)gpio_pin_set_dt(&cfg->sda_gpios, 0);
	k_msleep(I2C_GD32_RECOVER_PULSE_DELAY_MS);
	(void)gpio_pin_set_dt(&cfg->scl_gpios, 1);
	k_msleep(I2C_GD32_RECOVER_PULSE_DELAY_MS);
	(void)gpio_pin_set_dt(&cfg->sda_gpios, 1);
	k_msleep(I2C_GD32_RECOVER_PULSE_DELAY_MS);

	sda = gpio_pin_get_dt(&cfg->sda_gpios);
	if (sda < 0) {
		return sda;
	}

	return (sda != 0) ? 0 : -EBUSY;
}

#ifdef CONFIG_I2C_GD32_DMA
static void i2c_gd32_dma_callback(const struct device *dma_dev, void *user_data, uint32_t channel,
				  int status)
{
	const struct device *dev = user_data;
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;

	if (!data->dma_active || (dma_dev != data->dma_dev) || (channel != data->dma_channel) ||
	    (data->dma_cookie != data->xfer_cookie) || i2c_gd32_phase_is_terminal(data->phase)) {
		return;
	}

	if (status < 0) {
		data->errs |= I2C_GD32_ERR_DMA;
		i2c_gd32_finish_transfer(dev, true);
		return;
	}

	data->dma_done = true;

	if (data->dma_is_read) {
		bool issue_stop = data->xfer_len >= 2U;

		if (issue_stop) {
			I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
		}
		i2c_gd32_finish_transfer(dev, false);
	} else {
		I2C_CTL1(cfg->reg) &= ~I2C_CTL1_DMAON;
		data->dma_tx_done = true;
		if (I2C_STAT0(cfg->reg) & I2C_STAT0_BTC) {
			i2c_gd32_finish_transfer(dev, true);
		}
	}
}

static bool i2c_gd32_dma_available(const struct i2c_gd32_dma_config *dma)
{
	return (dma->dev != NULL) && device_is_ready(dma->dev);
}

static int i2c_gd32_dma_setup(const struct device *dev, bool is_read)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	const struct i2c_gd32_dma_config *dma = is_read ? &cfg->dma_rx : &cfg->dma_tx;
	int ret;

	if (!i2c_gd32_dma_available(dma)) {
		return -ENODEV;
	}

	if (data->xfer_len > UINT16_MAX) {
		return -ENOTSUP;
	}

	memset(&data->dma_cfg, 0, sizeof(data->dma_cfg));
	memset(&data->dma_blk, 0, sizeof(data->dma_blk));

	data->dma_blk.block_size = data->xfer_len;

	data->dma_cfg.source_burst_length = 1;
	data->dma_cfg.dest_burst_length = 1;
	data->dma_cfg.user_data = (void *)dev;
	data->dma_cfg.dma_callback = i2c_gd32_dma_callback;
	data->dma_cfg.block_count = 1U;
	data->dma_cfg.head_block = &data->dma_blk;
	data->dma_cfg.dma_slot = dma->slot;
	data->dma_cfg.channel_priority = GD32_DMA_CONFIG_PRIORITY(dma->config);
	data->dma_cfg.source_data_size = 1;
	data->dma_cfg.dest_data_size = 1;

	if (is_read) {
		data->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		data->dma_blk.source_address = (uint32_t)&I2C_DATA(cfg->reg);
		data->dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		data->dma_blk.dest_address = (uint32_t)data->current->buf;
		data->dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	} else {
		data->dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		data->dma_blk.source_address = (uint32_t)data->current->buf;
		data->dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		data->dma_blk.dest_address = (uint32_t)&I2C_DATA(cfg->reg);
		data->dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	}

	ret = dma_config(dma->dev, dma->channel, &data->dma_cfg);
	if (ret != 0) {
		return ret;
	}

	ret = dma_start(dma->dev, dma->channel);
	if (ret != 0) {
		return ret;
	}

	data->dma_dev = dma->dev;
	data->dma_channel = dma->channel;
	data->dma_is_read = is_read;
	data->dma_done = false;
	data->dma_tx_done = false;
	data->dma_active = true;
	data->dma_cookie = data->xfer_cookie;

	return 0;
}

static void i2c_gd32_dma_cleanup(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;

	if (!data->dma_active) {
		return;
	}

	(void)dma_stop(data->dma_dev, data->dma_channel);
	I2C_CTL1(cfg->reg) &= ~(I2C_CTL1_DMAON | I2C_CTL1_DMALST);
	data->dma_active = false;
	data->dma_done = false;
	data->dma_tx_done = false;
	data->dma_cookie = 0U;
}
#endif /* CONFIG_I2C_GD32_DMA */

static void i2c_gd32_finish_transfer(const struct device *dev, bool issue_stop)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	k_spinlock_key_t key;
	enum i2c_gd32_frontend frontend;

	/*
	 * The ISR must leave the peripheral quiet before waking the waiting thread,
	 * otherwise a still-pending source can tail-chain forever and starve cleanup.
	 */
	key = k_spin_lock(&data->lock);
	if (data->transfer_done) {
		k_spin_unlock(&data->lock, key);
		i2c_gd32_disable_interrupts(cfg);
		return;
	}
	data->phase = (data->errs != 0U) ? I2C_GD32_PHASE_ERROR : I2C_GD32_PHASE_COMPLETE;
	data->transfer_done = true;
	frontend = data->frontend;
#ifdef CONFIG_I2C_RTIO
	if (frontend == I2C_GD32_FRONTEND_RTIO) {
		data->rtio_complete_pending = true;
	}
#endif
	k_spin_unlock(&data->lock, key);

	i2c_gd32_disable_interrupts(cfg);

#ifdef CONFIG_I2C_GD32_DMA
	i2c_gd32_dma_cleanup(dev);
#endif

	if (issue_stop) {
		I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
	}

#ifdef CONFIG_I2C_CALLBACK
	if (frontend == I2C_GD32_FRONTEND_CALLBACK) {
		(void)k_work_submit(&data->async_work);
		return;
	}
#endif

#ifdef CONFIG_I2C_RTIO
	if (frontend == I2C_GD32_FRONTEND_RTIO) {
		(void)k_work_submit(&data->rtio_work);
		return;
	}
#endif

	k_sem_give(&data->sync_sem);
}

static void i2c_gd32_abort_transfer(const struct device *dev, uint16_t errs, bool issue_stop)
{
	struct i2c_gd32_data *data = dev->data;

	data->errs |= errs;
	i2c_gd32_finish_transfer(dev, issue_stop);
}

#ifdef CONFIG_I2C_TARGET
static void i2c_gd32_target_reset_session(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;

	data->target_state = I2C_GD32_TARGET_IDLE;
	data->target_tx_byte = 0U;
	data->target_first_tx = false;
	data->target_data_seen = false;
	data->target_pending_rx_valid = false;
	data->target_pending_rx_byte = 0U;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	data->target_tx_buf = NULL;
	data->target_tx_len = 0U;
	data->target_tx_pos = 0U;
	data->target_rx_len = 0U;
	data->target_rx_overflow = false;
#endif
	I2C_CTL1(cfg->reg) &= ~I2C_CTL1_BUFIE;
	I2C_CTL0(cfg->reg) &= ~I2C_CTL0_POAP;
	I2C_CTL0(cfg->reg) |= I2C_CTL0_ACKEN;
}

static void i2c_gd32_target_notify_stop(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;

	if (!data->target_attached) {
		return;
	}

	if ((data->target_state == I2C_GD32_TARGET_RECV) && data->target_pending_rx_valid) {
		i2c_gd32_target_deliver_rx_byte(dev, data->target_pending_rx_byte);
		data->target_pending_rx_valid = false;
	}

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	if ((data->target_state == I2C_GD32_TARGET_RECV) &&
	    i2c_gd32_target_use_write_buffer(data) &&
	    (data->target_cfg->callbacks != NULL) &&
	    (data->target_cfg->callbacks->buf_write_received != NULL) &&
	    !data->target_rx_overflow) {
		data->target_cfg->callbacks->buf_write_received(data->target_cfg, data->target_rx_buf,
								 data->target_rx_len);
	}
#endif

	if ((data->target_state != I2C_GD32_TARGET_IDLE) &&
	    (data->target_cfg->callbacks != NULL) &&
	    (data->target_cfg->callbacks->stop != NULL)) {
		(void)data->target_cfg->callbacks->stop(data->target_cfg);
	}

	i2c_gd32_target_reset_session(dev);
}

static void i2c_gd32_target_report_error(const struct device *dev, enum i2c_error_reason reason)
{
	struct i2c_gd32_data *data = dev->data;

	if (data->target_attached && (data->target_cfg->callbacks != NULL) &&
	    (data->target_cfg->callbacks->error != NULL)) {
		data->target_cfg->callbacks->error(data->target_cfg, reason);
	}
}

static void i2c_gd32_target_clear_stop(const struct device *dev)
{
	const struct i2c_gd32_config *cfg = dev->config;

	(void)I2C_STAT0(cfg->reg);
	I2C_CTL0(cfg->reg) |= I2C_CTL0_I2CEN;
}

static void i2c_gd32_target_deliver_rx_byte(const struct device *dev, uint8_t val)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	const struct i2c_target_callbacks *cb = data->target_cfg->callbacks;

	data->target_data_seen = true;

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	if (i2c_gd32_target_use_write_buffer(data)) {
		if (data->target_rx_len < ARRAY_SIZE(data->target_rx_buf)) {
			data->target_rx_buf[data->target_rx_len++] = val;
			I2C_CTL0(cfg->reg) |= I2C_CTL0_ACKEN;
		} else {
			data->target_rx_overflow = true;
			I2C_CTL0(cfg->reg) &= ~I2C_CTL0_ACKEN;
		}
		return;
	}
#endif

	if ((cb != NULL) && (cb->write_received != NULL) &&
	    (cb->write_received(data->target_cfg, val) < 0)) {
		I2C_CTL0(cfg->reg) &= ~I2C_CTL0_ACKEN;
	} else {
		I2C_CTL0(cfg->reg) |= I2C_CTL0_ACKEN;
	}
}

static void i2c_gd32_target_load_tx_byte(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_target_callbacks *cb = data->target_cfg->callbacks;
	uint8_t val = 0U;
	int ret = 0;

#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
	if (i2c_gd32_target_use_read_buffer(data)) {
		if (data->target_tx_pos < data->target_tx_len) {
			val = data->target_tx_buf[data->target_tx_pos++];
		} else {
			val = 0xFFU;
		}

		data->target_tx_byte = val;
		return;
	}
#endif

	if (data->target_first_tx) {
		if ((cb != NULL) && (cb->read_requested != NULL)) {
			ret = cb->read_requested(data->target_cfg, &val);
		}
		data->target_first_tx = false;
	} else if ((cb != NULL) && (cb->read_processed != NULL)) {
		ret = cb->read_processed(data->target_cfg, &val);
	}

	if (ret < 0) {
		val = 0xFFU;
	}

	data->target_tx_byte = val;
}

static void i2c_gd32_target_handle_address(const struct device *dev, uint32_t stat)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	const struct i2c_target_callbacks *cb = data->target_cfg->callbacks;
	enum i2c_gd32_target_state prev_state = data->target_state;
	bool prev_data_seen = data->target_data_seen;
	bool is_10bit = (data->target_cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U;
	uint32_t stat1;
	bool is_read;
	int ret = 0;

	ARG_UNUSED(stat);

	(void)I2C_STAT0(cfg->reg);
	stat1 = I2C_STAT1(cfg->reg);
	is_read = (stat1 & I2C_STAT1_TR) != 0U;

	if ((prev_state != I2C_GD32_TARGET_IDLE) &&
	    !(is_10bit && !prev_data_seen && (prev_state == I2C_GD32_TARGET_RECV) && is_read)) {
		i2c_gd32_target_notify_stop(dev);
	}

	if (is_10bit && !prev_data_seen && (prev_state == I2C_GD32_TARGET_RECV) && is_read) {
		data->target_pending_rx_valid = false;
	}

	data->target_data_seen = false;

	if (is_read) {
		data->target_state = I2C_GD32_TARGET_SEND;
		data->target_first_tx = true;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		if (i2c_gd32_target_use_read_buffer(data)) {
			uint8_t *buf = NULL;
			uint32_t len = 0U;

			ret = cb->buf_read_requested(data->target_cfg, &buf, &len);
			if ((ret == 0) && (len > 0U) && (buf == NULL)) {
				ret = -EINVAL;
			}

			if (ret == 0) {
				data->target_tx_buf = buf;
				data->target_tx_len = len;
				data->target_tx_pos = 0U;
			} else {
				data->target_tx_buf = NULL;
				data->target_tx_len = 0U;
				data->target_tx_pos = 0U;
			}
		}
#endif
	} else {
		data->target_state = I2C_GD32_TARGET_RECV;
#ifdef CONFIG_I2C_TARGET_BUFFER_MODE
		data->target_rx_len = 0U;
		data->target_rx_overflow = false;
		if (i2c_gd32_target_use_write_buffer(data)) {
			ret = 0;
		} else
#endif
		if ((cb != NULL) && (cb->write_requested != NULL)) {
			ret = cb->write_requested(data->target_cfg);
		}
		if (ret < 0) {
			I2C_CTL0(cfg->reg) &= ~I2C_CTL0_ACKEN;
		}
	}

	i2c_gd32_enable_target_data_interrupts(cfg);
}

static void i2c_gd32_target_event_isr(const struct device *dev, uint32_t stat)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;

	if (stat & I2C_STAT0_ADDSEND) {
		i2c_gd32_target_handle_address(dev, stat);
		return;
	}

	if (stat & I2C_STAT0_STPDET) {
		i2c_gd32_target_clear_stop(dev);
		i2c_gd32_target_notify_stop(dev);
		return;
	}

	if ((data->target_state == I2C_GD32_TARGET_RECV) && (stat & I2C_STAT0_RBNE)) {
		bool is_10bit = (data->target_cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U;
		uint8_t val = I2C_DATA(cfg->reg);

		if (is_10bit && !data->target_data_seen) {
			if (!data->target_pending_rx_valid) {
				data->target_pending_rx_valid = true;
				data->target_pending_rx_byte = val;
				return;
			}

			i2c_gd32_target_deliver_rx_byte(dev, data->target_pending_rx_byte);
			data->target_pending_rx_valid = false;
		}

		i2c_gd32_target_deliver_rx_byte(dev, val);
		return;
	}

	if ((data->target_state == I2C_GD32_TARGET_SEND) &&
	    (stat & (I2C_STAT0_TBE | I2C_STAT0_BTC)) &&
	    ((stat & I2C_STAT0_AERR) == 0U)) {
		data->target_data_seen = true;
		i2c_gd32_target_load_tx_byte(dev);
		I2C_DATA(cfg->reg) = data->target_tx_byte;
	}
}

static void i2c_gd32_target_error_isr(const struct device *dev, uint32_t stat)
{
	const struct i2c_gd32_config *cfg = dev->config;

	if (stat & I2C_STAT0_AERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_AERR;
		i2c_gd32_target_notify_stop(dev);
		return;
	}

	if (stat & I2C_STAT0_BERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_BERR;
		i2c_gd32_target_report_error(dev, I2C_ERROR_GENERIC);
	}

	if (stat & I2C_STAT0_LOSTARB) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_LOSTARB;
		i2c_gd32_target_report_error(dev, I2C_ERROR_ARBITRATION);
	}

	if (stat & I2C_STAT0_OUERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_OUERR;
		i2c_gd32_target_report_error(dev, I2C_ERROR_GENERIC);
	}

	if (stat & I2C_STAT0_PECERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_PECERR;
		i2c_gd32_target_report_error(dev, I2C_ERROR_GENERIC);
	}

	if (stat & I2C_STAT0_SMBTO) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_SMBTO;
		i2c_gd32_target_report_error(dev, I2C_ERROR_TIMEOUT);
	}

	if (stat & I2C_STAT0_SMBALT) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_SMBALT;
		i2c_gd32_target_report_error(dev, I2C_ERROR_GENERIC);
	}

	i2c_gd32_target_reset_session(dev);
}

static int i2c_gd32_target_register(const struct device *dev, struct i2c_target_config *config)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	int ret = 0;

	if ((config == NULL) || (config->callbacks == NULL)) {
		return -EINVAL;
	}

	if ((config->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U) {
		if (config->address > BIT_MASK(10)) {
			return -EINVAL;
		}
	} else if (config->address > 0x7FU) {
		return -EINVAL;
	}

	k_sem_take(&data->bus_mutex, K_FOREVER);

	if (data->target_attached) {
		ret = -EBUSY;
		goto out;
	}

	i2c_gd32_quiesce_controller(dev);
	I2C_CTL0(cfg->reg) |= I2C_CTL0_I2CEN | I2C_CTL0_ACKEN;
	if ((config->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) != 0U) {
		I2C_SADDR0(cfg->reg) = I2C_SADDR0_ADDFORMAT | (config->address & BIT_MASK(10));
	} else {
		I2C_SADDR0(cfg->reg) = (uint32_t)(config->address << 1U);
	}

	data->target_cfg = config;
	data->target_attached = true;
	i2c_gd32_target_reset_session(dev);
	i2c_gd32_enable_target_address_interrupts(cfg);

out:
	k_sem_give(&data->bus_mutex);

	return ret;
}

static int i2c_gd32_target_unregister(const struct device *dev, struct i2c_target_config *config)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	int ret = 0;

	k_sem_take(&data->bus_mutex, K_FOREVER);

	if (!data->target_attached || (data->target_cfg != config)) {
		ret = -EINVAL;
		goto out;
	}

	if ((data->target_state != I2C_GD32_TARGET_IDLE) || (I2C_STAT1(cfg->reg) & I2C_STAT1_I2CBSY)) {
		ret = -EBUSY;
		goto out;
	}

	i2c_gd32_disable_interrupts(cfg);
	i2c_gd32_target_reset_session(dev);
	data->target_cfg = NULL;
	data->target_attached = false;
	I2C_CTL0(cfg->reg) &= ~I2C_CTL0_ACKEN;
	I2C_CTL0(cfg->reg) &= ~I2C_CTL0_I2CEN;

out:
	k_sem_give(&data->bus_mutex);

	return ret;
}
#endif

static int i2c_gd32_recover_bus_locked(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t ctl0 = I2C_CTL0(cfg->reg);
	uint32_t ctl1 = I2C_CTL1(cfg->reg);
	uint32_t ckcfg = I2C_CKCFG(cfg->reg);
	uint32_t rt = I2C_RT(cfg->reg);
	uint32_t fctl = I2C_FCTL(cfg->reg);
#ifdef I2C_FMPCFG
	uint32_t fmpcfg = I2C_FMPCFG(cfg->reg);
	#endif
	int ret;

	data->phase = I2C_GD32_PHASE_RECOVERING;
	data->transfer_done = true;
	data->xfer_cookie++;
	i2c_gd32_disable_interrupts(cfg);
	k_sem_reset(&data->sync_sem);

#ifdef CONFIG_I2C_GD32_DMA
	i2c_gd32_dma_cleanup(dev);
#endif

	/* Try to abort any active transaction and release the bus. */
	I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
	(void)i2c_gd32_wait_not_busy(cfg, I2C_GD32_STOP_TIMEOUT_MS);

	/* Reset I2C internal state machine. */
	I2C_CTL0(cfg->reg) &= ~I2C_CTL0_I2CEN;

	ret = i2c_gd32_recover_bus_bitbang(dev);
	if ((ret != 0) && (ret != -ENOTSUP)) {
		LOG_WRN("Bus bitbang recovery failed: %d", ret);
	}
	(void)pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

	I2C_CTL0(cfg->reg) |= I2C_CTL0_SRESET;
	I2C_CTL0(cfg->reg) &= ~I2C_CTL0_SRESET;

	/* Full peripheral reset for a wedged controller, then restore timing config. */
	(void)reset_line_toggle_dt(&cfg->reset);

	I2C_CTL0(cfg->reg) = (ctl0 & (I2C_CTL0_SMBEN | I2C_CTL0_SMBSEL | I2C_CTL0_ARPEN |
				      I2C_CTL0_PECEN | I2C_CTL0_GCEN | I2C_CTL0_SS |
				      I2C_CTL0_PECTRANS | I2C_CTL0_SALT)) |
			     I2C_CTL0_I2CEN | I2C_CTL0_ACKEN;
	I2C_CTL1(cfg->reg) = ctl1 & ~(I2C_CTL1_ERRIE | I2C_CTL1_EVIE | I2C_CTL1_BUFIE |
				      I2C_CTL1_DMAON | I2C_CTL1_DMALST);
	I2C_CKCFG(cfg->reg) = ckcfg;
	I2C_RT(cfg->reg) = rt;
	I2C_FCTL(cfg->reg) = fctl;
#ifdef I2C_FMPCFG
	I2C_FMPCFG(cfg->reg) = fmpcfg;
#endif

	(void)I2C_STAT0(cfg->reg);
	(void)I2C_STAT1(cfg->reg);

	ret = i2c_gd32_wait_not_busy(cfg, I2C_GD32_STOP_TIMEOUT_MS);
	if (ret != 0) {
		data->errs |= I2C_GD32_ERR_BUSY;
	}

	data->phase = I2C_GD32_PHASE_IDLE;

	return ret;
}

static int i2c_gd32_recover_bus(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	int ret;

	k_sem_take(&data->bus_mutex, K_FOREVER);
#ifdef CONFIG_I2C_TARGET
	if (data->target_attached) {
		i2c_gd32_bus_unlock(dev);
		return -EBUSY;
	}
#endif
	ret = i2c_gd32_recover_bus_locked(dev);
	i2c_gd32_bus_unlock(dev);

	return ret;
}

static void i2c_gd32_xfer_begin(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;

	k_sem_reset(&data->sync_sem);

	data->errs = 0U;
	data->transfer_done = false;
	data->phase = I2C_GD32_PHASE_IDLE;
#ifdef CONFIG_I2C_RTIO
	data->rtio_complete_pending = false;
#endif
	i2c_gd32_quiesce_controller(dev);
	i2c_gd32_segment_prepare(dev);

	i2c_gd32_start_address_phase(dev, false);
}

static int i2c_gd32_transfer_prepare(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				     uint16_t addr, uint32_t *timeout_ms)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	enum i2c_gd32_seg_end seg_end;
	int err;

	err = i2c_gd32_validate_msgs(data, msgs, num_msgs, &data->xfer_addr_10_bits);
	if (err != 0) {
		return err;
	}

	err = i2c_gd32_set_target_address(data, addr);
	if (err != 0) {
		return err;
	}

	/* Enable i2c device */
	I2C_CTL0(cfg->reg) |= I2C_CTL0_I2CEN;
	data->xfer_cookie++;
	data->errs = 0U;
	data->phase = I2C_GD32_PHASE_IDLE;
	data->transfer_done = true;
	data->msgs = msgs;
	data->num_msgs = num_msgs;

	if (I2C_STAT1(cfg->reg) & I2C_STAT1_I2CBSY) {
		(void)i2c_gd32_wait_not_busy(cfg, I2C_GD32_STOP_TIMEOUT_MS);
		if (I2C_STAT1(cfg->reg) & I2C_STAT1_I2CBSY) {
			(void)i2c_gd32_recover_bus_locked(dev);
		}
		if (I2C_STAT1(cfg->reg) & I2C_STAT1_I2CBSY) {
			data->errs = I2C_GD32_ERR_BUSY;
			return -EBUSY;
		}
	}

#ifdef CONFIG_I2C_GD32_DMA
	if (data->dma_active) {
		i2c_gd32_dma_cleanup(dev);
	}

	data->dma_active = false;
	data->dma_done = false;
	data->dma_tx_done = false;
	data->dma_cookie = 0U;
	if (num_msgs == 1U) {
		bool is_read = (msgs[0].flags & I2C_MSG_READ) != 0U;
		bool use_dma = is_read ? (i2c_gd32_dma_available(&cfg->dma_rx) &&
					 (msgs[0].len >= MAX(CONFIG_I2C_GD32_DMA_MIN_MSG_LEN,
							     2)))
				       : (i2c_gd32_dma_available(&cfg->dma_tx) &&
					  (msgs[0].len >= CONFIG_I2C_GD32_DMA_MIN_MSG_LEN));

		if (use_dma) {
			data->current = &msgs[0];
			data->xfer_len = msgs[0].len;
			err = i2c_gd32_dma_setup(dev, is_read);
			if (err != 0) {
				data->dma_active = false;
			}
		}
	}
#endif

	(void)i2c_gd32_segment_init(dev, 0U, &seg_end);
	ARG_UNUSED(seg_end);

	if (timeout_ms != NULL) {
		*timeout_ms = i2c_gd32_sync_timeout_ms_xfer(dev, msgs, num_msgs);
	}

	i2c_gd32_xfer_begin(dev);

	return 0;
}

static int i2c_gd32_xfer_end(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;

	i2c_gd32_disable_interrupts(cfg);

	/* Wait for stop condition is done. */
	if (i2c_gd32_wait_not_busy(cfg, I2C_GD32_STOP_TIMEOUT_MS) != 0) {
		data->errs |= I2C_GD32_ERR_TIMEOUT;
		(void)i2c_gd32_recover_bus_locked(dev);
	}

#ifdef CONFIG_I2C_GD32_DMA
	i2c_gd32_dma_cleanup(dev);
#endif

	data->phase = I2C_GD32_PHASE_IDLE;

	if (data->errs) {
		if (data->errs & I2C_GD32_ERR_TIMEOUT) {
			return -ETIMEDOUT;
		}
		return -EIO;
	}

	return 0;
}

#ifdef CONFIG_I2C_RTIO
static void i2c_gd32_rtio_work_handler(struct k_work *work)
{
	struct i2c_gd32_data *data = CONTAINER_OF(work, struct i2c_gd32_data, rtio_work);
	const struct device *dev = data->dev;
	struct i2c_rtio *ctx = data->ctx;

	if (data->frontend != I2C_GD32_FRONTEND_RTIO) {
		if (k_sem_take(&data->bus_mutex, K_NO_WAIT) != 0) {
			return;
		}
	}

	for (;;) {
		struct rtio_sqe *sqe;
		bool next_ready = false;
		int err = 0;

		data->frontend = I2C_GD32_FRONTEND_RTIO;

		if (data->rtio_complete_pending) {
			data->rtio_complete_pending = false;
			err = i2c_gd32_xfer_end(dev);
			if (err < 0) {
				i2c_gd32_log_err(data);
			}
			i2c_gd32_transfer_release(dev);

			if (err != 0) {
				next_ready = i2c_rtio_complete(ctx, err);
			} else {
				for (uint8_t i = 0U; i < data->rtio_msg_count; i++) {
					next_ready = i2c_rtio_complete(ctx, 0);
				}
			}

			data->rtio_msg_count = 0U;
			if (next_ready) {
				continue;
			}
			break;
		}

		if ((ctx->txn_head == NULL) || (ctx->txn_curr == NULL)) {
			break;
		}

		sqe = &ctx->txn_curr->sqe;

		switch (sqe->op) {
		case RTIO_OP_RX:
		case RTIO_OP_TX:
		case RTIO_OP_TINY_TX: {
			uint16_t addr;

			err = i2c_gd32_rtio_prepare_msgs(dev, &addr);
			if (err == 0) {
				err = i2c_gd32_transfer_prepare(dev, data->rtio_msgs,
								data->rtio_msg_count, addr, NULL);
			}
			if (err == 0) {
				return;
			}

			i2c_gd32_transfer_release(dev);
			data->rtio_msg_count = 0U;
			next_ready = i2c_rtio_complete(ctx, err);
			if (next_ready) {
				continue;
			}
			break;
		}
		case RTIO_OP_I2C_CONFIGURE:
#ifdef CONFIG_I2C_TARGET
			if (data->target_attached) {
				err = -EBUSY;
			} else
#endif
			{
				err = i2c_gd32_configure_locked(dev, sqe->i2c_config);
			}
			next_ready = i2c_rtio_complete(ctx, err);
			if (next_ready) {
				continue;
			}
			break;
		case RTIO_OP_I2C_RECOVER:
#ifdef CONFIG_I2C_TARGET
			if (data->target_attached) {
				err = -EBUSY;
			} else
#endif
			{
				err = i2c_gd32_recover_bus_locked(dev);
			}
			next_ready = i2c_rtio_complete(ctx, err);
			if (next_ready) {
				continue;
			}
			break;
		case RTIO_OP_AWAIT:
			rtio_iodev_sqe_await_signal(CONTAINER_OF(sqe, struct rtio_iodev_sqe, sqe),
						    i2c_gd32_rtio_sqe_signaled, (void *)dev);
			data->frontend = I2C_GD32_FRONTEND_NONE;
			i2c_gd32_bus_unlock(dev);
			return;
		default:
			next_ready = i2c_rtio_complete(ctx, -EINVAL);
			if (next_ready) {
				continue;
			}
			break;
		}

		break;
	}

	data->frontend = I2C_GD32_FRONTEND_NONE;
	i2c_gd32_bus_unlock(dev);
}

static void i2c_gd32_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct i2c_gd32_data *data = dev->data;

	if (i2c_rtio_submit(data->ctx, iodev_sqe)) {
		i2c_gd32_rtio_kick(dev);
	}
}
#endif

static void i2c_gd32_transfer_release(const struct device *dev)
{
	const struct i2c_gd32_config *cfg = dev->config;

	I2C_CTL0(cfg->reg) &= ~I2C_CTL0_I2CEN;
}

#ifdef CONFIG_I2C_CALLBACK
static void i2c_gd32_async_complete_work_handler(struct k_work *work)
{
	struct i2c_gd32_data *data = CONTAINER_OF(work, struct i2c_gd32_data, async_work);
	const struct device *dev = data->dev;
	i2c_callback_t cb;
	void *userdata;
	int err;
	k_spinlock_key_t key;

	(void)k_work_cancel_delayable(&data->async_timeout_work);

	err = i2c_gd32_xfer_end(dev);
	if (err < 0) {
		i2c_gd32_log_err(data);
	}

	i2c_gd32_transfer_release(dev);

	key = k_spin_lock(&data->lock);
	cb = data->async_cb;
	userdata = data->async_userdata;
	data->async_cb = NULL;
	data->async_userdata = NULL;
	data->async_active = false;
	data->frontend = I2C_GD32_FRONTEND_NONE;
	k_spin_unlock(&data->lock, key);

	i2c_gd32_bus_unlock(dev);

	if (cb != NULL) {
		cb(dev, err, userdata);
	}
}

static void i2c_gd32_async_timeout_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct i2c_gd32_data *data = CONTAINER_OF(dwork, struct i2c_gd32_data, async_timeout_work);
	const struct device *dev = data->dev;
	const struct i2c_gd32_config *cfg = dev->config;
	k_spinlock_key_t key;
	bool claimed = false;

	key = k_spin_lock(&data->lock);
	if (data->async_active && !data->transfer_done) {
		data->errs |= I2C_GD32_ERR_TIMEOUT;
		data->phase = I2C_GD32_PHASE_RECOVERING;
		data->transfer_done = true;
		claimed = true;
	}
	k_spin_unlock(&data->lock, key);

	if (!claimed) {
		return;
	}

	i2c_gd32_disable_interrupts(cfg);
#ifdef CONFIG_I2C_GD32_DMA
	i2c_gd32_dma_cleanup(dev);
#endif
	I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
	(void)k_work_submit(&data->async_work);
}

static int i2c_gd32_transfer_cb(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				uint16_t addr, i2c_callback_t cb, void *userdata)
{
	struct i2c_gd32_data *data = dev->data;
	int err;
	uint32_t timeout_ms;

	if (cb == NULL) {
		return -EINVAL;
	}

	if (k_sem_take(&data->bus_mutex, K_NO_WAIT) != 0) {
		return -EWOULDBLOCK;
	}

#ifdef CONFIG_I2C_TARGET
	if (data->target_attached) {
		i2c_gd32_bus_unlock(dev);
		return -EBUSY;
	}
#endif

	data->async_cb = cb;
	data->async_userdata = userdata;
	data->async_active = true;
	data->frontend = I2C_GD32_FRONTEND_CALLBACK;

	err = i2c_gd32_transfer_prepare(dev, msgs, num_msgs, addr, &timeout_ms);
	if (err != 0) {
		data->async_cb = NULL;
		data->async_userdata = NULL;
		data->async_active = false;
		data->frontend = I2C_GD32_FRONTEND_NONE;
		i2c_gd32_transfer_release(dev);
		i2c_gd32_bus_unlock(dev);
		return err;
	}

	(void)k_work_reschedule(&data->async_timeout_work, K_MSEC(timeout_ms));

	return 0;
}
#endif /* CONFIG_I2C_CALLBACK */

static int i2c_gd32_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			     uint16_t addr)
{
	struct i2c_gd32_data *data = dev->data;
	int err;
	uint32_t timeout_ms;

	k_sem_take(&data->bus_mutex, K_FOREVER);
	data->frontend = I2C_GD32_FRONTEND_SYNC;

#ifdef CONFIG_I2C_TARGET
	/*
	 * Keep controller and target roles mutually exclusive at runtime. This keeps
	 * the interrupt path predictable on the GD32/STM32-v1 style peripheral.
	 */
	if (data->target_attached) {
		data->frontend = I2C_GD32_FRONTEND_NONE;
		i2c_gd32_bus_unlock(dev);
		return -EBUSY;
	}
#endif

	err = i2c_gd32_transfer_prepare(dev, msgs, num_msgs, addr, &timeout_ms);
	if (err != 0) {
		i2c_gd32_log_err(data);
		goto out;
	}

	if (k_sem_take(&data->sync_sem, K_MSEC(timeout_ms)) != 0) {
		data->errs |= I2C_GD32_ERR_TIMEOUT;
		(void)i2c_gd32_recover_bus_locked(dev);
	}

	err = i2c_gd32_xfer_end(dev);
	if (err < 0) {
		i2c_gd32_log_err(data);
	}

out:
	data->frontend = I2C_GD32_FRONTEND_NONE;
	i2c_gd32_transfer_release(dev);
	i2c_gd32_bus_unlock(dev);

	return err;
}

static int i2c_gd32_configure_locked(const struct device *dev, uint32_t dev_config)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t pclk1, freq, clkc;
	int ret;
	int err = 0;

	/* Disable I2C device */
	I2C_CTL0(cfg->reg) &= ~I2C_CTL0_I2CEN;

	ret = clock_control_get_rate(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid,
				     &pclk1);
	if (ret != 0) {
		err = ret;
		goto error;
	}

	/* i2c clock frequency, us */
	freq = pclk1 / 1000000U;
	if (freq > I2CCLK_MAX) {
		LOG_WRN("I2C max I2CCLK %u, current is %u", I2CCLK_MAX, freq);
		freq = I2CCLK_MAX;
	}

	/*
	 * Refer from SoC user manual.
	 * In standard mode:
	 *   T_high = CLKC * T_pclk1
	 *   T_low  = CLKC * T_pclk1
	 *
	 * In fast mode and fast mode plus with DTCY=1:
	 *   T_high = 9 * CLKC * T_pclk1
	 *   T_low  = 16 * CLKC * T_pclk1
	 *
	 * T_pclk1 is reciprocal of pclk1:
	 *   T_pclk1 = 1 / pclk1
	 *
	 * T_high and T_low construct the bit transfer:
	 *  T_high + T_low = 1 / bitrate
	 *
	 * And then, we can get the CLKC equation.
	 * Standard mode:
	 *   CLKC = pclk1 / (bitrate * 2)
	 * Fast mode and fast mode plus:
	 *   CLKC = pclk1 / (bitrate * 25)
	 *
	 * Variable list:
	 *   T_high:  high period of the SCL clock
	 *   T_low:   low period of the SCL clock
	 *   T_pclk1: duration of single pclk1 pulse
	 *   pclk1:   i2c device clock frequency
	 *   bitrate: 100 Kbits for standard mode
	 */
	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		if (freq < I2CCLK_MIN) {
			LOG_ERR("I2C standard-mode min clock freq %u, current is %u\n", I2CCLK_MIN,
				freq);
			err = -ENOTSUP;
			goto error;
		}
		I2C_CTL1(cfg->reg) &= ~I2C_CTL1_I2CCLK;
		I2C_CTL1(cfg->reg) |= freq;

		/* Standard-mode risetime maximum value: 1000ns */
		if (freq == I2CCLK_MAX) {
			I2C_RT(cfg->reg) = I2CCLK_MAX;
		} else {
			I2C_RT(cfg->reg) = freq + 1U;
		}

		/* CLKC = pclk1 / (bitrate * 2) */
		clkc = DIV_ROUND_UP(pclk1, I2C_BITRATE_STANDARD * 2U);
		if (clkc < 4U) {
			clkc = 4U;
		}
		if (clkc > I2C_CKCFG_CLKC) {
			LOG_ERR("I2C standard-mode CLKC overflow: %u", clkc);
			err = -ENOTSUP;
			goto error;
		}

		I2C_CKCFG(cfg->reg) &= ~I2C_CKCFG_CLKC;
		I2C_CKCFG(cfg->reg) |= clkc;
		/* standard-mode */
		I2C_CKCFG(cfg->reg) &= ~I2C_CKCFG_FAST;

		break;
	case I2C_SPEED_FAST:
		if (freq < I2CCLK_FM_MIN) {
			LOG_ERR("I2C fast-mode min clock freq %u, current is %u\n", I2CCLK_FM_MIN,
				freq);
			err = -ENOTSUP;
			goto error;
		}

		I2C_CTL1(cfg->reg) &= ~I2C_CTL1_I2CCLK;
		I2C_CTL1(cfg->reg) |= freq;

		/* Fast-mode risetime maximum value: 300ns */
		I2C_RT(cfg->reg) = freq * 300U / 1000U + 1U;

		{
			uint32_t clkc_dtcy2 = DIV_ROUND_UP(pclk1, I2C_BITRATE_FAST * 3U);
			uint32_t clkc_dtcy169 = DIV_ROUND_UP(pclk1, I2C_BITRATE_FAST * 25U);
			uint32_t bitrate_dtcy2 = pclk1 / (3U * MAX(clkc_dtcy2, 1U));
			uint32_t bitrate_dtcy169 = pclk1 / (25U * MAX(clkc_dtcy169, 1U));

			if (clkc_dtcy2 == 0U) {
				clkc_dtcy2 = 1U;
			}
			if (clkc_dtcy169 == 0U) {
				clkc_dtcy169 = 1U;
			}
			if ((clkc_dtcy2 > I2C_CKCFG_CLKC) || (clkc_dtcy169 > I2C_CKCFG_CLKC)) {
				LOG_ERR("I2C fast-mode CLKC overflow");
				err = -ENOTSUP;
				goto error;
			}

			I2C_CKCFG(cfg->reg) &= ~(I2C_CKCFG_DTCY | I2C_CKCFG_CLKC);
			if (bitrate_dtcy169 >= bitrate_dtcy2) {
				I2C_CKCFG(cfg->reg) |= I2C_CKCFG_DTCY;
				clkc = clkc_dtcy169;
			} else {
				clkc = clkc_dtcy2;
			}

			I2C_CKCFG(cfg->reg) |= clkc;
		}
		/* Transfer mode: fast-mode */
		I2C_CKCFG(cfg->reg) |= I2C_CKCFG_FAST;

#ifdef I2C_FMPCFG
		/* Disable transfer mode: fast-mode plus */
		I2C_FMPCFG(cfg->reg) &= ~I2C_FMPCFG_FMPEN;
#endif /* I2C_FMPCFG */

		break;
#ifdef I2C_FMPCFG
	case I2C_SPEED_FAST_PLUS:
		if (freq < I2CCLK_FM_PLUS_MIN) {
			LOG_ERR("I2C fast-mode plus min clock freq %u, current is %u\n",
				I2CCLK_FM_PLUS_MIN, freq);
			err = -ENOTSUP;
			goto error;
		}

		I2C_CTL1(cfg->reg) &= ~I2C_CTL1_I2CCLK;
		I2C_CTL1(cfg->reg) |= freq;

		/* Fast-mode plus risetime maximum value: 120ns */
		I2C_RT(cfg->reg) = freq * 120U / 1000U + 1U;

		{
			uint32_t clkc_dtcy2 = DIV_ROUND_UP(pclk1, I2C_BITRATE_FAST_PLUS * 3U);
			uint32_t clkc_dtcy169 = DIV_ROUND_UP(pclk1, I2C_BITRATE_FAST_PLUS * 25U);
			uint32_t bitrate_dtcy2 = pclk1 / (3U * MAX(clkc_dtcy2, 1U));
			uint32_t bitrate_dtcy169 = pclk1 / (25U * MAX(clkc_dtcy169, 1U));

			if (clkc_dtcy2 == 0U) {
				clkc_dtcy2 = 1U;
			}
			if (clkc_dtcy169 == 0U) {
				clkc_dtcy169 = 1U;
			}
			if ((clkc_dtcy2 > I2C_CKCFG_CLKC) || (clkc_dtcy169 > I2C_CKCFG_CLKC)) {
				LOG_ERR("I2C fast-mode plus CLKC overflow");
				err = -ENOTSUP;
				goto error;
			}

			I2C_CKCFG(cfg->reg) &= ~(I2C_CKCFG_DTCY | I2C_CKCFG_CLKC);
			if (bitrate_dtcy169 >= bitrate_dtcy2) {
				I2C_CKCFG(cfg->reg) |= I2C_CKCFG_DTCY;
				clkc = clkc_dtcy169;
			} else {
				clkc = clkc_dtcy2;
			}

			I2C_CKCFG(cfg->reg) |= clkc;
		}
		/* Transfer mode: fast-mode */
		I2C_CKCFG(cfg->reg) |= I2C_CKCFG_FAST;

		/* Enable transfer mode: fast-mode plus */
		I2C_FMPCFG(cfg->reg) |= I2C_FMPCFG_FMPEN;

		break;
#endif /* I2C_FMPCFG */
	case I2C_SPEED_HIGH:
	case I2C_SPEED_ULTRA:
		err = -ENOTSUP;
		goto error;
	default:
		/* Use standard-mode timing with arbitrary DT-provided bitrate. */
		uint32_t bitrate = cfg->bitrate ? cfg->bitrate : I2C_BITRATE_STANDARD;

		if (freq < I2CCLK_MIN) {
			LOG_ERR("I2C standard-mode min clock freq %u, current is %u\n",
				I2CCLK_MIN, freq);
			err = -ENOTSUP;
			goto error;
		}

		I2C_CTL1(cfg->reg) &= ~I2C_CTL1_I2CCLK;
		I2C_CTL1(cfg->reg) |= freq;

		/* Standard-mode risetime maximum value: 1000ns */
		if (freq == I2CCLK_MAX) {
			I2C_RT(cfg->reg) = I2CCLK_MAX;
		} else {
			I2C_RT(cfg->reg) = freq + 1U;
		}

		/* CLKC = pclk1 / (bitrate * 2) */
		clkc = DIV_ROUND_UP(pclk1, bitrate * 2U);
		if (clkc < 4U) {
			clkc = 4U;
		}
		if (clkc > I2C_CKCFG_CLKC) {
			LOG_ERR("I2C standard-mode CLKC overflow: %u", clkc);
			err = -ENOTSUP;
			goto error;
		}

		I2C_CKCFG(cfg->reg) &= ~I2C_CKCFG_CLKC;
		I2C_CKCFG(cfg->reg) |= clkc;
		/* Force standard-mode */
		I2C_CKCFG(cfg->reg) &= ~I2C_CKCFG_FAST;
		break;
	}

	data->dev_config = dev_config;

error:
	return err;
}

static int i2c_gd32_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_gd32_data *data = dev->data;
	int err = 0;

	k_sem_take(&data->bus_mutex, K_FOREVER);

#ifdef CONFIG_I2C_TARGET
	if (data->target_attached) {
		err = -EBUSY;
		i2c_gd32_bus_unlock(dev);
		return err;
	}
#endif

	err = i2c_gd32_configure_locked(dev, dev_config);

	i2c_gd32_bus_unlock(dev);

	return err;
}

static int i2c_gd32_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_gd32_data *data = dev->data;

	if ((dev_config == NULL) || (data->dev_config == 0U)) {
		return -EIO;
	}

	*dev_config = data->dev_config;

	return 0;
}

static DEVICE_API(i2c, i2c_gd32_driver_api) = {
	.configure = i2c_gd32_configure,
	.get_config = i2c_gd32_get_config,
	.transfer = i2c_gd32_transfer,
#ifdef CONFIG_I2C_CALLBACK
	.transfer_cb = i2c_gd32_transfer_cb,
#endif
#ifdef CONFIG_I2C_TARGET
	.target_register = i2c_gd32_target_register,
	.target_unregister = i2c_gd32_target_unregister,
#endif
	.recover_bus = i2c_gd32_recover_bus,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_gd32_submit,
#endif
};

static int i2c_gd32_init(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t bitrate_cfg;
	int err;

	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	/* Mutex semaphore to protect the i2c api in multi-thread env. */
	data->dev = dev;
	k_sem_init(&data->bus_mutex, 1, 1);

	/* Sync semaphore to sync i2c state between isr and transfer api. */
	k_sem_init(&data->sync_sem, 0, K_SEM_MAX_LIMIT);
#ifdef CONFIG_I2C_CALLBACK
	k_work_init(&data->async_work, i2c_gd32_async_complete_work_handler);
	k_work_init_delayable(&data->async_timeout_work, i2c_gd32_async_timeout_work_handler);
#endif
#ifdef CONFIG_I2C_RTIO
	k_work_init(&data->rtio_work, i2c_gd32_rtio_work_handler);
	i2c_rtio_init(data->ctx, dev);
#endif
	data->phase = I2C_GD32_PHASE_IDLE;
	data->frontend = I2C_GD32_FRONTEND_NONE;
	data->transfer_done = true;
#ifdef CONFIG_I2C_TARGET
	data->target_state = I2C_GD32_TARGET_IDLE;
#endif

	(void)clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

	(void)reset_line_toggle_dt(&cfg->reset);

	cfg->irq_cfg_func();

#ifdef CONFIG_I2C_GD32_DMA
	if ((cfg->dma_rx.dev != NULL) && !device_is_ready(cfg->dma_rx.dev)) {
		LOG_ERR("RX DMA device not ready");
		return -ENODEV;
	}
	if ((cfg->dma_tx.dev != NULL) && !device_is_ready(cfg->dma_tx.dev)) {
		LOG_ERR("TX DMA device not ready");
		return -ENODEV;
	}
#endif

	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate);

	return i2c_gd32_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
}

#ifdef CONFIG_DEVICE_DEINIT_SUPPORT
static int i2c_gd32_deinit(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	int ret;

	k_sem_take(&data->bus_mutex, K_FOREVER);

	if (!data->transfer_done || (data->phase != I2C_GD32_PHASE_IDLE)) {
		ret = -EBUSY;
		goto out;
	}

#ifdef CONFIG_I2C_CALLBACK
	if (data->async_active) {
		ret = -EBUSY;
		goto out;
	}

	(void)k_work_cancel(&data->async_work);
	(void)k_work_cancel_delayable(&data->async_timeout_work);
#endif

#ifdef CONFIG_I2C_RTIO
	if (data->rtio_complete_pending || i2c_gd32_rtio_pending(data)) {
		ret = -EBUSY;
		goto out;
	}

	(void)k_work_cancel(&data->rtio_work);
#endif

#ifdef CONFIG_I2C_TARGET
	if (data->target_attached) {
		ret = -EBUSY;
		goto out;
	}
#endif

	i2c_gd32_quiesce_controller(dev);
#ifdef CONFIG_I2C_GD32_DMA
	i2c_gd32_dma_cleanup(dev);
#endif
	I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
	(void)i2c_gd32_wait_not_busy(cfg, I2C_GD32_STOP_TIMEOUT_MS);
	I2C_CTL0(cfg->reg) &= ~(I2C_CTL0_I2CEN | I2C_CTL0_ACKEN | I2C_CTL0_POAP);
	I2C_CTL1(cfg->reg) &= ~(I2C_CTL1_ERRIE | I2C_CTL1_EVIE | I2C_CTL1_BUFIE |
				I2C_CTL1_DMAON | I2C_CTL1_DMALST);
	(void)reset_line_toggle_dt(&cfg->reset);
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
	if (ret == -ENOENT) {
		ret = 0;
	}
	if (ret != 0) {
		goto out;
	}

	(void)clock_control_off(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

	data->dev_config = 0U;
	data->msgs = NULL;
	data->current = NULL;
	data->num_msgs = 0U;
	data->seg_next = 0U;
	data->xfer_len = 0U;
	data->errs = 0U;
	data->phase = I2C_GD32_PHASE_IDLE;
	data->frontend = I2C_GD32_FRONTEND_NONE;
	data->transfer_done = true;
	data->addr_is_read = false;
	data->ten_bit_read_restart = false;
#ifdef CONFIG_I2C_RTIO
	data->rtio_msg_count = 0U;
	data->rtio_complete_pending = false;
#endif
#ifdef CONFIG_I2C_GD32_DMA
	data->dma_active = false;
	data->dma_is_read = false;
	data->dma_done = false;
	data->dma_tx_done = false;
	data->dma_dev = NULL;
	data->dma_channel = 0U;
#endif

out:
	k_sem_give(&data->bus_mutex);

	return ret;
}
#endif

#ifdef CONFIG_I2C_GD32_DMA
#define DMA_INITIALIZER(idx, dir)                                                                  \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, dir)),                         \
		.channel = DT_INST_DMAS_CELL_BY_NAME(idx, dir, channel),                           \
		.slot = COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1),	     \
				    (DT_INST_DMAS_CELL_BY_NAME(idx, dir, slot)), (0)),                            \
			 .config = DT_INST_DMAS_CELL_BY_NAME(idx, dir, config),                    \
			 .fifo_threshold = COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1), \
					      (DT_INST_DMAS_CELL_BY_NAME(idx, dir, fifo_threshold)), \
					      (0)),         \
			 }

#define DMA_BY_NAME(idx, dir)                                                                      \
	COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, dir), (DMA_INITIALIZER(idx, dir)), ({0}))
#else
#define DMA_BY_NAME(idx, dir) {0}
#endif /* CONFIG_I2C_GD32_DMA */

#ifdef CONFIG_DEVICE_DEINIT_SUPPORT
#define I2C_GD32_DEVICE_DEFINE(inst)                                                           \
	I2C_DEVICE_DT_INST_DEINIT_DEFINE(inst, i2c_gd32_init, i2c_gd32_deinit, NULL,         \
					 &i2c_gd32_data_##inst, &i2c_gd32_cfg_##inst,          \
					 POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,                \
					 &i2c_gd32_driver_api)
#else
#define I2C_GD32_DEVICE_DEFINE(inst)                                                           \
	I2C_DEVICE_DT_INST_DEFINE(inst, i2c_gd32_init, NULL, &i2c_gd32_data_##inst,          \
				  &i2c_gd32_cfg_##inst, POST_KERNEL,                        \
				  CONFIG_I2C_INIT_PRIORITY, &i2c_gd32_driver_api)
#endif

#define I2C_GD32_INIT(inst)                                                                        \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static void i2c_gd32_irq_cfg_func_##inst(void)                                             \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, event, irq),                                 \
			    DT_INST_IRQ_BY_NAME(inst, event, priority), i2c_gd32_event_isr,        \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, event, irq));                                 \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, error, irq),                                 \
			    DT_INST_IRQ_BY_NAME(inst, error, priority), i2c_gd32_error_isr,        \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
				irq_enable(DT_INST_IRQ_BY_NAME(inst, error, irq));                                 \
		}                                                                                          \
	IF_ENABLED(CONFIG_I2C_RTIO,								       \
		(I2C_RTIO_DEFINE(i2c_gd32_rtio_##inst,					       \
				 DT_INST_PROP_OR(inst, sq_size, CONFIG_I2C_RTIO_SQ_SIZE),      \
				 DT_INST_PROP_OR(inst, cq_size, CONFIG_I2C_RTIO_CQ_SIZE));       \
		 static struct i2c_msg i2c_gd32_rtio_msgs_##inst[				       \
			 DT_INST_PROP_OR(inst, sq_size, CONFIG_I2C_RTIO_SQ_SIZE)];))       \
	static struct i2c_gd32_data i2c_gd32_data_##inst = {                                       \
		IF_ENABLED(CONFIG_I2C_RTIO,							       \
			(.ctx = &i2c_gd32_rtio_##inst,					       \
			 .rtio_msgs = i2c_gd32_rtio_msgs_##inst,			       \
			 .rtio_msg_capacity = ARRAY_SIZE(i2c_gd32_rtio_msgs_##inst),))     \
	};                                                                                         \
	static const struct i2c_gd32_config i2c_gd32_cfg_##inst = {                                \
		.reg = DT_INST_REG_ADDR(inst),                                                     \
		.bitrate = DT_INST_PROP(inst, clock_frequency),                                    \
		.clkid = DT_INST_CLOCKS_CELL(inst, id),                                            \
		.reset = RESET_DT_SPEC_INST_GET(inst),                                             \
		.scl_gpios = GPIO_DT_SPEC_INST_GET_OR(inst, scl_gpios, {0}),                       \
		.sda_gpios = GPIO_DT_SPEC_INST_GET_OR(inst, sda_gpios, {0}),                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.irq_cfg_func = i2c_gd32_irq_cfg_func_##inst,                                      \
		IF_ENABLED(CONFIG_I2C_GD32_DMA,					\
			   (.dma_rx = DMA_BY_NAME(inst, rx),			\
			    .dma_tx = DMA_BY_NAME(inst, tx),)) };        \
	I2C_GD32_DEVICE_DEFINE(inst);

DT_INST_FOREACH_STATUS_OKAY(I2C_GD32_INIT)
