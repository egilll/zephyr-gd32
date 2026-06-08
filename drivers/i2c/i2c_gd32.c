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

struct i2c_gd32_data {
	struct k_sem bus_mutex;
	struct k_sem sync_sem;
	uint32_t dev_config;
	uint16_t addr1;
	uint16_t addr2;
	uint32_t xfer_len;
	struct i2c_msg *msgs;
	uint8_t num_msgs;
	uint8_t seg_next;
	struct i2c_msg *current;
	uint16_t errs;
	bool is_restart;
	bool addr_is_read;
	bool waiting_for_start;
	bool sync_done;
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

static inline void i2c_gd32_enable_interrupts(const struct i2c_gd32_config *cfg);
#ifdef CONFIG_I2C_GD32_DMA
static inline void i2c_gd32_enable_interrupts_dma(const struct i2c_gd32_config *cfg);
#endif

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

	if (data->dev_config & I2C_ADDR_10_BITS) {
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

static void i2c_gd32_segment_configure(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	bool is_read = (data->current->flags & I2C_MSG_READ) != 0U;

	I2C_CTL1(cfg->reg) &= ~(I2C_CTL1_DMAON | I2C_CTL1_DMALST);
	I2C_CTL0(cfg->reg) &= ~I2C_CTL0_POAP;

	I2C_CTL0(cfg->reg) |= I2C_CTL0_ACKEN;

	data->addr_is_read = is_read;
	data->is_restart = false;
	data->waiting_for_start = false;

	if (is_read) {
#ifdef CONFIG_I2C_GD32_DMA
		if (!data->dma_active && (data->xfer_len == 2U)) {
#else
		if (data->xfer_len == 2U) {
#endif
			I2C_CTL0(cfg->reg) |= I2C_CTL0_POAP;
		}

		if (data->dev_config & I2C_ADDR_10_BITS) {
			/*
			 * 10-bit master receive requires an address phase with W, then a repeated START
			 * and an address phase with R (GD32 RM 20.3.9).
			 */
			data->is_restart = true;
			data->addr_is_read = false;
		}
	}

#ifdef CONFIG_I2C_GD32_DMA
	if (data->dma_active) {
		i2c_gd32_enable_interrupts_dma(cfg);
	} else {
		i2c_gd32_enable_interrupts(cfg);
	}
#else
	i2c_gd32_enable_interrupts(cfg);
#endif
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
	i2c_gd32_segment_configure(dev);

	return 0;
}

static void i2c_gd32_segment_complete(const struct device *dev, enum i2c_gd32_seg_end seg_end)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;

	if (data->sync_done) {
		return;
	}

	if (data->errs != 0U) {
		data->sync_done = true;
		k_sem_give(&data->sync_sem);
		return;
	}

	if (seg_end == I2C_GD32_SEG_END_STOP) {
		I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
		data->sync_done = true;
		k_sem_give(&data->sync_sem);
		return;
	}

	if (data->waiting_for_start) {
		return;
	}

	if (data->seg_next >= data->num_msgs) {
		data->errs |= I2C_GD32_ERR_TIMEOUT;
		I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
		data->sync_done = true;
		k_sem_give(&data->sync_sem);
		return;
	}

	enum i2c_gd32_seg_end next_end;
	(void)i2c_gd32_segment_init(dev, data->seg_next, &next_end);
	data->waiting_for_start = true;
	I2C_CTL0(cfg->reg) |= I2C_CTL0_START;
}

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

static inline void i2c_gd32_enable_interrupts(const struct i2c_gd32_config *cfg)
{
	I2C_CTL1(cfg->reg) |= I2C_CTL1_ERRIE;
	I2C_CTL1(cfg->reg) |= I2C_CTL1_EVIE;
	I2C_CTL1(cfg->reg) |= I2C_CTL1_BUFIE;
}

#ifdef CONFIG_I2C_GD32_DMA
static inline void i2c_gd32_enable_interrupts_dma(const struct i2c_gd32_config *cfg)
{
	I2C_CTL1(cfg->reg) |= I2C_CTL1_ERRIE;
	I2C_CTL1(cfg->reg) |= I2C_CTL1_EVIE;
	I2C_CTL1(cfg->reg) &= ~I2C_CTL1_BUFIE;
}
#endif

static inline void i2c_gd32_disable_interrupts(const struct i2c_gd32_config *cfg)
{
	I2C_CTL1(cfg->reg) &= ~I2C_CTL1_ERRIE;
	I2C_CTL1(cfg->reg) &= ~I2C_CTL1_EVIE;
	I2C_CTL1(cfg->reg) &= ~I2C_CTL1_BUFIE;
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
	enum i2c_gd32_seg_end seg_end;

	switch (data->xfer_len) {
	case 0:
		/* Unwanted data received, ignore it. */
		seg_end = (data->seg_next >= data->num_msgs) ? I2C_GD32_SEG_END_STOP
							     : I2C_GD32_SEG_END_RESTART;
		i2c_gd32_segment_complete(dev, seg_end);
		break;
	case 1:
		/* If total_read_length == 1, read the data directly. */
		data->xfer_len--;
		i2c_gd32_xfer_read(data, cfg);
		seg_end = (data->seg_next >= data->num_msgs) ? I2C_GD32_SEG_END_STOP
							     : I2C_GD32_SEG_END_RESTART;
		i2c_gd32_segment_complete(dev, seg_end);

		break;
	case 2:
		__fallthrough;
	case 3:
		/*
		 * If total_read_length == 2, or total_read_length > 3
		 * and remaining_read_length == 3, disable the RBNE
		 * interrupt.
		 * Remaining data will be read from BTC interrupt.
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
	enum i2c_gd32_seg_end seg_end;

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
		seg_end = (data->seg_next >= data->num_msgs) ? I2C_GD32_SEG_END_STOP
							     : I2C_GD32_SEG_END_RESTART;
		i2c_gd32_segment_complete(dev, seg_end);
	}
}

static void i2c_gd32_handle_btc(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	enum i2c_gd32_seg_end seg_end = (data->seg_next >= data->num_msgs) ? I2C_GD32_SEG_END_STOP
									   : I2C_GD32_SEG_END_RESTART;

	if (data->current->flags & I2C_MSG_READ) {
		uint32_t counter = 0U;

		switch (data->xfer_len) {
		case 2:
			if (seg_end == I2C_GD32_SEG_END_STOP) {
				/* Stop condition must be generated before reading the last two bytes. */
				I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
			}

			for (counter = 2U; counter > 0; counter--) {
				data->xfer_len--;
				i2c_gd32_xfer_read(data, cfg);
			}
			i2c_gd32_segment_complete(dev, seg_end);

			break;
		case 3:
			/* Clear ACKEN bit */
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
	enum i2c_gd32_seg_end seg_end = (data->seg_next >= data->num_msgs) ? I2C_GD32_SEG_END_STOP
									   : I2C_GD32_SEG_END_RESTART;

	if ((data->current->flags & I2C_MSG_READ) && (data->xfer_len <= 2U)) {
		I2C_CTL0(cfg->reg) &= ~I2C_CTL0_ACKEN;
	}

	/* Clear ADDSEND bit */
	I2C_STAT0(cfg->reg);
	I2C_STAT1(cfg->reg);

	if (data->is_restart) {
		data->is_restart = false;
		data->addr_is_read = true;
		data->waiting_for_start = true;
		/* Enter repeated start condition */
		I2C_CTL0(cfg->reg) |= I2C_CTL0_START;

		return;
	}

	if ((seg_end == I2C_GD32_SEG_END_STOP) && (data->current->flags & I2C_MSG_READ) &&
	    (data->xfer_len == 1U)) {
		/* Enter stop condition */
		I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
	}
}

static void i2c_gd32_event_isr(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t stat;

	if (data->sync_done) {
		return;
	}

	stat = I2C_STAT0(cfg->reg);

	if (data->waiting_for_start && ((stat & I2C_STAT0_SBSEND) == 0U)) {
		return;
	}

	if (stat & I2C_STAT0_SBSEND) {
		data->waiting_for_start = false;
		if (data->addr_is_read) {
			I2C_DATA(cfg->reg) = (data->addr1 << 1U) | 1U;
		} else {
			I2C_DATA(cfg->reg) = (data->addr1 << 1U) | 0U;
		}
	} else if (stat & I2C_STAT0_ADD10SEND) {
		I2C_DATA(cfg->reg) = data->addr2;
	} else if (stat & I2C_STAT0_ADDSEND) {
#ifdef CONFIG_I2C_GD32_DMA
			if (data->dma_active) {
			/*
			 * DMAON must be set after clearing ADDSEND (GD32 RM 20.3.9).
			 * The read data phase for 10-bit addressing starts after the
			 * repeated START, so only enable DMA when is_restart is false.
			 */
			if (data->dma_is_read && (data->xfer_len == 1U)) {
				I2C_CTL0(cfg->reg) &= ~I2C_CTL0_ACKEN;
			}

			/* Clear ADDSEND bit */
			I2C_STAT0(cfg->reg);
			I2C_STAT1(cfg->reg);

				if (data->is_restart) {
					data->is_restart = false;
					data->addr_is_read = true;
					data->waiting_for_start = true;
					/* Enter repeated start condition */
					I2C_CTL0(cfg->reg) |= I2C_CTL0_START;
					return;
				}

			if (data->dma_is_read && (data->xfer_len >= 2U)) {
				I2C_CTL1(cfg->reg) |= I2C_CTL1_DMALST;
			} else {
				I2C_CTL1(cfg->reg) &= ~I2C_CTL1_DMALST;
			}

				enum i2c_gd32_seg_end seg_end = (data->seg_next >= data->num_msgs)
									? I2C_GD32_SEG_END_STOP
									: I2C_GD32_SEG_END_RESTART;
				if ((seg_end == I2C_GD32_SEG_END_STOP) && data->dma_is_read &&
				    (data->xfer_len == 1U)) {
					/* Enter stop condition */
					I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
				}

				I2C_CTL1(cfg->reg) |= I2C_CTL1_DMAON;
		} else
#endif
		{
			i2c_gd32_handle_addsend(dev);
		}
		/*
		 * Must handle BTC first.
		 * For I2C_STAT0, BTC is the superset of RBNE and TBE.
		 */
	} else if (stat & I2C_STAT0_BTC) {
#ifdef CONFIG_I2C_GD32_DMA
		if (data->dma_active) {
			if (!data->dma_is_read && data->dma_tx_done) {
				/* Enter stop condition */
				I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
				k_sem_give(&data->sync_sem);
			}
		} else
#endif
		{
			i2c_gd32_handle_btc(dev);
		}
	} else if (stat & I2C_STAT0_RBNE) {
#ifdef CONFIG_I2C_GD32_DMA
		if (data->dma_active) {
			return;
		}
#endif
		i2c_gd32_handle_rbne(dev);
	} else if (stat & I2C_STAT0_TBE) {
#ifdef CONFIG_I2C_GD32_DMA
		if (data->dma_active) {
			return;
		}
#endif
		i2c_gd32_handle_tbe(dev);
	}
}

static void i2c_gd32_error_isr(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t stat;

	stat = I2C_STAT0(cfg->reg);

	if (stat & I2C_STAT0_BERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_BERR;
		data->errs |= I2C_GD32_ERR_BERR;
	}

	if (stat & I2C_STAT0_LOSTARB) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_LOSTARB;
		data->errs |= I2C_GD32_ERR_LARB;
	}

	if (stat & I2C_STAT0_AERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_AERR;
		data->errs |= I2C_GD32_ERR_AERR;
	}

	if (stat & I2C_STAT0_OUERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_OUERR;
		data->errs |= I2C_GD32_ERR_OUERR;
	}

	if (stat & I2C_STAT0_PECERR) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_PECERR;
		data->errs |= I2C_GD32_ERR_PECERR;
	}

	if (stat & I2C_STAT0_SMBTO) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_SMBTO;
		data->errs |= I2C_GD32_ERR_SMBTO;
	}

	if (stat & I2C_STAT0_SMBALT) {
		I2C_STAT0(cfg->reg) &= ~I2C_STAT0_SMBALT;
		data->errs |= I2C_GD32_ERR_SMBALT;
	}

	if (data->errs != 0U) {
		/* Enter stop condition */
		I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;

#ifdef CONFIG_I2C_GD32_DMA
		if (data->dma_active) {
			(void)dma_stop(data->dma_dev, data->dma_channel);
			I2C_CTL1(cfg->reg) &= ~(I2C_CTL1_DMAON | I2C_CTL1_DMALST);
			data->dma_active = false;
		}
#endif

		data->sync_done = true;
		k_sem_give(&data->sync_sem);
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

	if (!data->dma_active || (dma_dev != data->dma_dev) || (channel != data->dma_channel)) {
		return;
	}

	if (status < 0) {
		data->errs |= I2C_GD32_ERR_DMA;
	}

	(void)dma_stop(dma_dev, channel);

	data->dma_done = true;

	if (data->dma_is_read) {
		I2C_CTL1(cfg->reg) &= ~(I2C_CTL1_DMAON | I2C_CTL1_DMALST);
		/* For N>=2 the STOP is generated on DMA completion (GD32 RM 20.3.9). */
		if (data->xfer_len >= 2U) {
			I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
		}
		k_sem_give(&data->sync_sem);
	} else {
		I2C_CTL1(cfg->reg) &= ~I2C_CTL1_DMAON;
		data->dma_tx_done = true;
		if (I2C_STAT0(cfg->reg) & I2C_STAT0_BTC) {
			I2C_CTL0(cfg->reg) |= I2C_CTL0_STOP;
			k_sem_give(&data->sync_sem);
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
	data->dma_blk.fifo_mode_control = dma->fifo_threshold;

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
}
#endif /* CONFIG_I2C_GD32_DMA */

static int i2c_gd32_recover_bus_locked(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t ctl1 = I2C_CTL1(cfg->reg);
	uint32_t ckcfg = I2C_CKCFG(cfg->reg);
	uint32_t rt = I2C_RT(cfg->reg);
	uint32_t fctl = I2C_FCTL(cfg->reg);
#ifdef I2C_FMPCFG
	uint32_t fmpcfg = I2C_FMPCFG(cfg->reg);
#endif
	int ret;

	i2c_gd32_disable_interrupts(cfg);

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

	return ret;
}

static int i2c_gd32_recover_bus(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	int ret;

	k_sem_take(&data->bus_mutex, K_FOREVER);
	ret = i2c_gd32_recover_bus_locked(dev);
	k_sem_give(&data->bus_mutex);

	return ret;
}

static void i2c_gd32_xfer_begin(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;

	k_sem_reset(&data->sync_sem);

	data->errs = 0U;
	data->sync_done = false;
	data->waiting_for_start = true;
	i2c_gd32_segment_configure(dev);

	/* Enter repeated start condition */
	I2C_CTL0(cfg->reg) |= I2C_CTL0_START;
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

	if (data->errs) {
		if (data->errs & I2C_GD32_ERR_TIMEOUT) {
			return -ETIMEDOUT;
		}
		return -EIO;
	}

	return 0;
}

static int i2c_gd32_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			     uint16_t addr)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	enum i2c_gd32_seg_end seg_end;
	int err = 0;
	uint32_t timeout_ms;

	for (uint8_t i = 0U; i < num_msgs; i++) {
		if ((msgs[i].buf == NULL) || (msgs[i].len == 0U)) {
			return -EINVAL;
		}
		if (i < (num_msgs - 1U)) {
			if ((msgs[i].flags & I2C_MSG_STOP) != 0U) {
				return -EINVAL;
			}
			if (((msgs[i].flags & I2C_MSG_RW_MASK) != (msgs[i + 1U].flags & I2C_MSG_RW_MASK)) &&
			    ((msgs[i + 1U].flags & I2C_MSG_RESTART) == 0U)) {
				return -EINVAL;
			}
		}
	}

	k_sem_take(&data->bus_mutex, K_FOREVER);

	/* Enable i2c device */
	I2C_CTL0(cfg->reg) |= I2C_CTL0_I2CEN;

	if (data->dev_config & I2C_ADDR_10_BITS) {
		data->addr1 = 0xF0 | ((addr & BITS(8, 9)) >> 8U);
		data->addr2 = addr & BITS(0, 7);
	} else {
		data->addr1 = addr & BITS(0, 6);
	}

	data->msgs = msgs;
	data->num_msgs = num_msgs;

	if (I2C_STAT1(cfg->reg) & I2C_STAT1_I2CBSY) {
		(void)i2c_gd32_wait_not_busy(cfg, I2C_GD32_STOP_TIMEOUT_MS);
		if (I2C_STAT1(cfg->reg) & I2C_STAT1_I2CBSY) {
			(void)i2c_gd32_recover_bus_locked(dev);
		}
		if (I2C_STAT1(cfg->reg) & I2C_STAT1_I2CBSY) {
			data->errs = I2C_GD32_ERR_BUSY;
			err = -EBUSY;
			i2c_gd32_log_err(data);
			goto out;
		}
	}

#ifdef CONFIG_I2C_GD32_DMA
	data->dma_active = false;
	if (num_msgs == 1U) {
		bool is_read = (msgs[0].flags & I2C_MSG_READ) != 0U;
		bool use_dma = is_read ? i2c_gd32_dma_available(&cfg->dma_rx)
				       : i2c_gd32_dma_available(&cfg->dma_tx);
		if (use_dma) {
			data->current = &msgs[0];
			data->xfer_len = msgs[0].len;
			(void)i2c_gd32_dma_setup(dev, is_read);
		}
	}
#endif

	(void)i2c_gd32_segment_init(dev, 0U, &seg_end);
	timeout_ms = i2c_gd32_sync_timeout_ms_xfer(dev, msgs, num_msgs);
	i2c_gd32_xfer_begin(dev);

	if (k_sem_take(&data->sync_sem, K_MSEC(timeout_ms)) != 0) {
		data->errs |= I2C_GD32_ERR_TIMEOUT;
		(void)i2c_gd32_recover_bus_locked(dev);
	}

	err = i2c_gd32_xfer_end(dev);
	if (err < 0) {
		i2c_gd32_log_err(data);
	}

out:
	/* Disable I2C device */
	I2C_CTL0(cfg->reg) &= ~I2C_CTL0_I2CEN;

	k_sem_give(&data->bus_mutex);

	return err;
}

static int i2c_gd32_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t pclk1, freq, clkc;
	int ret;
	int err = 0;

	k_sem_take(&data->bus_mutex, K_FOREVER);

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
	k_sem_give(&data->bus_mutex);

	return err;
}

static DEVICE_API(i2c, i2c_gd32_driver_api) = {
	.configure = i2c_gd32_configure,
	.transfer = i2c_gd32_transfer,
	.recover_bus = i2c_gd32_recover_bus,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
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
	k_sem_init(&data->bus_mutex, 1, 1);

	/* Sync semaphore to sync i2c state between isr and transfer api. */
	k_sem_init(&data->sync_sem, 0, K_SEM_MAX_LIMIT);

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

	i2c_gd32_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);

	return 0;
}

#ifdef CONFIG_I2C_GD32_DMA
#define DMA_INITIALIZER(idx, dir)                                                                  \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(idx, dir)),                         \
		.channel = DT_INST_DMAS_CELL_BY_NAME(idx, dir, channel),                           \
		.slot = COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1),	     \
				    (DT_INST_DMAS_CELL_BY_NAME(idx, dir, slot)), (0)),                            \
			 .config = DT_INST_DMAS_CELL_BY_NAME(idx, dir, config),                    \
			 .fifo_threshold = COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_dma_v1), \
					      (GD32_DMA_DT_FIFO_MODE(                         \
						      DT_INST_DMAS_CELL_BY_NAME(               \
							      idx, dir, fifo_threshold))), \
					      (0)),         \
			 }

#define DMA_BY_NAME(idx, dir)                                                                      \
	COND_CODE_1(DT_INST_DMAS_HAS_NAME(idx, dir), (DMA_INITIALIZER(idx, dir)), ({0}))
#else
#define DMA_BY_NAME(idx, dir) {0}
#endif /* CONFIG_I2C_GD32_DMA */

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
	static struct i2c_gd32_data i2c_gd32_data_##inst;                                          \
	const static struct i2c_gd32_config i2c_gd32_cfg_##inst = {                                \
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
	I2C_DEVICE_DT_INST_DEFINE(inst, i2c_gd32_init, NULL, &i2c_gd32_data_##inst,                \
				  &i2c_gd32_cfg_##inst, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,     \
				  &i2c_gd32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_GD32_INIT)
