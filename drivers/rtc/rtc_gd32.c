/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_rtc

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "rtc_utils.h"

#include <gd32_pmu.h>
#include <gd32_rcu.h>
#include <gd32_rtc.h>

#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_UPDATE)
#include <gd32_exti.h>
#endif

LOG_MODULE_REGISTER(rtc_gd32, CONFIG_RTC_LOG_LEVEL);

enum rtc_gd32_clk_sel {
	RTC_GD32_CLK_LXTAL = 0,
	RTC_GD32_CLK_IRC32K = 1,
	RTC_GD32_CLK_HXTAL = 2,
};

#define RTC_TIME_OFFSET    0x00U
#define RTC_DATE_OFFSET    0x04U
#define RTC_CTL_OFFSET     0x08U
#define RTC_STAT_OFFSET    0x0CU
#define RTC_PSC_OFFSET     0x10U
#define RTC_WUT_OFFSET     0x14U
#define RTC_ALRM0TD_OFFSET 0x1CU
#define RTC_WPK_OFFSET     0x24U
#define RTC_SS_OFFSET      0x28U

struct rtc_gd32_config {
	uintptr_t base;
	enum rtc_gd32_clk_sel clk_sel;
	uint8_t hxtal_div;
	bool lxtal_drive_high;
};

struct rtc_gd32_data {
	struct k_mutex mutex;
	struct k_spinlock lock;
	bool time_valid;

#if defined(CONFIG_RTC_ALARM)
	rtc_alarm_callback alarm_cb;
	void *alarm_user_data;
	bool alarm_pending;
#endif

#if defined(CONFIG_RTC_UPDATE)
	rtc_update_callback update_cb;
	void *update_user_data;
#endif
};

static inline uint32_t rtc_gd32_read(const struct rtc_gd32_config *cfg, uint32_t offset)
{
	return sys_read32(cfg->base + offset);
}

static inline void rtc_gd32_write(const struct rtc_gd32_config *cfg, uint32_t offset, uint32_t val)
{
	sys_write32(val, cfg->base + offset);
}

static inline void rtc_gd32_wp_unlock(const struct rtc_gd32_config *cfg)
{
	rtc_gd32_write(cfg, RTC_WPK_OFFSET, RTC_UNLOCK_KEY1);
	rtc_gd32_write(cfg, RTC_WPK_OFFSET, RTC_UNLOCK_KEY2);
}

static inline void rtc_gd32_wp_lock(const struct rtc_gd32_config *cfg)
{
	rtc_gd32_write(cfg, RTC_WPK_OFFSET, RTC_LOCK_KEY);
}

static int rtc_gd32_wait_stat_set(const struct rtc_gd32_config *cfg, uint32_t mask, int timeout_ms)
{
	int64_t end = k_uptime_get() + timeout_ms;

	while ((rtc_gd32_read(cfg, RTC_STAT_OFFSET) & mask) == 0U) {
		if (k_uptime_get() >= end) {
			return -ETIMEDOUT;
		}
		k_msleep(1);
	}

	return 0;
}

static int rtc_gd32_wait_rcu_flag(rcu_flag_enum flag, int timeout_ms)
{
	int64_t end = k_uptime_get() + timeout_ms;

	while (rcu_flag_get(flag) == RESET) {
		if (k_uptime_get() >= end) {
			return -ETIMEDOUT;
		}
		k_msleep(1);
	}

	return 0;
}

static int rtc_gd32_register_sync_wait(const struct rtc_gd32_config *cfg)
{
	if ((rtc_gd32_read(cfg, RTC_CTL_OFFSET) & RTC_CTL_BPSHAD) != 0U) {
		return 0;
	}

	unsigned int key = irq_lock();
	rtc_gd32_wp_unlock(cfg);
	rtc_gd32_write(cfg, RTC_STAT_OFFSET, rtc_gd32_read(cfg, RTC_STAT_OFFSET) & ~RTC_STAT_RSYNF);
	rtc_gd32_wp_lock(cfg);
	irq_unlock(key);

	return rtc_gd32_wait_stat_set(cfg, RTC_STAT_RSYNF, CONFIG_RTC_GD32_REG_TIMEOUT_MS);
}

static int rtc_gd32_init_mode_enter(const struct rtc_gd32_config *cfg)
{
	if ((rtc_gd32_read(cfg, RTC_STAT_OFFSET) & RTC_STAT_INITF) != 0U) {
		return 0;
	}

	unsigned int key = irq_lock();
	rtc_gd32_wp_unlock(cfg);
	rtc_gd32_write(cfg, RTC_STAT_OFFSET, rtc_gd32_read(cfg, RTC_STAT_OFFSET) | RTC_STAT_INITM);
	rtc_gd32_wp_lock(cfg);
	irq_unlock(key);

	return rtc_gd32_wait_stat_set(cfg, RTC_STAT_INITF, CONFIG_RTC_GD32_REG_TIMEOUT_MS);
}

static void rtc_gd32_init_mode_exit(const struct rtc_gd32_config *cfg)
{
	unsigned int key = irq_lock();
	rtc_gd32_wp_unlock(cfg);
	rtc_gd32_write(cfg, RTC_STAT_OFFSET, rtc_gd32_read(cfg, RTC_STAT_OFFSET) & ~RTC_STAT_INITM);
	rtc_gd32_wp_lock(cfg);
	irq_unlock(key);
}

#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_UPDATE)
static void rtc_gd32_flag_clear(const struct rtc_gd32_config *cfg, uint32_t flag)
{
	unsigned int key = irq_lock();
	rtc_gd32_wp_unlock(cfg);
	rtc_gd32_write(cfg, RTC_STAT_OFFSET, rtc_gd32_read(cfg, RTC_STAT_OFFSET) & ~flag);
	rtc_gd32_wp_lock(cfg);
	irq_unlock(key);
}
#endif

static int rtc_gd32_prescaler_compute(uint32_t rtc_clk_hz, uint16_t *presc_a, uint16_t *presc_s)
{
	for (int a = 0x7F; a >= 0; a--) {
		uint32_t div_a = (uint32_t)a + 1U;

		if ((rtc_clk_hz % div_a) != 0U) {
			continue;
		}

		uint32_t div_s = rtc_clk_hz / div_a;
		if (div_s == 0U || div_s > 0x8000U) {
			continue;
		}

		*presc_a = (uint16_t)a;
		*presc_s = (uint16_t)(div_s - 1U);
		return 0;
	}

	return -EINVAL;
}

static int rtc_gd32_prescaler_apply(const struct rtc_gd32_config *cfg, uint16_t presc_a,
				    uint16_t presc_s)
{
	int ret = rtc_gd32_init_mode_enter(cfg);
	if (ret != 0) {
		return ret;
	}

	unsigned int key = irq_lock();
	rtc_gd32_wp_unlock(cfg);
	rtc_gd32_write(cfg, RTC_PSC_OFFSET, PSC_FACTOR_A(presc_a) | PSC_FACTOR_S(presc_s));
	rtc_gd32_write(cfg, RTC_CTL_OFFSET, rtc_gd32_read(cfg, RTC_CTL_OFFSET) & ~RTC_CTL_CS);
	rtc_gd32_wp_lock(cfg);
	irq_unlock(key);

	rtc_gd32_init_mode_exit(cfg);
	return rtc_gd32_register_sync_wait(cfg);
}

static int rtc_gd32_calendar_set(const struct rtc_gd32_config *cfg, const struct rtc_time *timeptr)
{
	int ret = rtc_gd32_init_mode_enter(cfg);
	if (ret != 0) {
		return ret;
	}

	uint8_t year = (uint8_t)(timeptr->tm_year - 100);
	uint8_t month = (uint8_t)(timeptr->tm_mon + 1);

	uint8_t dow = RTC_MONDAY;
	if (timeptr->tm_wday >= 0 && timeptr->tm_wday <= 6) {
		dow = (timeptr->tm_wday == 0) ? RTC_SUNDAY : (uint8_t)timeptr->tm_wday;
	}

	uint32_t reg_date = DATE_YR(bin2bcd(year)) | DATE_DOW(dow) | DATE_MON(bin2bcd(month)) |
			    DATE_DAY(bin2bcd((uint8_t)timeptr->tm_mday));
	uint32_t reg_time = TIME_HR(bin2bcd((uint8_t)timeptr->tm_hour)) |
			    TIME_MN(bin2bcd((uint8_t)timeptr->tm_min)) |
			    TIME_SC(bin2bcd((uint8_t)timeptr->tm_sec));

	unsigned int key = irq_lock();
	rtc_gd32_wp_unlock(cfg);
	rtc_gd32_write(cfg, RTC_TIME_OFFSET, reg_time);
	rtc_gd32_write(cfg, RTC_DATE_OFFSET, reg_date);
	rtc_gd32_write(cfg, RTC_CTL_OFFSET, rtc_gd32_read(cfg, RTC_CTL_OFFSET) & ~RTC_CTL_CS);
	rtc_gd32_wp_lock(cfg);
	irq_unlock(key);

	rtc_gd32_init_mode_exit(cfg);
	return rtc_gd32_register_sync_wait(cfg);
}

static uint32_t rtc_gd32_dt_clk_sel_to_rcu_src(enum rtc_gd32_clk_sel sel)
{
	switch (sel) {
	case RTC_GD32_CLK_LXTAL:
		return RCU_RTCSRC_LXTAL;
	case RTC_GD32_CLK_IRC32K:
		return RCU_RTCSRC_IRC32K;
	case RTC_GD32_CLK_HXTAL:
		return RCU_RTCSRC_HXTAL_DIV_RTCDIV;
	default:
		return RCU_RTCSRC_NONE;
	}
}

static uint32_t rtc_gd32_rtc_clk_hz(const struct rtc_gd32_config *cfg)
{
	switch (cfg->clk_sel) {
	case RTC_GD32_CLK_LXTAL:
		return 32768U;
	case RTC_GD32_CLK_IRC32K:
#ifdef IRC32K_VALUE
		return (uint32_t)IRC32K_VALUE;
#else
		return 32000U;
#endif
	case RTC_GD32_CLK_HXTAL:
#ifdef HXTAL_VALUE
		return (uint32_t)HXTAL_VALUE / MAX(2U, (uint32_t)cfg->hxtal_div);
#else
		return 0U;
#endif
	default:
		return 0U;
	}
}

static bool rtc_gd32_is_already_configured(const struct rtc_gd32_config *cfg)
{
	return GET_BITS(RCU_BDCTL, 8, 9) != 0U;
}

static int rtc_gd32_clock_configure(const struct rtc_gd32_config *cfg)
{
	int ret;

	if (cfg->clk_sel == RTC_GD32_CLK_LXTAL) {
		if (cfg->lxtal_drive_high) {
			rcu_lxtal_drive_capability_config(RCU_LXTALDRI_HIGHER_DRIVE);
		}
		rcu_osci_on(RCU_LXTAL);
		ret = rtc_gd32_wait_rcu_flag(RCU_FLAG_LXTALSTB, CONFIG_RTC_GD32_OSC_TIMEOUT_MS);
		if (ret != 0) {
			return ret;
		}
		rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);
	} else if (cfg->clk_sel == RTC_GD32_CLK_IRC32K) {
		rcu_osci_on(RCU_IRC32K);
		ret = rtc_gd32_wait_rcu_flag(RCU_FLAG_IRC32KSTB, CONFIG_RTC_GD32_OSC_TIMEOUT_MS);
		if (ret != 0) {
			return ret;
		}
		rcu_rtc_div_config(RCU_RTC_HXTAL_NONE);
		rcu_rtc_clock_config(RCU_RTCSRC_IRC32K);
	} else if (cfg->clk_sel == RTC_GD32_CLK_HXTAL) {
#ifndef HXTAL_VALUE
		return -ENOTSUP;
#endif
		uint32_t div = CLAMP((uint32_t)cfg->hxtal_div, 2U, 31U);
		rcu_osci_on(RCU_HXTAL);
		ret = rtc_gd32_wait_rcu_flag(RCU_FLAG_HXTALSTB, CONFIG_RTC_GD32_OSC_TIMEOUT_MS);
		if (ret != 0) {
			return ret;
		}
		rcu_rtc_div_config(CFG0_RTCDIV(div));
		rcu_rtc_clock_config(RCU_RTCSRC_HXTAL_DIV_RTCDIV);
	} else {
		return -EINVAL;
	}

	return 0;
}

static int rtc_gd32_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	if (timeptr == NULL) {
		return -EINVAL;
	}

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	uint16_t required = RTC_ALARM_TIME_MASK_YEAR | RTC_ALARM_TIME_MASK_MONTH |
			    RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_HOUR |
			    RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_SECOND;
	if (!rtc_utils_validate_rtc_time(timeptr, required)) {
		return -EINVAL;
	}

	const struct rtc_gd32_config *cfg = dev->config;
	struct rtc_gd32_data *data = dev->data;

	k_mutex_lock(&data->mutex, K_FOREVER);
	int ret = rtc_gd32_calendar_set(cfg, timeptr);
	if (ret == 0) {
		data->time_valid = true;
	}
	k_mutex_unlock(&data->mutex);

	if (ret == 0) {
		LOG_DBG("time set: %04d-%02d-%02d %02d:%02d:%02d", timeptr->tm_year + 1900,
			timeptr->tm_mon + 1, timeptr->tm_mday, timeptr->tm_hour, timeptr->tm_min,
			timeptr->tm_sec);
	}

	return ret;
}

static int rtc_gd32_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	if (timeptr == NULL) {
		return -EINVAL;
	}

	struct rtc_gd32_data *data = dev->data;
	if (!data->time_valid) {
		return -ENODATA;
	}

	const struct rtc_gd32_config *cfg = dev->config;

	uint32_t ss = rtc_gd32_read(cfg, RTC_SS_OFFSET);
	uint32_t tr = rtc_gd32_read(cfg, RTC_TIME_OFFSET);
	uint32_t dr = rtc_gd32_read(cfg, RTC_DATE_OFFSET);
	uint32_t psc = rtc_gd32_read(cfg, RTC_PSC_OFFSET);

	uint16_t factor_s = (uint16_t)GET_PSC_FACTOR_S(psc);
	uint32_t sub = ss & RTC_SS_SSC;

	timeptr->tm_sec = bcd2bin((uint8_t)GET_TIME_SC(tr));
	timeptr->tm_min = bcd2bin((uint8_t)GET_TIME_MN(tr));
	timeptr->tm_hour = bcd2bin((uint8_t)GET_TIME_HR(tr));
	timeptr->tm_mday = bcd2bin((uint8_t)GET_DATE_DAY(dr));
	timeptr->tm_mon = bcd2bin((uint8_t)GET_DATE_MON(dr)) - 1;
	timeptr->tm_year = bcd2bin((uint8_t)GET_DATE_YR(dr)) + 100;

	uint8_t dow = (uint8_t)GET_DATE_DOW(dr);
	if (dow == RTC_SUNDAY) {
		timeptr->tm_wday = 0;
	} else if (dow >= RTC_MONDAY && dow <= RTC_SATURDAY) {
		timeptr->tm_wday = dow;
	} else {
		timeptr->tm_wday = -1;
	}

	timeptr->tm_yday = -1;
	timeptr->tm_isdst = -1;

	if (factor_s > 0U) {
		uint64_t num = (uint64_t)(factor_s - sub) * 1000000000ULL;
		timeptr->tm_nsec = (int)(num / (uint64_t)(factor_s + 1U));
	} else {
		timeptr->tm_nsec = 0;
	}

	return 0;
}

#if defined(CONFIG_RTC_ALARM)
static uint16_t rtc_gd32_alarm_supported_fields(void)
{
	return RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |
	       RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_WEEKDAY;
}

static int rtc_gd32_alarm_get_supported_fields(const struct device *dev, uint16_t id,
					       uint16_t *mask)
{
	ARG_UNUSED(dev);

	if (id != 0U || mask == NULL) {
		return -EINVAL;
	}

	*mask = rtc_gd32_alarm_supported_fields();
	return 0;
}

static int rtc_gd32_alarm_disable0(const struct rtc_gd32_config *cfg)
{
	unsigned int key = irq_lock();
	rtc_gd32_wp_unlock(cfg);
	rtc_gd32_write(cfg, RTC_CTL_OFFSET, rtc_gd32_read(cfg, RTC_CTL_OFFSET) & ~RTC_CTL_ALRM0EN);
	rtc_gd32_wp_lock(cfg);
	irq_unlock(key);

	return rtc_gd32_wait_stat_set(cfg, RTC_STAT_ALRM0WF, CONFIG_RTC_GD32_REG_TIMEOUT_MS);
}

static int rtc_gd32_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				   const struct rtc_time *timeptr)
{
	if (id != 0U) {
		return -EINVAL;
	}

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	uint16_t supported = rtc_gd32_alarm_supported_fields();
	if ((mask & ~supported) != 0U) {
		return -EINVAL;
	}

	if (mask == 0U) {
		const struct rtc_gd32_config *cfg = dev->config;
		struct rtc_gd32_data *data = dev->data;

		k_mutex_lock(&data->mutex, K_FOREVER);
		(void)rtc_gd32_alarm_disable0(cfg);
		rtc_interrupt_disable(RTC_INT_ALARM0);
		k_mutex_unlock(&data->mutex);

		k_spinlock_key_t key = k_spin_lock(&data->lock);
		data->alarm_pending = false;
		k_spin_unlock(&data->lock, key);

		return 0;
	}

	if (timeptr == NULL) {
		return -EINVAL;
	}

	if (!rtc_utils_validate_rtc_time(timeptr, mask)) {
		return -EINVAL;
	}

	const struct rtc_gd32_config *cfg = dev->config;
	struct rtc_gd32_data *data = dev->data;

	rtc_alarm_struct alarm_cfg = {
		.alarm_mask = RTC_ALARM_ALL_MASK,
		.weekday_or_date = RTC_ALARM_DATE_SELECTED,
		.alarm_day = 0U,
		.alarm_hour = 0U,
		.alarm_minute = 0U,
		.alarm_second = 0U,
		.am_pm = RTC_AM,
	};

	if ((mask & RTC_ALARM_TIME_MASK_WEEKDAY) != 0U) {
		alarm_cfg.weekday_or_date = RTC_ALARM_WEEKDAY_SELECTED;
		alarm_cfg.alarm_day =
			(timeptr->tm_wday == 0) ? RTC_SUNDAY : (uint8_t)timeptr->tm_wday;
		alarm_cfg.alarm_mask &= ~RTC_ALARM_DATE_MASK;
	} else if ((mask & RTC_ALARM_TIME_MASK_MONTHDAY) != 0U) {
		alarm_cfg.weekday_or_date = RTC_ALARM_DATE_SELECTED;
		alarm_cfg.alarm_day = bin2bcd((uint8_t)timeptr->tm_mday);
		alarm_cfg.alarm_mask &= ~RTC_ALARM_DATE_MASK;
	}

	if ((mask & RTC_ALARM_TIME_MASK_HOUR) != 0U) {
		alarm_cfg.alarm_hour = bin2bcd((uint8_t)timeptr->tm_hour);
		alarm_cfg.alarm_mask &= ~RTC_ALARM_HOUR_MASK;
	}
	if ((mask & RTC_ALARM_TIME_MASK_MINUTE) != 0U) {
		alarm_cfg.alarm_minute = bin2bcd((uint8_t)timeptr->tm_min);
		alarm_cfg.alarm_mask &= ~RTC_ALARM_MINUTE_MASK;
	}
	if ((mask & RTC_ALARM_TIME_MASK_SECOND) != 0U) {
		alarm_cfg.alarm_second = bin2bcd((uint8_t)timeptr->tm_sec);
		alarm_cfg.alarm_mask &= ~RTC_ALARM_SECOND_MASK;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	int ret = rtc_gd32_alarm_disable0(cfg);
	if (ret == 0) {
		rtc_gd32_flag_clear(cfg, RTC_FLAG_ALRM0);
		rtc_alarm_config(RTC_ALARM0, &alarm_cfg);
		rtc_alarm_enable(RTC_ALARM0);
		rtc_interrupt_enable(RTC_INT_ALARM0);

		exti_flag_clear(EXTI_17);
		exti_init(EXTI_17, EXTI_INTERRUPT, EXTI_TRIG_RISING);
	}

	k_mutex_unlock(&data->mutex);

	if (ret == 0) {
		k_spinlock_key_t key = k_spin_lock(&data->lock);
		data->alarm_pending = false;
		k_spin_unlock(&data->lock, key);
	}

	return ret;
}

static int rtc_gd32_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
				   struct rtc_time *timeptr)
{
	if (id != 0U || mask == NULL || timeptr == NULL) {
		return -EINVAL;
	}

	const struct rtc_gd32_config *cfg = dev->config;
	uint32_t ctl = rtc_gd32_read(cfg, RTC_CTL_OFFSET);

	memset(timeptr, 0, sizeof(*timeptr));
	timeptr->tm_sec = -1;
	timeptr->tm_min = -1;
	timeptr->tm_hour = -1;
	timeptr->tm_mday = -1;
	timeptr->tm_mon = -1;
	timeptr->tm_year = -1;
	timeptr->tm_wday = -1;
	timeptr->tm_yday = -1;
	timeptr->tm_isdst = -1;
	timeptr->tm_nsec = 0;

	if ((ctl & RTC_CTL_ALRM0EN) == 0U) {
		*mask = 0U;
		return 0;
	}

	rtc_alarm_struct alarm_cfg;
	rtc_alarm_get(RTC_ALARM0, &alarm_cfg);

	uint16_t zephyr_mask = 0U;
	if ((alarm_cfg.alarm_mask & RTC_ALARM_SECOND_MASK) == 0U) {
		zephyr_mask |= RTC_ALARM_TIME_MASK_SECOND;
		timeptr->tm_sec = bcd2bin(alarm_cfg.alarm_second);
	}
	if ((alarm_cfg.alarm_mask & RTC_ALARM_MINUTE_MASK) == 0U) {
		zephyr_mask |= RTC_ALARM_TIME_MASK_MINUTE;
		timeptr->tm_min = bcd2bin(alarm_cfg.alarm_minute);
	}
	if ((alarm_cfg.alarm_mask & RTC_ALARM_HOUR_MASK) == 0U) {
		zephyr_mask |= RTC_ALARM_TIME_MASK_HOUR;
		timeptr->tm_hour = bcd2bin(alarm_cfg.alarm_hour);
	}
	if ((alarm_cfg.alarm_mask & RTC_ALARM_DATE_MASK) == 0U) {
		if (alarm_cfg.weekday_or_date == RTC_ALARM_WEEKDAY_SELECTED) {
			zephyr_mask |= RTC_ALARM_TIME_MASK_WEEKDAY;
			timeptr->tm_wday =
				(alarm_cfg.alarm_day == RTC_SUNDAY) ? 0 : alarm_cfg.alarm_day;
		} else {
			zephyr_mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
			timeptr->tm_mday = bcd2bin(alarm_cfg.alarm_day);
		}
	}

	*mask = zephyr_mask;
	return 0;
}

static int rtc_gd32_alarm_is_pending(const struct device *dev, uint16_t id)
{
	if (id != 0U) {
		return -EINVAL;
	}

	struct rtc_gd32_data *data = dev->data;
	bool pending;

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	pending = data->alarm_pending;
	data->alarm_pending = false;
	k_spin_unlock(&data->lock, key);

	return pending ? 1 : 0;
}

static int rtc_gd32_alarm_set_callback(const struct device *dev, uint16_t id,
				       rtc_alarm_callback callback, void *user_data)
{
	if (id != 0U) {
		return -EINVAL;
	}

	struct rtc_gd32_data *data = dev->data;

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	data->alarm_cb = callback;
	data->alarm_user_data = user_data;
	k_spin_unlock(&data->lock, key);

	return 0;
}

static void rtc_gd32_alarm_isr(const struct device *dev)
{
	const struct rtc_gd32_config *cfg = dev->config;
	struct rtc_gd32_data *data = dev->data;

	if (rtc_flag_get(RTC_FLAG_ALRM0) == RESET) {
		return;
	}

	rtc_gd32_flag_clear(cfg, RTC_FLAG_ALRM0);
	exti_flag_clear(EXTI_17);

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	rtc_alarm_callback cb = data->alarm_cb;
	void *ud = data->alarm_user_data;

	if (cb != NULL) {
		k_spin_unlock(&data->lock, key);
		cb(dev, 0U, ud);
	} else {
		data->alarm_pending = true;
		k_spin_unlock(&data->lock, key);
	}
}
#endif /* CONFIG_RTC_ALARM */

#if defined(CONFIG_RTC_UPDATE)
static int rtc_gd32_wakeup_wait_wtwf(const struct rtc_gd32_config *cfg)
{
	return rtc_gd32_wait_stat_set(cfg, RTC_STAT_WTWF, CONFIG_RTC_GD32_REG_TIMEOUT_MS);
}

static int rtc_gd32_wakeup_disable(const struct rtc_gd32_config *cfg)
{
	unsigned int key = irq_lock();
	rtc_gd32_wp_unlock(cfg);
	rtc_gd32_write(cfg, RTC_CTL_OFFSET, rtc_gd32_read(cfg, RTC_CTL_OFFSET) & ~RTC_CTL_WTEN);
	rtc_gd32_wp_lock(cfg);
	irq_unlock(key);

	return rtc_gd32_wakeup_wait_wtwf(cfg);
}

static int rtc_gd32_wakeup_config_1hz(const struct rtc_gd32_config *cfg)
{
	int ret = rtc_gd32_wakeup_disable(cfg);
	if (ret != 0) {
		return ret;
	}

	ret = rtc_gd32_wakeup_wait_wtwf(cfg);
	if (ret != 0) {
		return ret;
	}

	unsigned int key = irq_lock();
	rtc_gd32_wp_unlock(cfg);
	rtc_gd32_write(cfg, RTC_CTL_OFFSET,
		       (rtc_gd32_read(cfg, RTC_CTL_OFFSET) & ~RTC_CTL_WTCS) |
			       (uint32_t)WAKEUP_CKSPRE);
	rtc_gd32_write(cfg, RTC_WUT_OFFSET, 0U);
	rtc_gd32_wp_lock(cfg);
	irq_unlock(key);

	rtc_gd32_flag_clear(cfg, RTC_FLAG_WT);
	rtc_interrupt_enable(RTC_INT_WAKEUP);
	rtc_wakeup_enable();

	return 0;
}

static int rtc_gd32_update_set_callback(const struct device *dev, rtc_update_callback callback,
					void *user_data)
{
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	const struct rtc_gd32_config *cfg = dev->config;
	struct rtc_gd32_data *data = dev->data;

	k_mutex_lock(&data->mutex, K_FOREVER);

	int ret = 0;
	if (callback != NULL) {
		ret = rtc_gd32_wakeup_config_1hz(cfg);
		if (ret == 0) {
			exti_flag_clear(EXTI_22);
			exti_init(EXTI_22, EXTI_INTERRUPT, EXTI_TRIG_RISING);
		}
	} else {
		(void)rtc_gd32_wakeup_disable(cfg);
		rtc_interrupt_disable(RTC_INT_WAKEUP);
	}

	k_mutex_unlock(&data->mutex);

	if (ret == 0) {
		k_spinlock_key_t key = k_spin_lock(&data->lock);
		data->update_cb = callback;
		data->update_user_data = user_data;
		k_spin_unlock(&data->lock, key);
	}

	return ret;
}

static void rtc_gd32_wakeup_isr(const struct device *dev)
{
	const struct rtc_gd32_config *cfg = dev->config;
	struct rtc_gd32_data *data = dev->data;

	if (rtc_flag_get(RTC_FLAG_WT) == RESET) {
		return;
	}

	rtc_gd32_flag_clear(cfg, RTC_FLAG_WT);
	exti_flag_clear(EXTI_22);

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	rtc_update_callback cb = data->update_cb;
	void *ud = data->update_user_data;
	k_spin_unlock(&data->lock, key);

	if (cb != NULL) {
		cb(dev, ud);
	}
}
#endif /* CONFIG_RTC_UPDATE */

static DEVICE_API(rtc, rtc_gd32_driver_api) = {
	.set_time = rtc_gd32_set_time,
	.get_time = rtc_gd32_get_time,
#if defined(CONFIG_RTC_ALARM)
	.alarm_get_supported_fields = rtc_gd32_alarm_get_supported_fields,
	.alarm_set_time = rtc_gd32_alarm_set_time,
	.alarm_get_time = rtc_gd32_alarm_get_time,
	.alarm_is_pending = rtc_gd32_alarm_is_pending,
	.alarm_set_callback = rtc_gd32_alarm_set_callback,
#endif
#if defined(CONFIG_RTC_UPDATE)
	.update_set_callback = rtc_gd32_update_set_callback,
#endif
};

static int rtc_gd32_init(const struct device *dev)
{
	const struct rtc_gd32_config *cfg = dev->config;
	struct rtc_gd32_data *data = dev->data;

	k_mutex_init(&data->mutex);
	data->time_valid = false;

	rcu_periph_clock_enable(RCU_PMU);
	pmu_backup_write_enable();

	bool already = rtc_gd32_is_already_configured(cfg);

	if (!already) {
		uint32_t clk_hz = rtc_gd32_rtc_clk_hz(cfg);
		if (clk_hz == 0U) {
			LOG_ERR("unsupported RTC clock source");
			return -ENOTSUP;
		}

		int ret = rtc_gd32_clock_configure(cfg);
		if (ret != 0) {
			LOG_ERR("RTC clock source failed to start (%d)", ret);
			return ret;
		}

		rcu_periph_clock_enable(RCU_RTC);

		ret = rtc_gd32_register_sync_wait(cfg);
		if (ret != 0) {
			LOG_ERR("RTC register sync timeout (%d)", ret);
			return ret;
		}

		uint16_t presc_a;
		uint16_t presc_s;
		ret = rtc_gd32_prescaler_compute(clk_hz, &presc_a, &presc_s);
		if (ret != 0) {
			LOG_ERR("cannot derive prescalers for rtc_clk=%u", clk_hz);
			return ret;
		}

		ret = rtc_gd32_prescaler_apply(cfg, presc_a, presc_s);
		if (ret != 0) {
			LOG_ERR("RTC prescaler apply failed (%d)", ret);
			return ret;
		}

		LOG_INF("configured RTC (clk=%u Hz, presc_a=0x%x presc_s=0x%x)", clk_hz, presc_a,
			presc_s);
	} else {
		rcu_periph_clock_enable(RCU_RTC);

		int ret = rtc_gd32_register_sync_wait(cfg);
		if (ret != 0) {
			LOG_WRN("RTC register sync timeout (%d)", ret);
		}

		uint32_t rcu_src = rtc_gd32_dt_clk_sel_to_rcu_src(cfg->clk_sel);
		if (rcu_src != RCU_RTCSRC_NONE && (RCU_BDCTL & RCU_BDCTL_RTCSRC) != rcu_src) {
			LOG_WRN("RTC clock source differs from devicetree; preserving existing");
		}
	}

	/* RTC_STAT.YCM is a hardware "year configuration mark" bit (RM: indicates calendar init).
	 * Use it to implement Zephyr's rtc_get_time() contract: return -ENODATA until the calendar
	 * has been set at least once after backup-domain reset/power loss.
	 */
	if ((rtc_gd32_read(cfg, RTC_STAT_OFFSET) & RTC_STAT_YCM) != 0U) {
		data->time_valid = true;
	}

#if defined(CONFIG_RTC_ALARM) && DT_INST_IRQ_HAS_IDX(0, 0)
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 0, irq), DT_INST_IRQ_BY_IDX(0, 0, priority),
		    rtc_gd32_alarm_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_IDX(0, 0, irq));
#endif

#if defined(CONFIG_RTC_UPDATE) && DT_INST_IRQ_HAS_IDX(0, 2)
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(0, 2, irq), DT_INST_IRQ_BY_IDX(0, 2, priority),
		    rtc_gd32_wakeup_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQ_BY_IDX(0, 2, irq));
#endif

	return 0;
}

static const struct rtc_gd32_config rtc_gd32_cfg = {
	.base = DT_INST_REG_ADDR(0),
	.clk_sel = (enum rtc_gd32_clk_sel)DT_INST_ENUM_IDX_OR(0, clock_source, RTC_GD32_CLK_LXTAL),
	.hxtal_div = (uint8_t)DT_INST_PROP_OR(0, hxtal_div, 2),
	.lxtal_drive_high = DT_INST_PROP_OR(0, lxtal_drive_high, 0),
};

static struct rtc_gd32_data rtc_gd32_data;

DEVICE_DT_INST_DEFINE(0, rtc_gd32_init, NULL, &rtc_gd32_data, &rtc_gd32_cfg, POST_KERNEL,
		      CONFIG_RTC_INIT_PRIORITY, &rtc_gd32_driver_api);
