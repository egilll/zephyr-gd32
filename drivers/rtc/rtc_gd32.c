/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * Driver for the GigaDevice chips that have an calendar-based RTC peripheral, which are:
 * - GD32F3x0, which has one alarm but no wakeup timer.
 * - GD32F4xx, GD32F5xx, and GD32L23x, which have two alarms and wakeup timer.
 *
 *
 * Uses RTC backup registers like so:
 * - RTC_BKP0 stores a magic value indicating the RTC has been initialised and is still valid after
 *   a reboot.
 * - RTC_BKP1 persists the full tm_year on every read (if needed) and is used to prevent Y2K.
 *   Y2K prevention therefore requires the user to read the time at least once a century. Tamper
 *   events erase the backup registers by default on these RTC blocks. On GD32L23x this driver
 *   enables the TPxNOERASE bits so tamper detection will not wipe RTC_BKP0/RTC_BKP1.
 */
#define DT_DRV_COMPAT gd_gd32_rtc

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/sys/util.h>

#include "rtc_utils.h"

#include <gd32_pmu.h>
#include <gd32_rcu.h>
#include <gd32_rtc.h>
#include <gd32_exti.h>

LOG_MODULE_REGISTER(gd32_rtc, CONFIG_RTC_LOG_LEVEL);

#if defined(CONFIG_SOC_SERIES_GD32F3X0)
#define GD32_RTC_SOC_NEEDS_BKP_CLK    false
#define GD32_RTC_SOC_ALARM_EXTI_LINE  EXTI_17
#define GD32_RTC_SOC_WAKEUP_EXTI_LINE 0U
#elif defined(CONFIG_SOC_SERIES_GD32F4XX) || defined(CONFIG_SOC_SERIES_GD32F5XX)
#define GD32_RTC_SOC_NEEDS_BKP_CLK    false
#define GD32_RTC_SOC_ALARM_EXTI_LINE  EXTI_17
#define GD32_RTC_SOC_WAKEUP_EXTI_LINE EXTI_22
#elif defined(CONFIG_SOC_SERIES_GD32L23X)
#define GD32_RTC_SOC_NEEDS_BKP_CLK    true
#define GD32_RTC_SOC_ALARM_EXTI_LINE  EXTI_17
#define GD32_RTC_SOC_WAKEUP_EXTI_LINE EXTI_22
#else
#error "Unsupported GD32 calendar RTC SoC series"
#endif

enum gd32_rtc_clk_sel {
	GD32_RTC_CLK_LXTAL,
	GD32_RTC_CLK_IRC40K,
	GD32_RTC_CLK_IRC32K,
	GD32_RTC_CLK_HXTAL_DIV32,
	GD32_RTC_CLK_HXTAL_DIV_RTCDIV,
};

#ifndef GD32_RTC_INITIALIZED_MAGIC_VALUE
#define GD32_RTC_INITIALIZED_MAGIC_VALUE 0x32F1U
#endif

#ifndef GD32_RTC_BASE_YEAR
#define GD32_RTC_BASE_YEAR 2000U
#endif

#define GD32_RTC_MAX_ALARMS               2U
#define GD32_RTC_OSC_TIMEOUT_MS           2000
#define GD32_RTC_REG_TIMEOUT_MS           100
#define GD32_RTC_BYPASS_READ_MAX_ATTEMPTS 5

#define GD32_RTC_ALRM_SC_SHIFT    0U
#define GD32_RTC_ALRM_MN_SHIFT    8U
#define GD32_RTC_ALRM_HR_SHIFT    16U
#define GD32_RTC_ALRM_DAY_SHIFT   24U
#define GD32_RTC_ALRM_SC_MASK     BIT(7)
#define GD32_RTC_ALRM_MN_MASK     BIT(15)
#define GD32_RTC_ALRM_HR_MASK     BIT(23)
#define GD32_RTC_ALRM_DAY_MASK    BIT(31)
#define GD32_RTC_ALRM_WEEKDAY_SEL BIT(30)
#define GD32_RTC_ALRM_SS_ALL_MASK 0U
#define GD32_RTC_ALRM0TD_ADDR     (RTC_BASE + 0x1CU)
#define GD32_RTC_ALRM1TD_ADDR     (RTC_BASE + 0x20U)
#define GD32_RTC_ALRM0SS_ADDR     (RTC_BASE + 0x44U)
#define GD32_RTC_ALRM1SS_ADDR     (RTC_BASE + 0x48U)

struct gd32_rtc_config {
	const char *clock_source;
	uint8_t alarms_count;
	uint8_t hxtal_div;
	uint32_t alarm_exti_line;
	uint32_t wakeup_exti_line;
	bool lxtal_drive_high;
	bool needs_bkp_clk;
	bool has_wakeup;
};

struct gd32_rtc_data {
	struct k_mutex mutex;
	struct k_spinlock lock;
	bool time_valid;

#if defined(CONFIG_RTC_ALARM)
	rtc_alarm_callback alarm_cb[GD32_RTC_MAX_ALARMS];
	void *alarm_user_data[GD32_RTC_MAX_ALARMS];
	bool alarm_pending[GD32_RTC_MAX_ALARMS];
#endif

#if defined(CONFIG_RTC_UPDATE)
	rtc_update_callback update_cb;
	void *update_user_data;
#endif
};

static inline void gd32_rtc_wp_unlock(void)
{
	RTC_WPK = RTC_UNLOCK_KEY1;
	RTC_WPK = RTC_UNLOCK_KEY2;
}

static inline void gd32_rtc_wp_lock(void)
{
	RTC_WPK = RTC_LOCK_KEY;
}

static int gd32_rtc_wait_stat_set(uint32_t mask, int timeout_ms)
{
	int64_t end = k_uptime_get() + timeout_ms;

	while ((RTC_STAT & mask) == 0U) {
		if (k_uptime_get() >= end) {
			return -ETIMEDOUT;
		}
		k_msleep(1);
	}

	return 0;
}

static int gd32_rtc_wait_rcu_flag(rcu_flag_enum flag, int timeout_ms)
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

static bool gd32_rtc_is_leap_year(uint32_t full_year)
{
	return ((full_year % 400U) == 0U) ||
	       (((full_year % 4U) == 0U) && ((full_year % 100U) != 0U));
}

static uint8_t gd32_rtc_days_in_month(uint32_t full_year, uint8_t month)
{
	static const uint8_t days_per_month[] = {
		31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U,
	};

	uint8_t days = days_per_month[month - 1U];

	if ((month == 2U) && gd32_rtc_is_leap_year(full_year)) {
		days++;
	}

	return days;
}

static uint8_t gd32_rtc_weekday_compute(uint32_t full_year, uint8_t month, uint8_t day)
{
	static const uint8_t month_offsets[] = {
		0U, 3U, 2U, 5U, 0U, 3U, 5U, 1U, 4U, 6U, 2U, 4U,
	};
	uint32_t year = full_year;

	if (month < 3U) {
		year--;
	}

	return (uint8_t)((year + (year / 4U) - (year / 100U) + (year / 400U) +
			  month_offsets[month - 1U] + day) %
			 7U);
}

static int gd32_rtc_hw_to_zephyr_wday(uint8_t hw_wday, int *zephyr_wday)
{
	if (zephyr_wday == NULL) {
		return -EINVAL;
	}

	if (hw_wday == RTC_SUNDAY) {
		*zephyr_wday = 0;
		return 0;
	}

	if ((hw_wday >= RTC_MONDAY) && (hw_wday <= RTC_SATURDAY)) {
		*zephyr_wday = hw_wday;
		return 0;
	}

	return -EINVAL;
}

static int gd32_rtc_zephyr_to_hw_wday(uint8_t zephyr_wday, uint8_t *hw_wday)
{
	if ((hw_wday == NULL) || (zephyr_wday > 6U)) {
		return -EINVAL;
	}

	*hw_wday = (zephyr_wday == 0U) ? RTC_SUNDAY : zephyr_wday;
	return 0;
}

static int gd32_rtc_validate_calendar_time(const struct rtc_time *timeptr, uint8_t *hw_wday)
{
	const uint16_t required = RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_MONTHDAY |
				  RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_MINUTE |
				  RTC_ALARM_TIME_MASK_SECOND;

	if ((timeptr == NULL) || (timeptr->tm_year < 0) ||
	    !rtc_utils_validate_rtc_time(timeptr, required)) {
		return -EINVAL;
	}

	uint32_t full_year = (uint32_t)timeptr->tm_year + TIME_UTILS_BASE_YEAR;
	uint8_t month = (uint8_t)timeptr->tm_mon + 1U;
	uint8_t day = (uint8_t)timeptr->tm_mday;

	if (day > gd32_rtc_days_in_month(full_year, month)) {
		return -EINVAL;
	}

	return gd32_rtc_zephyr_to_hw_wday(gd32_rtc_weekday_compute(full_year, month, day), hw_wday);
}

static bool gd32_rtc_get_persisted_tm_year(uint32_t *persisted_tm_year)
{
	uint32_t magic = RTC_BKP0;
	uint32_t year = RTC_BKP1;

	if ((persisted_tm_year == NULL) || (magic != GD32_RTC_INITIALIZED_MAGIC_VALUE) ||
	    (year > (uint32_t)INT_MAX)) {
		return false;
	}

	*persisted_tm_year = year;
	return true;
}

static void gd32_rtc_set_persisted_tm_year(uint32_t persisted_tm_year)
{
	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_BKP1 = persisted_tm_year;
	RTC_BKP0 = GD32_RTC_INITIALIZED_MAGIC_VALUE;
	gd32_rtc_wp_lock();

	irq_unlock(key);
}

static bool gd32_rtc_legacy_has_year_been_set(void)
{
	return (RTC_STAT & RTC_STAT_YCM) != 0U;
}

static int gd32_rtc_register_sync_wait(void)
{
	if ((RTC_CTL & RTC_CTL_BPSHAD) != 0U) {
		return 0;
	}

	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_STAT &= ~RTC_STAT_RSYNF;
	gd32_rtc_wp_lock();

	irq_unlock(key);

	return gd32_rtc_wait_stat_set(RTC_STAT_RSYNF, GD32_RTC_REG_TIMEOUT_MS);
}

struct gd32_rtc_calendar_reading {
	uint32_t ss;
	uint32_t tr;
	uint32_t dr;
};

static bool gd32_rtc_calendar_reading_equal(const struct gd32_rtc_calendar_reading *lhs,
					    const struct gd32_rtc_calendar_reading *rhs)
{
	return (lhs->ss == rhs->ss) && (lhs->tr == rhs->tr) && (lhs->dr == rhs->dr);
}

static int gd32_rtc_calendar_read(struct gd32_rtc_calendar_reading *reading)
{
	if (reading == NULL) {
		return -EINVAL;
	}

	if ((RTC_CTL & RTC_CTL_BPSHAD) == 0U) {
		int ret = gd32_rtc_register_sync_wait();

		if (ret != 0) {
			return ret;
		}

		reading->ss = RTC_SS;
		reading->tr = RTC_TIME;
		reading->dr = RTC_DATE;

		return 0;
	}

	for (int i = 0; i < GD32_RTC_BYPASS_READ_MAX_ATTEMPTS; i++) {
		struct gd32_rtc_calendar_reading current = {
			.ss = RTC_SS,
			.tr = RTC_TIME,
			.dr = RTC_DATE,
		};
		struct gd32_rtc_calendar_reading next = {
			.ss = RTC_SS,
			.tr = RTC_TIME,
			.dr = RTC_DATE,
		};

		if (gd32_rtc_calendar_reading_equal(&current, &next)) {
			*reading = next;
			return 0;
		}
	}

	return -EIO;
}

static int gd32_rtc_init_mode_enter(void)
{
	if ((RTC_STAT & RTC_STAT_INITF) != 0U) {
		return 0;
	}

	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_STAT |= RTC_STAT_INITM;
	gd32_rtc_wp_lock();

	irq_unlock(key);

	return gd32_rtc_wait_stat_set(RTC_STAT_INITF, GD32_RTC_REG_TIMEOUT_MS);
}

static void gd32_rtc_init_mode_exit(void)
{
	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_STAT &= ~RTC_STAT_INITM;
	gd32_rtc_wp_lock();

	irq_unlock(key);
}

#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_UPDATE)
static void gd32_rtc_flag_clear(uint32_t flag)
{
	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_STAT &= ~flag;
	gd32_rtc_wp_lock();

	irq_unlock(key);
}

static void gd32_rtc_interrupt_update(uint32_t mask, bool enable)
{
	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	if (enable) {
		RTC_CTL |= mask;
	} else {
		RTC_CTL &= ~mask;
	}
	gd32_rtc_wp_lock();

	irq_unlock(key);
}
#endif /* CONFIG_RTC_ALARM || CONFIG_RTC_UPDATE */

static void gd32_rtc_tamper_bkp_preserve_enable(void)
{
#if defined(CONFIG_SOC_SERIES_GD32L23X) && defined(RTC_TAMP_TP0NOERASE) &&                         \
	defined(RTC_TAMP_TP1NOERASE) && defined(RTC_TAMP_TP2NOERASE)
	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_TAMP |= RTC_TAMP_TP0NOERASE | RTC_TAMP_TP1NOERASE | RTC_TAMP_TP2NOERASE;
	gd32_rtc_wp_lock();

	irq_unlock(key);
#else
	/* Only GD32L23x exposes TPxNOERASE for the calendar RTC tamper inputs. */
#endif
}

static int gd32_rtc_prescaler_compute(uint32_t rtc_clk_hz, uint16_t *presc_a, uint16_t *presc_s)
{
	for (int a = 0x7F; a >= 0; a--) {
		uint32_t div_a = (uint32_t)a + 1U;

		if ((rtc_clk_hz % div_a) != 0U) {
			continue;
		}

		uint32_t div_s = rtc_clk_hz / div_a;

		if ((div_s == 0U) || (div_s > 0x8000U)) {
			continue;
		}

		*presc_a = (uint16_t)a;
		*presc_s = (uint16_t)(div_s - 1U);
		return 0;
	}

	return -EINVAL;
}

static int gd32_rtc_prescaler_apply(uint16_t presc_a, uint16_t presc_s)
{
	int ret = gd32_rtc_init_mode_enter();

	if (ret != 0) {
		return ret;
	}

	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_PSC = PSC_FACTOR_A(presc_a) | PSC_FACTOR_S(presc_s);
	RTC_CTL &= ~RTC_CTL_CS;
	gd32_rtc_wp_lock();

	irq_unlock(key);

	gd32_rtc_init_mode_exit();
	return gd32_rtc_register_sync_wait();
}

static int gd32_rtc_calendar_set(const struct rtc_time *timeptr, uint8_t hw_wday)
{
	int ret = gd32_rtc_init_mode_enter();

	if (ret != 0) {
		return ret;
	}

	uint8_t year = (uint8_t)(timeptr->tm_year % 100);
	uint8_t month = (uint8_t)(timeptr->tm_mon + 1);
	uint32_t reg_date = DATE_YR(bin2bcd(year)) | DATE_DOW(hw_wday) | DATE_MON(bin2bcd(month)) |
			    DATE_DAY(bin2bcd((uint8_t)timeptr->tm_mday));
	uint32_t reg_time = TIME_HR(bin2bcd((uint8_t)timeptr->tm_hour)) |
			    TIME_MN(bin2bcd((uint8_t)timeptr->tm_min)) |
			    TIME_SC(bin2bcd((uint8_t)timeptr->tm_sec));
	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_TIME = reg_time;
	RTC_DATE = reg_date;
	RTC_CTL &= ~RTC_CTL_CS;
	gd32_rtc_wp_lock();

	irq_unlock(key);

	gd32_rtc_init_mode_exit();
	return gd32_rtc_register_sync_wait();
}

static int gd32_rtc_clock_source_parse(const struct gd32_rtc_config *cfg,
				       enum gd32_rtc_clk_sel *clk_sel)
{
	if ((cfg == NULL) || (clk_sel == NULL) || (cfg->clock_source == NULL)) {
		return -EINVAL;
	}

	if (strcmp(cfg->clock_source, "lxtal") == 0) {
		*clk_sel = GD32_RTC_CLK_LXTAL;
		return 0;
	}

	if (strcmp(cfg->clock_source, "irc40k") == 0) {
#if !defined(CONFIG_SOC_SERIES_GD32F3X0)
		return -EINVAL;
#endif

		*clk_sel = GD32_RTC_CLK_IRC40K;
		return 0;
	}

	if (strcmp(cfg->clock_source, "irc32k") == 0) {
#if defined(CONFIG_SOC_SERIES_GD32F3X0)
		return -EINVAL;
#endif

		*clk_sel = GD32_RTC_CLK_IRC32K;
		return 0;
	}

	if (strcmp(cfg->clock_source, "hxtal-div32") == 0) {
#if defined(CONFIG_SOC_SERIES_GD32F4XX) || defined(CONFIG_SOC_SERIES_GD32F5XX)
		return -EINVAL;
#endif

		*clk_sel = GD32_RTC_CLK_HXTAL_DIV32;
		return 0;
	}

	if (strcmp(cfg->clock_source, "hxtal-div-rtcdiv") == 0) {
#if !defined(CONFIG_SOC_SERIES_GD32F4XX) && !defined(CONFIG_SOC_SERIES_GD32F5XX)
		return -EINVAL;
#endif

		*clk_sel = GD32_RTC_CLK_HXTAL_DIV_RTCDIV;
		return 0;
	}

	return -EINVAL;
}

static uint32_t gd32_rtc_clk_sel_to_rcu_src(enum gd32_rtc_clk_sel clk_sel)
{
	switch (clk_sel) {
	case GD32_RTC_CLK_LXTAL:
		return RCU_RTCSRC_LXTAL;
	case GD32_RTC_CLK_IRC40K:
#ifdef RCU_RTCSRC_IRC40K
		return RCU_RTCSRC_IRC40K;
#else
		return RCU_RTCSRC_NONE;
#endif
	case GD32_RTC_CLK_IRC32K:
#ifdef RCU_RTCSRC_IRC32K
		return RCU_RTCSRC_IRC32K;
#else
		return RCU_RTCSRC_NONE;
#endif
	case GD32_RTC_CLK_HXTAL_DIV32:
#ifdef RCU_RTCSRC_HXTAL_DIV32
		return RCU_RTCSRC_HXTAL_DIV32;
#else
		return RCU_RTCSRC_NONE;
#endif
	case GD32_RTC_CLK_HXTAL_DIV_RTCDIV:
#ifdef RCU_RTCSRC_HXTAL_DIV_RTCDIV
		return RCU_RTCSRC_HXTAL_DIV_RTCDIV;
#else
		return RCU_RTCSRC_NONE;
#endif
	default:
		return RCU_RTCSRC_NONE;
	}
}

static uint32_t gd32_rtc_clk_hz(const struct gd32_rtc_config *cfg, enum gd32_rtc_clk_sel clk_sel)
{
	switch (clk_sel) {
	case GD32_RTC_CLK_LXTAL:
		return 32768U;
	case GD32_RTC_CLK_IRC40K:
#ifdef IRC40K_VALUE
		return (uint32_t)IRC40K_VALUE;
#else
		return 40000U;
#endif
	case GD32_RTC_CLK_IRC32K:
#ifdef IRC32K_VALUE
		return (uint32_t)IRC32K_VALUE;
#else
		return 32000U;
#endif
	case GD32_RTC_CLK_HXTAL_DIV32:
#ifdef HXTAL_VALUE
		return (uint32_t)HXTAL_VALUE / 32U;
#else
		return 0U;
#endif
	case GD32_RTC_CLK_HXTAL_DIV_RTCDIV:
#ifdef HXTAL_VALUE
		return (uint32_t)HXTAL_VALUE / MAX(2U, (uint32_t)cfg->hxtal_div);
#else
		return 0U;
#endif
	default:
		return 0U;
	}
}

static bool gd32_rtc_is_already_configured(void)
{
	return (RCU_BDCTL & RCU_BDCTL_RTCSRC) != 0U;
}

static void gd32_rtc_lxtal_drive_high_apply(const struct gd32_rtc_config *cfg)
{
	if (!cfg->lxtal_drive_high) {
		return;
	}

#if defined(CONFIG_SOC_SERIES_GD32F4XX) || defined(CONFIG_SOC_SERIES_GD32F5XX)
#if defined(RCU_LXTALDRI_HIGHER_DRIVE)
	rcu_lxtal_drive_capability_config(RCU_LXTALDRI_HIGHER_DRIVE);
#endif
#endif

#if defined(CONFIG_SOC_SERIES_GD32F3X0) || defined(CONFIG_SOC_SERIES_GD32L23X)
	rcu_lxtal_drive_capability_config(RCU_LXTAL_HIGHDRI);
#endif
}

static int gd32_rtc_clock_configure(const struct gd32_rtc_config *cfg,
				    enum gd32_rtc_clk_sel clk_sel)
{
	int ret;

	switch (clk_sel) {
	case GD32_RTC_CLK_LXTAL:
		gd32_rtc_lxtal_drive_high_apply(cfg);
		rcu_osci_on(RCU_LXTAL);
		ret = gd32_rtc_wait_rcu_flag(RCU_FLAG_LXTALSTB, GD32_RTC_OSC_TIMEOUT_MS);
		if (ret != 0) {
			return ret;
		}

		rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);
		return 0;
	case GD32_RTC_CLK_IRC40K:
#ifdef RCU_IRC40K
		rcu_osci_on(RCU_IRC40K);
		ret = gd32_rtc_wait_rcu_flag(RCU_FLAG_IRC40KSTB, GD32_RTC_OSC_TIMEOUT_MS);
		if (ret != 0) {
			return ret;
		}

		rcu_rtc_clock_config(RCU_RTCSRC_IRC40K);
		return 0;
#else
		return -ENOTSUP;
#endif
	case GD32_RTC_CLK_IRC32K:
#ifdef RCU_IRC32K
		rcu_osci_on(RCU_IRC32K);
		ret = gd32_rtc_wait_rcu_flag(RCU_FLAG_IRC32KSTB, GD32_RTC_OSC_TIMEOUT_MS);
		if (ret != 0) {
			return ret;
		}

#ifdef RCU_RTC_HXTAL_NONE
		rcu_rtc_div_config(RCU_RTC_HXTAL_NONE);
#endif
		rcu_rtc_clock_config(RCU_RTCSRC_IRC32K);
		return 0;
#else
		return -ENOTSUP;
#endif
	case GD32_RTC_CLK_HXTAL_DIV32:
#if defined(HXTAL_VALUE) && defined(RCU_RTCSRC_HXTAL_DIV32)
		rcu_osci_on(RCU_HXTAL);
		ret = gd32_rtc_wait_rcu_flag(RCU_FLAG_HXTALSTB, GD32_RTC_OSC_TIMEOUT_MS);
		if (ret != 0) {
			return ret;
		}

		rcu_rtc_clock_config(RCU_RTCSRC_HXTAL_DIV32);
		return 0;
#else
		return -ENOTSUP;
#endif
	case GD32_RTC_CLK_HXTAL_DIV_RTCDIV:
#if defined(HXTAL_VALUE) && defined(RCU_RTCSRC_HXTAL_DIV_RTCDIV)
		rcu_osci_on(RCU_HXTAL);
		ret = gd32_rtc_wait_rcu_flag(RCU_FLAG_HXTALSTB, GD32_RTC_OSC_TIMEOUT_MS);
		if (ret != 0) {
			return ret;
		}

		rcu_rtc_div_config(CFG0_RTCDIV(CLAMP((uint32_t)cfg->hxtal_div, 2U, 31U)));
		rcu_rtc_clock_config(RCU_RTCSRC_HXTAL_DIV_RTCDIV);
		return 0;
#else
		return -ENOTSUP;
#endif
	default:
		return -EINVAL;
	}
}

#if defined(CONFIG_RTC_ALARM)
static int gd32_rtc_alarm_id_check(const struct gd32_rtc_config *cfg, uint16_t id)
{
	if ((cfg == NULL) || (id >= cfg->alarms_count)) {
		return -EINVAL;
	}

	return 0;
}

static uint32_t gd32_rtc_alarm_enable_mask(uint16_t id)
{
	switch (id) {
	case 0:
		return RTC_CTL_ALRM0EN;
#ifdef RTC_CTL_ALRM1EN
	case 1:
		return RTC_CTL_ALRM1EN;
#endif
	default:
		return 0U;
	}
}

static uint32_t gd32_rtc_alarm_int_mask(uint16_t id)
{
	switch (id) {
	case 0:
		return RTC_CTL_ALRM0IE;
#ifdef RTC_CTL_ALRM1IE
	case 1:
		return RTC_CTL_ALRM1IE;
#endif
	default:
		return 0U;
	}
}

static uint32_t gd32_rtc_alarm_stat_wf(uint16_t id)
{
	switch (id) {
	case 0:
		return RTC_STAT_ALRM0WF;
#ifdef RTC_STAT_ALRM1WF
	case 1:
		return RTC_STAT_ALRM1WF;
#endif
	default:
		return 0U;
	}
}

static uint32_t gd32_rtc_alarm_flag(uint16_t id)
{
	switch (id) {
	case 0:
		return RTC_STAT_ALRM0F;
#ifdef RTC_STAT_ALRM1F
	case 1:
		return RTC_STAT_ALRM1F;
#endif
	default:
		return 0U;
	}
}

static uintptr_t gd32_rtc_alarm_td_addr(uint16_t id)
{
	switch (id) {
	case 0:
		return GD32_RTC_ALRM0TD_ADDR;
#ifdef RTC_ALRM1TD
	case 1:
		return GD32_RTC_ALRM1TD_ADDR;
#endif
	default:
		return 0U;
	}
}

static uintptr_t gd32_rtc_alarm_ss_addr(uint16_t id)
{
	switch (id) {
	case 0:
		return GD32_RTC_ALRM0SS_ADDR;
#ifdef RTC_ALRM1SS
	case 1:
		return GD32_RTC_ALRM1SS_ADDR;
#endif
	default:
		return 0U;
	}
}

static int gd32_rtc_alarm_disable(uint16_t id)
{
	uint32_t enable_mask = gd32_rtc_alarm_enable_mask(id);
	uint32_t writable_flag = gd32_rtc_alarm_stat_wf(id);

	if ((enable_mask == 0U) || (writable_flag == 0U)) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_CTL &= ~enable_mask;
	gd32_rtc_wp_lock();

	irq_unlock(key);

	return gd32_rtc_wait_stat_set(writable_flag, GD32_RTC_REG_TIMEOUT_MS);
}

static uint32_t gd32_rtc_alarm_td_encode(uint16_t mask, const struct rtc_time *timeptr)
{
	uint32_t alarm_td = GD32_RTC_ALRM_SC_MASK | GD32_RTC_ALRM_MN_MASK | GD32_RTC_ALRM_HR_MASK |
			    GD32_RTC_ALRM_DAY_MASK;

	if ((mask & RTC_ALARM_TIME_MASK_WEEKDAY) != 0U) {
		alarm_td &= ~GD32_RTC_ALRM_DAY_MASK;
		alarm_td |= GD32_RTC_ALRM_WEEKDAY_SEL;
		alarm_td |= ((uint32_t)((timeptr->tm_wday == 0) ? RTC_SUNDAY : timeptr->tm_wday) &
			     0x3FU)
			    << GD32_RTC_ALRM_DAY_SHIFT;
	} else if ((mask & RTC_ALARM_TIME_MASK_MONTHDAY) != 0U) {
		alarm_td &= ~GD32_RTC_ALRM_DAY_MASK;
		alarm_td |= ((uint32_t)bin2bcd((uint8_t)timeptr->tm_mday) & 0x3FU)
			    << GD32_RTC_ALRM_DAY_SHIFT;
	}

	if ((mask & RTC_ALARM_TIME_MASK_HOUR) != 0U) {
		alarm_td &= ~GD32_RTC_ALRM_HR_MASK;
		alarm_td |= ((uint32_t)bin2bcd((uint8_t)timeptr->tm_hour) & 0x3FU)
			    << GD32_RTC_ALRM_HR_SHIFT;
	}

	if ((mask & RTC_ALARM_TIME_MASK_MINUTE) != 0U) {
		alarm_td &= ~GD32_RTC_ALRM_MN_MASK;
		alarm_td |= ((uint32_t)bin2bcd((uint8_t)timeptr->tm_min) & 0x7FU)
			    << GD32_RTC_ALRM_MN_SHIFT;
	}

	if ((mask & RTC_ALARM_TIME_MASK_SECOND) != 0U) {
		alarm_td &= ~GD32_RTC_ALRM_SC_MASK;
		alarm_td |= ((uint32_t)bin2bcd((uint8_t)timeptr->tm_sec) & 0x7FU)
			    << GD32_RTC_ALRM_SC_SHIFT;
	}

	return alarm_td;
}

static void gd32_rtc_alarm_pending_clear(struct gd32_rtc_data *data, uint16_t id)
{
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->alarm_pending[id] = false;

	k_spin_unlock(&data->lock, key);
}

static int gd32_rtc_alarm_get_supported_fields(const struct device *dev, uint16_t id,
					       uint16_t *mask)
{
	static const uint16_t supported = RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE |
					  RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_MONTHDAY |
					  RTC_ALARM_TIME_MASK_WEEKDAY;
	const struct gd32_rtc_config *cfg = dev->config;

	if ((mask == NULL) || (gd32_rtc_alarm_id_check(cfg, id) != 0)) {
		return -EINVAL;
	}

	*mask = supported;
	return 0;
}

static int gd32_rtc_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				   const struct rtc_time *timeptr)
{
	const struct gd32_rtc_config *cfg = dev->config;
	struct gd32_rtc_data *data = dev->data;
	uint16_t supported;
	uintptr_t alarm_td_addr;
	uintptr_t alarm_ss_addr;
	uint32_t alarm_td;
	int ret;

	if (gd32_rtc_alarm_id_check(cfg, id) != 0) {
		return -EINVAL;
	}

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	ret = gd32_rtc_alarm_get_supported_fields(dev, id, &supported);
	if (ret != 0) {
		return ret;
	}

	if ((mask & ~supported) != 0U) {
		return -EINVAL;
	}

	if (((mask & RTC_ALARM_TIME_MASK_WEEKDAY) != 0U) &&
	    ((mask & RTC_ALARM_TIME_MASK_MONTHDAY) != 0U)) {
		return -EINVAL;
	}

	if (mask == 0U) {
		k_mutex_lock(&data->mutex, K_FOREVER);
		ret = gd32_rtc_alarm_disable(id);
		if (ret == 0) {
			gd32_rtc_interrupt_update(gd32_rtc_alarm_int_mask(id), false);
			gd32_rtc_flag_clear(gd32_rtc_alarm_flag(id));
		}
		k_mutex_unlock(&data->mutex);

		if (ret == 0) {
			gd32_rtc_alarm_pending_clear(data, id);
		}

		return ret;
	}

	if ((timeptr == NULL) || !rtc_utils_validate_rtc_time(timeptr, mask)) {
		return -EINVAL;
	}

	if (((mask & RTC_ALARM_TIME_MASK_WEEKDAY) != 0U) &&
	    ((timeptr->tm_wday < 0) || (timeptr->tm_wday > 6))) {
		return -EINVAL;
	}

	alarm_td_addr = gd32_rtc_alarm_td_addr(id);
	alarm_ss_addr = gd32_rtc_alarm_ss_addr(id);
	if (alarm_td_addr == 0U) {
		return -EINVAL;
	}

	alarm_td = gd32_rtc_alarm_td_encode(mask, timeptr);

	k_mutex_lock(&data->mutex, K_FOREVER);

	ret = gd32_rtc_alarm_disable(id);
	if (ret == 0) {
		unsigned int key = irq_lock();

		gd32_rtc_wp_unlock();
		REG32(alarm_td_addr) = alarm_td;
		if (alarm_ss_addr != 0U) {
			REG32(alarm_ss_addr) = GD32_RTC_ALRM_SS_ALL_MASK;
		}
		gd32_rtc_wp_lock();

		irq_unlock(key);

		gd32_rtc_flag_clear(gd32_rtc_alarm_flag(id));
		gd32_rtc_interrupt_update(gd32_rtc_alarm_int_mask(id), true);
		gd32_rtc_interrupt_update(gd32_rtc_alarm_enable_mask(id), true);
	}

	k_mutex_unlock(&data->mutex);

	if (ret == 0) {
		gd32_rtc_alarm_pending_clear(data, id);
	}

	return ret;
}

static int gd32_rtc_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
				   struct rtc_time *timeptr)
{
	const struct gd32_rtc_config *cfg = dev->config;
	uint32_t enable_mask;
	uint32_t alarm_td;
	uint16_t zephyr_mask = 0U;

	if ((mask == NULL) || (timeptr == NULL) || (gd32_rtc_alarm_id_check(cfg, id) != 0)) {
		return -EINVAL;
	}

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

	enable_mask = gd32_rtc_alarm_enable_mask(id);
	if ((RTC_CTL & enable_mask) == 0U) {
		*mask = 0U;
		return 0;
	}

	alarm_td = REG32(gd32_rtc_alarm_td_addr(id));

	if ((alarm_td & GD32_RTC_ALRM_SC_MASK) == 0U) {
		zephyr_mask |= RTC_ALARM_TIME_MASK_SECOND;
		timeptr->tm_sec = bcd2bin((uint8_t)((alarm_td >> GD32_RTC_ALRM_SC_SHIFT) & 0x7FU));
	}

	if ((alarm_td & GD32_RTC_ALRM_MN_MASK) == 0U) {
		zephyr_mask |= RTC_ALARM_TIME_MASK_MINUTE;
		timeptr->tm_min = bcd2bin((uint8_t)((alarm_td >> GD32_RTC_ALRM_MN_SHIFT) & 0x7FU));
	}

	if ((alarm_td & GD32_RTC_ALRM_HR_MASK) == 0U) {
		zephyr_mask |= RTC_ALARM_TIME_MASK_HOUR;
		timeptr->tm_hour = bcd2bin((uint8_t)((alarm_td >> GD32_RTC_ALRM_HR_SHIFT) & 0x3FU));
	}

	if ((alarm_td & GD32_RTC_ALRM_DAY_MASK) == 0U) {
		uint8_t day = (uint8_t)((alarm_td >> GD32_RTC_ALRM_DAY_SHIFT) & 0x3FU);

		if ((alarm_td & GD32_RTC_ALRM_WEEKDAY_SEL) != 0U) {
			zephyr_mask |= RTC_ALARM_TIME_MASK_WEEKDAY;
			timeptr->tm_wday = (day == RTC_SUNDAY) ? 0 : day;
		} else {
			zephyr_mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
			timeptr->tm_mday = bcd2bin(day);
		}
	}

	*mask = zephyr_mask;
	return 0;
}

static int gd32_rtc_alarm_is_pending(const struct device *dev, uint16_t id)
{
	const struct gd32_rtc_config *cfg = dev->config;
	struct gd32_rtc_data *data = dev->data;
	bool pending;
	k_spinlock_key_t key;

	if (gd32_rtc_alarm_id_check(cfg, id) != 0) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	pending = data->alarm_pending[id];
	data->alarm_pending[id] = false;
	k_spin_unlock(&data->lock, key);

	return pending ? 1 : 0;
}

static int gd32_rtc_alarm_set_callback(const struct device *dev, uint16_t id,
				       rtc_alarm_callback callback, void *user_data)
{
	const struct gd32_rtc_config *cfg = dev->config;
	struct gd32_rtc_data *data = dev->data;
	k_spinlock_key_t key;

	if (gd32_rtc_alarm_id_check(cfg, id) != 0) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);
	data->alarm_cb[id] = callback;
	data->alarm_user_data[id] = user_data;
	k_spin_unlock(&data->lock, key);

	return 0;
}

static void gd32_rtc_alarm_fire(const struct device *dev, uint16_t id)
{
	struct gd32_rtc_data *data = dev->data;
	rtc_alarm_callback cb;
	void *user_data;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);
	cb = data->alarm_cb[id];
	user_data = data->alarm_user_data[id];
	if (cb == NULL) {
		data->alarm_pending[id] = true;
	}
	k_spin_unlock(&data->lock, key);

	if (cb != NULL) {
		cb(dev, id, user_data);
	}
}

static void gd32_rtc_alarm_isr(const struct device *dev)
{
	const struct gd32_rtc_config *cfg = dev->config;
	bool handled = false;

	for (uint16_t id = 0U; id < cfg->alarms_count; id++) {
		uint32_t flag = gd32_rtc_alarm_flag(id);

		if ((flag != 0U) && ((RTC_STAT & flag) != 0U)) {
			gd32_rtc_flag_clear(flag);
			gd32_rtc_alarm_fire(dev, id);
			handled = true;
		}
	}

	if (handled) {
		exti_flag_clear(cfg->alarm_exti_line);
	}
}
#endif /* CONFIG_RTC_ALARM */

#if defined(CONFIG_RTC_UPDATE)
static int gd32_rtc_wakeup_wait_wtwf(void)
{
#ifdef RTC_STAT_WTWF
	return gd32_rtc_wait_stat_set(RTC_STAT_WTWF, GD32_RTC_REG_TIMEOUT_MS);
#else
	return -ENOTSUP;
#endif
}

static int gd32_rtc_wakeup_disable(void)
{
#ifdef RTC_CTL_WTEN
	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_CTL &= ~RTC_CTL_WTEN;
	gd32_rtc_wp_lock();

	irq_unlock(key);

	return gd32_rtc_wakeup_wait_wtwf();
#else
	return -ENOTSUP;
#endif
}

static int gd32_rtc_wakeup_prepare_1hz(void)
{
#if defined(RTC_CTL_WTEN) && defined(RTC_CTL_WTIE) && defined(RTC_WUT) &&                    \
	defined(WAKEUP_CKSPRE)
	int ret = gd32_rtc_wakeup_disable();

	if (ret != 0) {
		return ret;
	}

	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_CTL = (RTC_CTL & ~(RTC_CTL_WTCS | RTC_CTL_WTIE)) |
		  (uint32_t)WAKEUP_CKSPRE;
	RTC_WUT = 0U;
	gd32_rtc_wp_lock();

	irq_unlock(key);

	return 0;
#else
	return -ENOTSUP;
#endif
}

static void gd32_rtc_wakeup_enable(void)
{
#if defined(RTC_CTL_WTEN) && defined(RTC_CTL_WTIE)
	unsigned int key = irq_lock();

	gd32_rtc_wp_unlock();
	RTC_CTL |= RTC_CTL_WTIE | RTC_CTL_WTEN;
	gd32_rtc_wp_lock();

	irq_unlock(key);
#endif
}

static int gd32_rtc_update_set_callback(const struct device *dev, rtc_update_callback callback,
					void *user_data)
{
	const struct gd32_rtc_config *cfg = dev->config;
	struct gd32_rtc_data *data = dev->data;
	int ret = 0;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if (!cfg->has_wakeup) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	if (callback != NULL) {
		ret = gd32_rtc_wakeup_prepare_1hz();
	} else {
		ret = gd32_rtc_wakeup_disable();
#ifdef RTC_CTL_WTIE
		if (ret == 0) {
			gd32_rtc_interrupt_update(RTC_CTL_WTIE, false);
		}
#endif
	}

	if (ret == 0) {
#ifdef RTC_STAT_WTF
		gd32_rtc_flag_clear(RTC_STAT_WTF);
#endif
		exti_flag_clear(cfg->wakeup_exti_line);

		k_spinlock_key_t key = k_spin_lock(&data->lock);

		data->update_cb = callback;
		data->update_user_data = user_data;

		k_spin_unlock(&data->lock, key);

		if (callback != NULL) {
			gd32_rtc_wakeup_enable();
		}
	}

	k_mutex_unlock(&data->mutex);

	return ret;
}

static void gd32_rtc_wakeup_isr(const struct device *dev)
{
#ifdef RTC_STAT_WTF
	const struct gd32_rtc_config *cfg = dev->config;
	struct gd32_rtc_data *data = dev->data;
	rtc_update_callback cb;
	void *user_data;
	k_spinlock_key_t key;

	if ((RTC_STAT & RTC_STAT_WTF) == 0U) {
		return;
	}

	gd32_rtc_flag_clear(RTC_STAT_WTF);
	exti_flag_clear(cfg->wakeup_exti_line);

	key = k_spin_lock(&data->lock);
	cb = data->update_cb;
	user_data = data->update_user_data;
	k_spin_unlock(&data->lock, key);

	if (cb != NULL) {
		cb(dev, user_data);
	}
#else
	ARG_UNUSED(dev);
#endif
}
#endif /* CONFIG_RTC_UPDATE */

static int gd32_rtc_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	struct gd32_rtc_data *data = dev->data;
	uint8_t hw_wday;
	int ret;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	if (gd32_rtc_validate_calendar_time(timeptr, &hw_wday) != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	ret = gd32_rtc_calendar_set(timeptr, hw_wday);
	if (ret == 0) {
		gd32_rtc_set_persisted_tm_year((uint32_t)timeptr->tm_year);
		data->time_valid = true;
	}

	k_mutex_unlock(&data->mutex);

	if (ret == 0) {
		LOG_DBG("time set: %04d-%02d-%02d %02d:%02d:%02d",
			timeptr->tm_year + TIME_UTILS_BASE_YEAR, timeptr->tm_mon + 1,
			timeptr->tm_mday, timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);
	}

	return ret;
}

static int gd32_rtc_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	struct gd32_rtc_data *data = dev->data;
	struct gd32_rtc_calendar_reading reading;
	uint32_t persisted_tm_year;
	uint32_t psc;
	uint16_t factor_s;
	uint32_t sub;
	uint8_t hw_year;
	uint8_t hw_wday;
	bool has_persisted_tm_year;
	uint32_t year_delta;
	uint32_t full_year;
	int ret;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	if (!data->time_valid) {
		k_mutex_unlock(&data->mutex);
		return -ENODATA;
	}

	ret = gd32_rtc_calendar_read(&reading);
	if (ret != 0) {
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	psc = RTC_PSC;

	factor_s = (uint16_t)GET_PSC_FACTOR_S(psc);
	sub = reading.ss & RTC_SS_SSC;
	hw_year = bcd2bin((uint8_t)GET_DATE_YR(reading.dr));
	hw_wday = (uint8_t)GET_DATE_DOW(reading.dr);
	has_persisted_tm_year = gd32_rtc_get_persisted_tm_year(&persisted_tm_year);

	if (!has_persisted_tm_year) {
		if (!gd32_rtc_legacy_has_year_been_set()) {
			data->time_valid = false;
			k_mutex_unlock(&data->mutex);
			return -ENODATA;
		}

		persisted_tm_year = (GD32_RTC_BASE_YEAR - TIME_UTILS_BASE_YEAR) + hw_year;
		gd32_rtc_set_persisted_tm_year(persisted_tm_year);
	}

	/* The hardware only stores the low two digits of the year, so software must observe the RTC
	 * at least once per 100 years to advance the persisted full-year state across century
	 * rollover.
	 */
	year_delta = (hw_year + 100U - (persisted_tm_year % 100U)) % 100U;
	full_year = persisted_tm_year + year_delta;

	if (full_year > (uint32_t)INT_MAX) {
		k_mutex_unlock(&data->mutex);
		return -ERANGE;
	}

	if (full_year != persisted_tm_year) {
		gd32_rtc_set_persisted_tm_year(full_year);
	}

	timeptr->tm_sec = bcd2bin((uint8_t)GET_TIME_SC(reading.tr));
	timeptr->tm_min = bcd2bin((uint8_t)GET_TIME_MN(reading.tr));
	timeptr->tm_hour = bcd2bin((uint8_t)GET_TIME_HR(reading.tr));
	timeptr->tm_mday = bcd2bin((uint8_t)GET_DATE_DAY(reading.dr));
	timeptr->tm_mon = bcd2bin((uint8_t)GET_DATE_MON(reading.dr)) - 1;
	timeptr->tm_year = (int)full_year;
	ret = gd32_rtc_hw_to_zephyr_wday(hw_wday, &timeptr->tm_wday);
	if (ret != 0) {
		timeptr->tm_wday = gd32_rtc_weekday_compute(
			(uint32_t)timeptr->tm_year + TIME_UTILS_BASE_YEAR,
			(uint8_t)timeptr->tm_mon + 1U, (uint8_t)timeptr->tm_mday);
	}
	timeptr->tm_yday = -1;
	timeptr->tm_isdst = -1;

	if (factor_s > 0U) {
		uint64_t num = (uint64_t)(factor_s - sub) * 1000000000ULL;

		timeptr->tm_nsec = (int)(num / (uint64_t)(factor_s + 1U));
	} else {
		timeptr->tm_nsec = 0;
	}

	k_mutex_unlock(&data->mutex);
	return 0;
}

static DEVICE_API(rtc, gd32_rtc_driver_api) = {
	.set_time = gd32_rtc_set_time,
	.get_time = gd32_rtc_get_time,
#if defined(CONFIG_RTC_ALARM)
	.alarm_get_supported_fields = gd32_rtc_alarm_get_supported_fields,
	.alarm_set_time = gd32_rtc_alarm_set_time,
	.alarm_get_time = gd32_rtc_alarm_get_time,
	.alarm_is_pending = gd32_rtc_alarm_is_pending,
	.alarm_set_callback = gd32_rtc_alarm_set_callback,
#endif
#if defined(CONFIG_RTC_UPDATE)
	.update_set_callback = gd32_rtc_update_set_callback,
#endif
};

static int gd32_rtc_common_init(const struct device *dev)
{
	const struct gd32_rtc_config *cfg = dev->config;
	struct gd32_rtc_data *data = dev->data;
	enum gd32_rtc_clk_sel clk_sel;
	uint32_t clk_hz;
	uint32_t rcu_src;
	uint16_t presc_a;
	uint16_t presc_s;
	uint32_t persisted_tm_year;
	int ret;

	k_mutex_init(&data->mutex);
	data->time_valid = false;

	ret = gd32_rtc_clock_source_parse(cfg, &clk_sel);
	if (ret != 0) {
		LOG_ERR("unsupported RTC clock source \"%s\"", cfg->clock_source);
		return ret;
	}

	rcu_periph_clock_enable(RCU_PMU);
#ifdef RCU_BKP
	if (cfg->needs_bkp_clk) {
		rcu_periph_clock_enable(RCU_BKP);
	}
#endif
	pmu_backup_write_enable();

	if (!gd32_rtc_is_already_configured()) {
		clk_hz = gd32_rtc_clk_hz(cfg, clk_sel);
		if (clk_hz == 0U) {
			LOG_ERR("unsupported RTC input clock");
			return -ENOTSUP;
		}

		ret = gd32_rtc_clock_configure(cfg, clk_sel);
		if (ret != 0) {
			LOG_ERR("RTC clock source failed to start (%d)", ret);
			return ret;
		}

		rcu_periph_clock_enable(RCU_RTC);
		gd32_rtc_tamper_bkp_preserve_enable();

		ret = gd32_rtc_register_sync_wait();
		if (ret != 0) {
			LOG_ERR("RTC register sync timeout (%d)", ret);
			return ret;
		}

		ret = gd32_rtc_prescaler_compute(clk_hz, &presc_a, &presc_s);
		if (ret != 0) {
			LOG_ERR("cannot derive prescalers for rtc_clk=%u", clk_hz);
			return ret;
		}

		ret = gd32_rtc_prescaler_apply(presc_a, presc_s);
		if (ret != 0) {
			LOG_ERR("RTC prescaler apply failed (%d)", ret);
			return ret;
		}

		LOG_INF("configured RTC (clk=%u Hz, presc_a=0x%x presc_s=0x%x)", clk_hz, presc_a,
			presc_s);
	} else {
		rcu_periph_clock_enable(RCU_RTC);
		gd32_rtc_tamper_bkp_preserve_enable();

		ret = gd32_rtc_register_sync_wait();
		if (ret != 0) {
			LOG_WRN("RTC register sync timeout (%d)", ret);
		}

		rcu_src = gd32_rtc_clk_sel_to_rcu_src(clk_sel);
		if ((rcu_src != RCU_RTCSRC_NONE) && ((RCU_BDCTL & RCU_BDCTL_RTCSRC) != rcu_src)) {
			LOG_WRN("RTC clock source differs from devicetree; preserving existing");
		}
	}

	if (gd32_rtc_get_persisted_tm_year(&persisted_tm_year) ||
	    gd32_rtc_legacy_has_year_been_set()) {
		data->time_valid = true;
	}

	return 0;
}

#define GD32_RTC_CONFIG_INIT(inst)                                                                 \
	static const struct gd32_rtc_config gd32_rtc_cfg_##inst = {                                \
		.clock_source = DT_INST_PROP(inst, clock_source),                                  \
		.alarms_count = DT_INST_PROP(inst, alarms_count),                                  \
		.hxtal_div = (uint8_t)DT_INST_PROP_OR(inst, hxtal_div, 2),                         \
		.alarm_exti_line = GD32_RTC_SOC_ALARM_EXTI_LINE,                                   \
		.wakeup_exti_line = GD32_RTC_SOC_WAKEUP_EXTI_LINE,                                 \
		.lxtal_drive_high = DT_INST_PROP_OR(inst, lxtal_drive_high, 0),                    \
		.needs_bkp_clk = GD32_RTC_SOC_NEEDS_BKP_CLK,                                       \
		.has_wakeup = DT_INST_IRQ_HAS_NAME(inst, wakeup),                                  \
	};                                                                                         \
	static struct gd32_rtc_data gd32_rtc_data_##inst

#define GD32_RTC_IRQ_INIT(inst)                                                                    \
	do {                                                                                       \
		IF_ENABLED(CONFIG_RTC_ALARM,                                                 \
			   (COND_CODE_1(DT_INST_IRQ_HAS_NAME(inst, alarm),                  \
					(IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, alarm, irq),   \
						 DT_INST_IRQ_BY_NAME(inst, alarm, priority), \
						 gd32_rtc_alarm_isr, DEVICE_DT_INST_GET(inst), \
						 0);                                         \
					 irq_enable(DT_INST_IRQ_BY_NAME(inst, alarm, irq));  \
					 exti_flag_clear(gd32_rtc_cfg_## inst.alarm_exti_line); \
					 exti_init(gd32_rtc_cfg_## inst.alarm_exti_line,        \
						   EXTI_INTERRUPT, EXTI_TRIG_RISING);),     \
					())))     \
		IF_ENABLED(CONFIG_RTC_UPDATE,                                                \
			   (COND_CODE_1(DT_INST_IRQ_HAS_NAME(inst, wakeup),                 \
					(IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, wakeup, irq),  \
						 DT_INST_IRQ_BY_NAME(inst, wakeup, priority), \
						 gd32_rtc_wakeup_isr, DEVICE_DT_INST_GET(inst), \
						 0);                                          \
					 irq_enable(DT_INST_IRQ_BY_NAME(inst, wakeup, irq)); \
					 exti_flag_clear(gd32_rtc_cfg_## inst.wakeup_exti_line); \
					 exti_init(gd32_rtc_cfg_## inst.wakeup_exti_line,       \
						   EXTI_INTERRUPT, EXTI_TRIG_RISING);),    \
					())))     \
	} while (false)

#define GD32_RTC_INIT(inst)                                                                        \
	GD32_RTC_CONFIG_INIT(inst);                                                                \
	static int gd32_rtc_init_##inst(const struct device *dev)                                  \
	{                                                                                          \
		int ret = gd32_rtc_common_init(dev);                                               \
		if (ret != 0) {                                                                    \
			return ret;                                                                \
		}                                                                                  \
		GD32_RTC_IRQ_INIT(inst);                                                           \
		return 0;                                                                          \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(inst, gd32_rtc_init_##inst, NULL, &gd32_rtc_data_##inst,             \
			      &gd32_rtc_cfg_##inst, POST_KERNEL, CONFIG_RTC_INIT_PRIORITY,         \
			      &gd32_rtc_driver_api)

DT_INST_FOREACH_STATUS_OKAY(GD32_RTC_INIT)
