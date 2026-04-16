/*
 * Copyright (c) 2026, Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_rtc

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/sys/util.h>

#if defined(RTC_CLOCKS)
#include <fsl_clock.h>
#endif
#include <fsl_rtc.h>

#include "rtc_utils.h"

LOG_MODULE_REGISTER(rtc_mcux_rtc, CONFIG_RTC_LOG_LEVEL);

#define MCUX_RTC_YEAR_MIN 1970

#define MCUX_RTC_TIME_FIELDS                                                                      \
	(RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |   \
	 RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_YEAR)

/*
 * FSL_FEATURE_* may be defined with parentheses which cannot be used with IS_ENABLED().
 */
#if defined(FSL_FEATURE_RTC_HAS_LPO_ADJUST) && FSL_FEATURE_RTC_HAS_LPO_ADJUST
#define MCUX_RTC_HAS_LPO_ADJUST 1
#else
#define MCUX_RTC_HAS_LPO_ADJUST 0
#endif

struct rtc_mcux_config {
	RTC_Type *base;
	void (*irq_config_func)(const struct device *dev);
	uint8_t clock_source;
};

struct rtc_mcux_data {
	struct k_spinlock lock;
	bool time_valid;
};

static bool rtc_mcux_is_leap_year(uint32_t year)
{
	return ((year % 400U) == 0U) || (((year % 4U) == 0U) && ((year % 100U) != 0U));
}

static uint8_t rtc_mcux_days_in_month(uint32_t year, uint8_t month)
{
	static const uint8_t days_per_month[] = {
		31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U,
	};

	uint8_t days = days_per_month[month - 1U];

	if ((month == 2U) && rtc_mcux_is_leap_year(year)) {
		days++;
	}

	return days;
}

static bool rtc_mcux_time_to_seconds(const struct rtc_time *src, uint32_t *seconds)
{
	struct tm tm = {0};
	int64_t seconds64;
	uint32_t year;
	uint8_t month;

	if ((src == NULL) || (seconds == NULL) ||
	    !rtc_utils_validate_rtc_time(src, MCUX_RTC_TIME_FIELDS)) {
		return false;
	}

	year = (uint32_t)src->tm_year + TIME_UTILS_BASE_YEAR;
	if (year < MCUX_RTC_YEAR_MIN) {
		return false;
	}

	month = (uint8_t)src->tm_mon + 1U;
	if (src->tm_mday > rtc_mcux_days_in_month(year, month)) {
		return false;
	}

	tm.tm_sec = src->tm_sec;
	tm.tm_min = src->tm_min;
	tm.tm_hour = src->tm_hour;
	tm.tm_mday = src->tm_mday;
	tm.tm_mon = src->tm_mon;
	tm.tm_year = src->tm_year;
	tm.tm_isdst = -1;

	seconds64 = timeutil_timegm64(&tm);
	if ((seconds64 < 0) || ((uint64_t)seconds64 > UINT32_MAX)) {
		return false;
	}

	*seconds = (uint32_t)seconds64;

	return true;
}

static int rtc_mcux_time_from_seconds(uint32_t seconds, struct rtc_time *dst)
{
	struct tm tm = {0};
	time_t timestamp = (time_t)seconds;

	if ((dst == NULL) || (gmtime_r(&timestamp, &tm) == NULL)) {
		return -EINVAL;
	}

	dst->tm_sec = tm.tm_sec;
	dst->tm_min = tm.tm_min;
	dst->tm_hour = tm.tm_hour;
	dst->tm_mday = tm.tm_mday;
	dst->tm_mon = tm.tm_mon;
	dst->tm_year = tm.tm_year;
	dst->tm_wday = tm.tm_wday;
	dst->tm_yday = tm.tm_yday;
	dst->tm_isdst = -1;
	dst->tm_nsec = 0;

	return 0;
}

static uint32_t rtc_mcux_read_seconds(RTC_Type *base)
{
	uint32_t seconds = base->TSR;

	/*
	 * The reference manual recommends verifying the same value is returned by two consecutive
	 * reads in case TSR is sampled while it is incrementing.
	 */
	if (base->TSR == seconds) {
		return seconds;
	}

	return base->TSR;
}

static bool rtc_mcux_is_running(RTC_Type *base)
{
	return (base->SR & RTC_SR_TCE_MASK) != 0U;
}

static void rtc_mcux_clear_time_flags(RTC_Type *base)
{
	bool restart = rtc_mcux_is_running(base);
	uint32_t flags = RTC_GetStatusFlags(base);

	if ((flags & ((uint32_t)kRTC_TimeInvalidFlag | (uint32_t)kRTC_TimeOverflowFlag)) == 0U) {
		return;
	}

	if (restart) {
		RTC_StopTimer(base);
	}

	RTC_ClearStatusFlags(base, flags & ((uint32_t)kRTC_TimeInvalidFlag |
					    (uint32_t)kRTC_TimeOverflowFlag));

	if (restart) {
		RTC_StartTimer(base);
	}
}

static int rtc_mcux_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	const struct rtc_mcux_config *config = dev->config;
	struct rtc_mcux_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t seconds;

	if (!rtc_mcux_time_to_seconds(timeptr, &seconds)) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	if (rtc_mcux_is_running(config->base)) {
		RTC_StopTimer(config->base);
	}

	config->base->TPR = 0U;
	config->base->TSR = seconds;
	RTC_StartTimer(config->base);
	data->time_valid = true;

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int rtc_mcux_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	const struct rtc_mcux_config *config = dev->config;
	struct rtc_mcux_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t seconds;
	uint32_t flags;
	int ret;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	flags = RTC_GetStatusFlags(config->base);
	if (!data->time_valid ||
	    ((flags & ((uint32_t)kRTC_TimeInvalidFlag | (uint32_t)kRTC_TimeOverflowFlag)) != 0U)) {
		data->time_valid = false;
		k_spin_unlock(&data->lock, key);
		return -ENODATA;
	}

	seconds = rtc_mcux_read_seconds(config->base);
	ret = rtc_mcux_time_from_seconds(seconds, timeptr);

	k_spin_unlock(&data->lock, key);

	return ret;
}

static int rtc_mcux_init(const struct device *dev)
{
	const struct rtc_mcux_config *config = dev->config;
	struct rtc_mcux_data *data = dev->data;
	rtc_config_t rtc_config;
	uint32_t flags;

#if defined(RTC_CLOCKS)
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
	CLOCK_EnableClock(kCLOCK_Rtc0);
#endif
#endif

	flags = RTC_GetStatusFlags(config->base);
	RTC_GetDefaultConfig(&rtc_config);
	RTC_Init(config->base, &rtc_config);
	/*
	 * Preserve whether the retained time was valid before RTC_Init(), which resets the block and
	 * clears TIF after VBAT POR.
	 */
	data->time_valid = (flags & ((uint32_t)kRTC_TimeInvalidFlag |
				     (uint32_t)kRTC_TimeOverflowFlag)) == 0U;

	RTC_DisableInterrupts(config->base,
			      (uint32_t)kRTC_AlarmInterruptEnable |
				      (uint32_t)kRTC_SecondsInterruptEnable);
	RTC_EnableInterrupts(config->base,
			     (uint32_t)kRTC_TimeInvalidInterruptEnable |
				     (uint32_t)kRTC_TimeOverflowInterruptEnable);

#if defined(FSL_FEATURE_RTC_HAS_TSIC) && FSL_FEATURE_RTC_HAS_TSIC
	config->base->IER &= ~RTC_IER_TSIC_MASK;
#endif

#if defined(FSL_FEATURE_RTC_HAS_LPO_ADJUST) && FSL_FEATURE_RTC_HAS_LPO_ADJUST
	RTC_EnableLPOClock(config->base, config->clock_source);
#endif

#if !(defined(FSL_FEATURE_RTC_HAS_NO_CR_OSCE) && FSL_FEATURE_RTC_HAS_NO_CR_OSCE)
	RTC_SetClockSource(config->base);
	k_busy_wait(USEC_PER_MSEC);
#endif

	if (!rtc_mcux_is_running(config->base)) {
		RTC_StartTimer(config->base);
	}

	config->irq_config_func(dev);

	return 0;
}

static void rtc_mcux_isr(const struct device *dev)
{
	const struct rtc_mcux_config *config = dev->config;
	struct rtc_mcux_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t flags;

	key = k_spin_lock(&data->lock);

	flags = RTC_GetStatusFlags(config->base);
	if ((flags & ((uint32_t)kRTC_TimeInvalidFlag | (uint32_t)kRTC_TimeOverflowFlag)) != 0U) {
		data->time_valid = false;
		rtc_mcux_clear_time_flags(config->base);
	}

	k_spin_unlock(&data->lock, key);
}

static DEVICE_API(rtc, rtc_mcux_driver_api) = {
	.set_time = rtc_mcux_set_time,
	.get_time = rtc_mcux_get_time,
};

#define RTC_MCUX_IRQ_CONNECT(inst, idx)                                                            \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, idx, irq), DT_INST_IRQ_BY_IDX(inst, idx, priority), \
		    rtc_mcux_isr, DEVICE_DT_INST_GET(inst), 0)

#define RTC_MCUX_IRQ_CONFIG(inst)                                                                  \
	static void rtc_mcux_irq_config_##inst(const struct device *dev)                          \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		RTC_MCUX_IRQ_CONNECT(inst, 0);                                                    \
		irq_enable(DT_INST_IRQ_BY_IDX(inst, 0, irq));                                     \
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(inst, 1),                                          \
			   (RTC_MCUX_IRQ_CONNECT(inst, 1);                                        \
			    irq_enable(DT_INST_IRQ_BY_IDX(inst, 1, irq));))                       \
	}

#define RTC_MCUX_DEVICE(inst)                                                                      \
	BUILD_ASSERT((((DT_INST_ENUM_IDX(inst, clock_source) == 1) && MCUX_RTC_HAS_LPO_ADJUST) || \
		      (DT_INST_ENUM_IDX(inst, clock_source) == 0)),                            \
		     "Cannot choose the LPO clock for that instance of the RTC");                  \
	RTC_MCUX_IRQ_CONFIG(inst);                                                                 \
	static const struct rtc_mcux_config rtc_mcux_config_##inst = {                            \
		.base = (RTC_Type *)DT_INST_REG_ADDR(inst),                                       \
		.clock_source = DT_INST_ENUM_IDX(inst, clock_source),                             \
		.irq_config_func = rtc_mcux_irq_config_##inst,                                    \
	};                                                                                         \
	static struct rtc_mcux_data rtc_mcux_data_##inst;                                         \
	DEVICE_DT_INST_DEFINE(inst, rtc_mcux_init, NULL, &rtc_mcux_data_##inst,                   \
			      &rtc_mcux_config_##inst, POST_KERNEL, CONFIG_RTC_INIT_PRIORITY,    \
			      &rtc_mcux_driver_api)

DT_INST_FOREACH_STATUS_OKAY(RTC_MCUX_DEVICE)
