// clang-format off
/*
 * Copyright (c) 2022 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_adc

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <gd32_adc.h>
#include <gd32_rcu.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_gd32, CONFIG_ADC_LOG_LEVEL);

/*
 * Some GD32 SoCs share ADC interrupts across instances; use node labels to bind
 * the shared IRQ handler to the active ADC instance(s).
 */
#define ADC0_NODE		DT_NODELABEL(adc0)
#define ADC1_NODE		DT_NODELABEL(adc1)
#define ADC2_NODE		DT_NODELABEL(adc2)

#define ADC0_ENABLE		DT_NODE_HAS_STATUS_OKAY(ADC0_NODE)
#define ADC1_ENABLE		DT_NODE_HAS_STATUS_OKAY(ADC1_NODE)
#define ADC2_ENABLE		DT_NODE_HAS_STATUS_OKAY(ADC2_NODE)

#ifndef	ADC0
/**
 * @brief The name of gd32 ADC HAL are different between single and multi ADC SoCs.
 * This adjust the single ADC SoC HAL, so we can call gd32 ADC HAL in a common way.
 */
#undef ADC_STAT
#undef ADC_CTL0
#undef ADC_CTL1
#undef ADC_SAMPT0
#undef ADC_SAMPT1
#undef ADC_RSQ0
#undef ADC_RSQ2
#undef ADC_RDATA

#define ADC_STAT(adc0)		REG32((adc0) + 0x00000000U)
#define ADC_CTL0(adc0)		REG32((adc0) + 0x00000004U)
#define ADC_CTL1(adc0)		REG32((adc0) + 0x00000008U)
#define ADC_SAMPT0(adc0)	REG32((adc0) + 0x0000000CU)
#define ADC_SAMPT1(adc0)	REG32((adc0) + 0x00000010U)
#define ADC_RSQ0(adc0)		REG32((adc0) + 0x0000002CU)
#define ADC_RSQ2(adc0)		REG32((adc0) + 0x00000034U)
#define ADC_RDATA(adc0)		REG32((adc0) + 0x0000004CU)
#endif

#define SPT_WIDTH	3U
#define SAMPT1_SIZE	10U

#if defined(CONFIG_SOC_SERIES_GD32F4XX)
#define SMP_TIME(x)	ADC_SAMPLETIME_##x

static const uint16_t acq_time_tbl[8] = {3, 15, 28, 56, 84, 112, 144, 480};
static const uint32_t table_samp_time[] = {
	SMP_TIME(3),
	SMP_TIME(15),
	SMP_TIME(28),
	SMP_TIME(56),
	SMP_TIME(84),
	SMP_TIME(112),
	SMP_TIME(144),
	SMP_TIME(480)
};
#elif defined(CONFIG_SOC_SERIES_GD32L23X)
#define SMP_TIME(x)	ADC_SAMPLETIME_##x##POINT5

static const uint16_t acq_time_tbl[8] = {3, 8, 14, 29, 42, 56, 72, 240};
static const uint32_t table_samp_time[] = {
	SMP_TIME(2),
	SMP_TIME(7),
	SMP_TIME(13),
	SMP_TIME(28),
	SMP_TIME(41),
	SMP_TIME(55),
	SMP_TIME(71),
	SMP_TIME(239),
};
#elif defined(CONFIG_SOC_SERIES_GD32A50X)
#define SMP_TIME(x)	ADC_SAMPLETIME_##x##POINT5

static const uint16_t acq_time_tbl[8] = {3, 15, 28, 56, 84, 112, 144, 480};
static const uint32_t table_samp_time[] = {
	SMP_TIME(2),
	SMP_TIME(14),
	SMP_TIME(27),
	SMP_TIME(55),
	SMP_TIME(83),
	SMP_TIME(111),
	SMP_TIME(143),
	SMP_TIME(479)
};
#else
#define SMP_TIME(x)	ADC_SAMPLETIME_##x##POINT5

static const uint16_t acq_time_tbl[8] = {2, 8, 14, 29, 42, 56, 72, 240};
static const uint32_t table_samp_time[] = {
	SMP_TIME(1),
	SMP_TIME(7),
	SMP_TIME(13),
	SMP_TIME(28),
	SMP_TIME(41),
	SMP_TIME(55),
	SMP_TIME(71),
	SMP_TIME(239)
};
#endif

struct adc_gd32_config {
	uint32_t reg;
#ifdef CONFIG_SOC_SERIES_GD32F3X0
	uint32_t rcu_clock_source;
#endif
	uint16_t clkid;
	struct reset_dt_spec reset;
	uint8_t channels;
	const struct pinctrl_dev_config *pcfg;
	uint8_t irq_num;
	void (*irq_config_func)(void);
};

struct adc_gd32_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint8_t resolution;
	uint8_t data_shift;
	uint32_t syncctl_enable_mask;
};

static void adc_gd32_isr(const struct device *dev)
{
	struct adc_gd32_data *data = dev->data;
	const struct adc_gd32_config *cfg = dev->config;

	if (ADC_STAT(cfg->reg) & ADC_STAT_EOC) {
		uint16_t sample = (uint16_t)(ADC_RDATA(cfg->reg) & ADC_RDATA_RDATA);

		if (data->data_shift != 0U) {
			sample >>= data->data_shift;
		}

		*data->buffer++ = sample;

		/* Disable EOC interrupt. */
		ADC_CTL0(cfg->reg) &= ~ADC_CTL0_EOCIE;
		/* Clear EOC bit. */
		ADC_STAT(cfg->reg) &= ~ADC_STAT_EOC;

		adc_context_on_sampling_done(&data->ctx, dev);
	}
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_gd32_data *data = CONTAINER_OF(ctx, struct adc_gd32_data, ctx);
	const struct device *dev = data->dev;
	const struct adc_gd32_config *cfg = dev->config;

	data->repeat_buffer = data->buffer;

#if defined(ADC_SYNCCTL)
	if (data->syncctl_enable_mask != 0U) {
		ADC_SYNCCTL |= data->syncctl_enable_mask;
	}
#endif

	/* Ensure stale flags won't stall conversion. */
	ADC_STAT(cfg->reg) &= ~(ADC_STAT_EOC | ADC_STAT_ROVF);

	/* Enable EOC interrupt */
	ADC_CTL0(cfg->reg) |= ADC_CTL0_EOCIE;

	/* Set ADC software conversion trigger. */
	ADC_CTL1(cfg->reg) |= ADC_CTL1_SWRCST;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct adc_gd32_data *data = CONTAINER_OF(ctx, struct adc_gd32_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	ARG_UNUSED(status);

	struct adc_gd32_data *data = CONTAINER_OF(ctx, struct adc_gd32_data, ctx);

#if defined(ADC_SYNCCTL)
	if (data->syncctl_enable_mask != 0U) {
		ADC_SYNCCTL &= ~data->syncctl_enable_mask;
		data->syncctl_enable_mask = 0U;
	}
#endif
}

static int adc_gd32_wait_mask_clear(volatile uint32_t *reg, uint32_t mask, uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	while ((*reg & mask) != 0U) {
		if (k_uptime_get() > deadline) {
			return -ETIMEDOUT;
		}
		k_usleep(10);
	}

	return 0;
}

static int adc_gd32_calibration(const struct adc_gd32_config *cfg)
{
	/*
	 * ADC needs a short stabilization time after being enabled before a
	 * calibration cycle is started.
	 */
	k_usleep(10);

	ADC_CTL1(cfg->reg) |= ADC_CTL1_RSTCLB;
	if (adc_gd32_wait_mask_clear(&ADC_CTL1(cfg->reg), ADC_CTL1_RSTCLB, 10U) != 0) {
		return -ETIMEDOUT;
	}

	ADC_CTL1(cfg->reg) |= ADC_CTL1_CLB;
	if (adc_gd32_wait_mask_clear(&ADC_CTL1(cfg->reg), ADC_CTL1_CLB, 10U) != 0) {
		return -ETIMEDOUT;
	}

	return 0;
}

static int adc_gd32_set_resolution(const struct adc_gd32_config *cfg, uint8_t resolution_id)
{
	/* DRES must only be changed when ADCON is reset (see reference manual). */
	ADC_CTL0(cfg->reg) &= ~ADC_CTL0_EOCIE;
	ADC_CTL1(cfg->reg) &= ~ADC_CTL1_ADCON;

#if defined(CONFIG_SOC_SERIES_GD32F4XX) || \
	defined(CONFIG_SOC_SERIES_GD32F3X0) || \
	defined(CONFIG_SOC_SERIES_GD32L23X)
	ADC_CTL0(cfg->reg) &= ~ADC_CTL0_DRES;
	ADC_CTL0(cfg->reg) |= CTL0_DRES(resolution_id);
#elif defined(CONFIG_SOC_SERIES_GD32F403) || \
	defined(CONFIG_SOC_SERIES_GD32A50X)
	ADC_OVSAMPCTL(cfg->reg) &= ~ADC_OVSAMPCTL_DRES;
	ADC_OVSAMPCTL(cfg->reg) |= OVSAMPCTL_DRES(resolution_id);
#elif defined(CONFIG_SOC_SERIES_GD32VF103)
	ADC_OVSCR(cfg->reg) &= ~ADC_OVSCR_DRES;
	ADC_OVSCR(cfg->reg) |= OVSCR_DRES(resolution_id);
#else
	ADC_CTL1(cfg->reg) |= ADC_CTL1_ADCON;
	return -ENOTSUP;
#endif

	ADC_CTL1(cfg->reg) |= ADC_CTL1_ADCON;

	return adc_gd32_calibration(cfg);
}

static int adc_gd32_configure_sampt(const struct adc_gd32_config *cfg,
				    uint8_t channel, uint16_t acq_time)
{
	uint8_t index = 0, offset;

	if (acq_time != ADC_ACQ_TIME_DEFAULT) {
		/* Acquisition time unit is adc clock cycle. */
		if (ADC_ACQ_TIME_UNIT(acq_time) != ADC_ACQ_TIME_TICKS) {
			return -EINVAL;
		}

		for ( ; index < ARRAY_SIZE(acq_time_tbl); index++) {
			if (ADC_ACQ_TIME_VALUE(acq_time) <= acq_time_tbl[index]) {
				break;
			}
		}

		if (index == ARRAY_SIZE(acq_time_tbl) ||
		    ADC_ACQ_TIME_VALUE(acq_time) != acq_time_tbl[index]) {
			return -ENOTSUP;
		}
	}

	if (channel < SAMPT1_SIZE) {
		offset = SPT_WIDTH * channel;
		ADC_SAMPT1(cfg->reg) &= ~(ADC_SAMPTX_SPTN << offset);
		ADC_SAMPT1(cfg->reg) |= table_samp_time[index] << offset;
	} else {
		offset = SPT_WIDTH * (channel - SAMPT1_SIZE);
		ADC_SAMPT0(cfg->reg) &= ~(ADC_SAMPTX_SPTN << offset);
		ADC_SAMPT0(cfg->reg) |= table_samp_time[index] << offset;
	}

	return 0;
}

static int adc_gd32_channel_setup(const struct device *dev,
				  const struct adc_channel_cfg *chan_cfg)
{
	const struct adc_gd32_config *cfg = dev->config;

	if (chan_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Gain is not valid");
		return -ENOTSUP;
	}

	/*
	 * GD32 ADC conversions are referenced to VREF+/VDDA and the reference is not
	 * selectable in hardware. Keep supporting ADC_REF_INTERNAL for backwards compatibility.
	 */
	if (chan_cfg->reference != ADC_REF_INTERNAL &&
	    chan_cfg->reference != ADC_REF_VDD_1) {
		LOG_ERR("Reference is not valid");
		return -ENOTSUP;
	}

	if (chan_cfg->differential) {
		LOG_ERR("Differential sampling not supported");
		return -ENOTSUP;
	}

	if (chan_cfg->channel_id >= cfg->channels) {
#if defined(ADC_SYNCCTL) && defined(ADC0)
		if ((cfg->reg == ADC0) &&
		    (chan_cfg->channel_id >= 16U) &&
		    (chan_cfg->channel_id <= 18U)) {
			/* Internal channels. */
		} else
#endif
		{
		LOG_ERR("Invalid channel (%u)", chan_cfg->channel_id);
		return -EINVAL;
		}
	}

	return adc_gd32_configure_sampt(cfg, chan_cfg->channel_id,
					chan_cfg->acquisition_time);
}

static int adc_gd32_start_read(const struct device *dev,
			       const struct adc_sequence *sequence)
{
	struct adc_gd32_data *data = dev->data;
	const struct adc_gd32_config *cfg = dev->config;
	uint8_t resolution_id;
	uint32_t index;
	int ret;
	size_t needed_buffer_size = sizeof(uint16_t);

	if (sequence->options != NULL) {
		needed_buffer_size *= (1U + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_buffer_size) {
		return -ENOMEM;
	}

	if (sequence->oversampling != 0U) {
		return -ENOTSUP;
	}

	if (sequence->channels == 0U) {
		return 0;
	}

	index = find_lsb_set(sequence->channels) - 1;
	if (sequence->channels > BIT(index)) {
		LOG_ERR("Only single channel supported");
		return -ENOTSUP;
	}

	data->syncctl_enable_mask = 0U;

#if defined(ADC_SYNCCTL) && defined(ADC0)
	/*
	 * On GD32F4xx ADC0 channels 16/17/18 are internally connected; ADC1/ADC2
	 * see VSSA on these channels.
	 */
	if ((index >= 16U) && (cfg->reg != ADC0)) {
		return -ENOTSUP;
	}
#endif

#if defined(ADC_SYNCCTL_TSVREN)
	if ((index == 16U) || (index == 17U)) {
		data->syncctl_enable_mask |= ADC_SYNCCTL_TSVREN;
	}
#endif

#if defined(ADC_SYNCCTL_VBATEN)
	if (index == 18U) {
		data->syncctl_enable_mask |= ADC_SYNCCTL_VBATEN;
	}
#endif

	/* Force a single conversion in the regular group (RL=0 => 1 conversion). */
	ADC_RSQ0(cfg->reg) &= ~ADC_RSQ0_RL;

	/* Select channel as rank 0 in the regular group. */
	ADC_RSQ2(cfg->reg) &= ~ADC_RSQX_RSQN;
	ADC_RSQ2(cfg->reg) |= (uint32_t)index;

	switch (sequence->resolution) {
	case 12U:
		resolution_id = 0U;
		break;
	case 10U:
		resolution_id = 1U;
		break;
	case 8U:
		resolution_id = 2U;
		break;
	case 6U:
		resolution_id = 3U;
		break;
	default:
		return -EINVAL;
	}

	if (data->resolution != sequence->resolution) {
		ret = adc_gd32_set_resolution(cfg, resolution_id);
		if (ret < 0) {
			return ret;
		}
		data->resolution = sequence->resolution;
	}

	data->data_shift = 12U - data->resolution;

	if (sequence->calibrate) {
		ret = adc_gd32_calibration(cfg);
		if (ret < 0) {
			return ret;
		}
	}
	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_gd32_read(const struct device *dev,
			 const struct adc_sequence *sequence)
{
	struct adc_gd32_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = adc_gd32_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_gd32_read_async(const struct device *dev,
			       const struct adc_sequence *sequence,
			       struct k_poll_signal *async)
{
	struct adc_gd32_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, true, async);
	error = adc_gd32_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif /* CONFIG_ADC_ASYNC */

#ifdef CONFIG_ADC_ASYNC
#define ADC_GD32_API_ASYNC_FIELD .read_async = adc_gd32_read_async,
#else
#define ADC_GD32_API_ASYNC_FIELD
#endif

static int adc_gd32_init(const struct device *dev)
{
	struct adc_gd32_data *data = dev->data;
	const struct adc_gd32_config *cfg = dev->config;
	int ret;

	data->dev = dev;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_SOC_SERIES_GD32F3X0
	/* Select adc clock source and its prescaler. */
	rcu_adc_clock_config(cfg->rcu_clock_source);
#endif

	(void)clock_control_on(GD32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&cfg->clkid);

	(void)reset_line_toggle_dt(&cfg->reset);

#if defined(CONFIG_SOC_SERIES_GD32F403) || \
	defined(CONFIG_SOC_SERIES_GD32VF103) || \
	defined(CONFIG_SOC_SERIES_GD32F3X0) || \
	defined(CONFIG_SOC_SERIES_GD32L23X)
	/* Set SWRCST as the regular channel external trigger. */
	ADC_CTL1(cfg->reg) &= ~ADC_CTL1_ETSRC;
	ADC_CTL1(cfg->reg) |= CTL1_ETSRC(7);

	/* Enable external trigger for regular channel. */
	ADC_CTL1(cfg->reg) |= ADC_CTL1_ETERC;
#endif

#ifdef CONFIG_SOC_SERIES_GD32A50X
	ADC_CTL1(cfg->reg) |= ADC_CTL1_ETSRC;
	ADC_CTL1(cfg->reg) |= ADC_CTL1_ETERC;
#endif

	/* Enable ADC */
	ADC_CTL1(cfg->reg) |= ADC_CTL1_ADCON;

	/* Default to a single regular conversion and right-aligned data. */
	ADC_RSQ0(cfg->reg) &= ~ADC_RSQ0_RL;
	ADC_CTL1(cfg->reg) &= ~ADC_CTL1_DAL;
	ADC_CTL0(cfg->reg) &= ~ADC_CTL0_SM;

	data->resolution = 12U;
	data->data_shift = 0U;
	data->syncctl_enable_mask = 0U;

	ret = adc_gd32_calibration(cfg);
	if (ret < 0) {
		return ret;
	}

	cfg->irq_config_func();

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define HANDLE_SHARED_IRQ(n, active_irq)							\
	static const struct device *const dev_##n = DEVICE_DT_INST_GET(n);			\
	const struct adc_gd32_config *cfg_##n = dev_##n->config;				\
												\
	if ((cfg_##n->irq_num == active_irq) &&							\
		(ADC_CTL0(cfg_##n->reg) & ADC_CTL0_EOCIE)) {					\
		adc_gd32_isr(dev_##n);								\
	}

static void adc_gd32_global_irq_handler(const struct device *dev)
{
	const struct adc_gd32_config *cfg = dev->config;

	LOG_DBG("global irq handler: %u", cfg->irq_num);

	DT_INST_FOREACH_STATUS_OKAY_VARGS(HANDLE_SHARED_IRQ, (cfg->irq_num));
}

static void adc_gd32_global_irq_cfg(void)
{
	static bool global_irq_init = true;

	if (!global_irq_init) {
		return;
	}

	global_irq_init = false;

#if ADC0_ENABLE
	/* Shared irq config default to adc0. */
	IRQ_CONNECT(DT_IRQN(ADC0_NODE),
		DT_IRQ(ADC0_NODE, priority),
		adc_gd32_global_irq_handler,
		DEVICE_DT_GET(ADC0_NODE),
		0);
	irq_enable(DT_IRQN(ADC0_NODE));
#elif ADC1_ENABLE
	IRQ_CONNECT(DT_IRQN(ADC1_NODE),
		DT_IRQ(ADC1_NODE, priority),
		adc_gd32_global_irq_handler,
		DEVICE_DT_GET(ADC1_NODE),
		0);
	irq_enable(DT_IRQN(ADC1_NODE));
#endif

#if (ADC0_ENABLE || ADC1_ENABLE) && \
	defined(CONFIG_SOC_SERIES_GD32F4XX)
	/* gd32f4xx adc2 share the same irq number with adc0 and adc1. */
#elif ADC2_ENABLE
	IRQ_CONNECT(DT_IRQN(ADC2_NODE),
		DT_IRQ(ADC2_NODE, priority),
		adc_gd32_global_irq_handler,
		DEVICE_DT_GET(ADC2_NODE),
		0);
	irq_enable(DT_IRQN(ADC2_NODE));
#endif
}

#ifdef CONFIG_SOC_SERIES_GD32F3X0
#define ADC_CLOCK_SOURCE(n)									\
	.rcu_clock_source = DT_INST_PROP(n, rcu_clock_source)
#else
#define ADC_CLOCK_SOURCE(n)
#endif

#define ADC_GD32_INIT(n)									\
	PINCTRL_DT_INST_DEFINE(n);								\
	static struct adc_gd32_data adc_gd32_data_##n = {					\
		ADC_CONTEXT_INIT_TIMER(adc_gd32_data_##n, ctx),					\
		ADC_CONTEXT_INIT_LOCK(adc_gd32_data_##n, ctx),					\
		ADC_CONTEXT_INIT_SYNC(adc_gd32_data_##n, ctx),					\
	};											\
	static DEVICE_API(adc, adc_gd32_driver_api_##n) = {					\
		.channel_setup = adc_gd32_channel_setup,						\
		.read = adc_gd32_read,								\
		ADC_GD32_API_ASYNC_FIELD							\
		.ref_internal = DT_INST_PROP(n, vref_mv),						\
	};											\
	const static struct adc_gd32_config adc_gd32_config_##n = {				\
		.reg = DT_INST_REG_ADDR(n),							\
		.clkid = DT_INST_CLOCKS_CELL(n, id),						\
		.reset = RESET_DT_SPEC_INST_GET(n),						\
		.channels = DT_INST_PROP(n, channels),						\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					\
		.irq_num = DT_INST_IRQN(n),							\
		.irq_config_func = adc_gd32_global_irq_cfg,					\
		ADC_CLOCK_SOURCE(n)								\
	};											\
	DEVICE_DT_INST_DEFINE(n,								\
			      adc_gd32_init, NULL,						\
			      &adc_gd32_data_##n, &adc_gd32_config_##n,				\
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,				\
			      &adc_gd32_driver_api_##n);						\

DT_INST_FOREACH_STATUS_OKAY(ADC_GD32_INIT)
