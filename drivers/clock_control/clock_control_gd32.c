/*
 * Copyright (c) 2022 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_cctl

#include <errno.h>
#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/kernel.h>

#include <gd32_regs.h>

#if defined(CONFIG_SOC_SERIES_GD32F4XX)
#include <gd32_rcu.h>
#endif

/** RCU offset (from id cell) */
#define GD32_CLOCK_ID_OFFSET(id) (((id) >> 6U) & 0xFFU)
/** RCU configuration bit (from id cell) */
#define GD32_CLOCK_ID_BIT(id)	 ((id)&0x1FU)

#define CPU_FREQ DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)

/** AHB prescaler exponents */
static const uint8_t ahb_exp[16] = {
	0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U,
};
/** APB1 prescaler exponents */
static const uint8_t apb1_exp[8] = {
	0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U,
};
/** APB2 prescaler exponents */
static const uint8_t apb2_exp[8] = {
	0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U,
};

struct clock_control_gd32_config {
	uint32_t base;
};

struct clock_control_gd32_data {
	uint16_t i2s_owner;
	bool i2s_claimed;
};

#if defined(CONFIG_SOC_SERIES_GD32F4XX)
K_MUTEX_DEFINE(gd32_i2s_clock_mutex);

struct gd32f4xx_i2s_clock_plan {
	uint16_t plli2s_n;
	uint8_t plli2s_r;
	uint16_t divider;
	bool odd;
	uint32_t actual_rate_hz;
	uint64_t rate_num;
	uint32_t rate_den;
	uint64_t error_num;
};

static int gd32_wait_flag(rcu_flag_enum flag, FlagStatus target, uint32_t timeout_ms)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (rcu_flag_get(flag) != target) {
		if (k_uptime_get() >= deadline) {
			return -ETIMEDOUT;
		}

		k_sleep(K_MSEC(1));
	}

	return 0;
}

static int gd32f4xx_plli2s_input_rate_get(uint32_t *rate_hz)
{
	uint32_t pllm = RCU_PLL & RCU_PLL_PLLPSC;

	if (pllm == 0U) {
		return -EINVAL;
	}

	if ((RCU_PLL & RCU_PLL_PLLSEL) == RCU_PLLSRC_HXTAL) {
		rcu_osci_on(RCU_HXTAL);

		int ret = gd32_wait_flag(RCU_FLAG_HXTALSTB, SET, 100);

		if (ret < 0) {
			return ret;
		}

		*rate_hz = HXTAL_VALUE / pllm;
	} else {
		*rate_hz = IRC16M_VALUE / pllm;
	}

	return 0;
}

static void gd32f4xx_i2s_plan_consider(struct gd32f4xx_i2s_clock_plan *plan, bool *found,
				       uint32_t input_rate_hz, uint32_t target_rate_hz,
				       uint16_t frame_factor, uint16_t plli2s_n, uint8_t plli2s_r,
				       uint16_t serial_div)
{
	uint64_t rate_num = (uint64_t)input_rate_hz * plli2s_n;
	uint32_t rate_den = (uint32_t)plli2s_r * frame_factor * serial_div;
	uint64_t target_num = (uint64_t)target_rate_hz * rate_den;
	uint64_t error_num = rate_num > target_num ? rate_num - target_num : target_num - rate_num;

	if (!*found || (error_num * plan->rate_den < plan->error_num * rate_den) ||
	    ((error_num * plan->rate_den == plan->error_num * rate_den) &&
	     (rate_num / plli2s_r < plan->rate_num / plan->plli2s_r))) {
		plan->plli2s_n = plli2s_n;
		plan->plli2s_r = plli2s_r;
		plan->divider = serial_div / 2U;
		plan->odd = (serial_div & 1U) != 0U;
		plan->actual_rate_hz = DIV_ROUND_CLOSEST(rate_num, rate_den);
		plan->rate_num = rate_num;
		plan->rate_den = rate_den;
		plan->error_num = error_num;
		*found = true;
	}
}

static int gd32f4xx_i2s_clock_plan(uint32_t input_rate_hz, uint32_t target_rate_hz,
				   uint16_t frame_factor, struct gd32f4xx_i2s_clock_plan *plan)
{
	bool found = false;

	if (target_rate_hz == 0U || frame_factor == 0U) {
		return -EINVAL;
	}

	for (uint16_t n = 50U; n <= 500U; n++) {
		for (uint8_t r = 2U; r <= 7U; r++) {
			uint64_t rate_num = (uint64_t)input_rate_hz * n;
			uint64_t serial_den = (uint64_t)target_rate_hz * r * frame_factor;
			uint64_t serial_div = rate_num / serial_den;
			uint64_t source_rate_hz = rate_num / r;

			if (rate_num < 100000000ULL || rate_num > 500000000ULL ||
			    source_rate_hz > 240000000ULL) {
				continue;
			}

			for (uint8_t round_up = 0U; round_up <= 1U; round_up++) {
				uint64_t candidate = serial_div + round_up;

				if (candidate < 4U || candidate > 511U) {
					continue;
				}

				gd32f4xx_i2s_plan_consider(plan, &found, input_rate_hz,
							   target_rate_hz, frame_factor, n, r,
							   candidate);
			}
		}
	}

	return found ? 0 : -EINVAL;
}

static int gd32f4xx_i2s_external_clock_plan(uint32_t source_rate_hz, uint32_t target_rate_hz,
					    uint16_t frame_factor,
					    struct gd32f4xx_i2s_clock_plan *plan)
{
	uint64_t serial_den;
	uint64_t serial_div;
	bool found = false;

	if (source_rate_hz == 0U || target_rate_hz == 0U || frame_factor == 0U) {
		return -EINVAL;
	}

	serial_den = (uint64_t)target_rate_hz * frame_factor;
	serial_div = source_rate_hz / serial_den;

	for (uint8_t round_up = 0U; round_up <= 1U; round_up++) {
		uint64_t candidate = serial_div + round_up;
		uint32_t rate_den;
		uint64_t target_num;
		uint64_t error_num;

		if (candidate < 4U || candidate > 511U) {
			continue;
		}

		rate_den = frame_factor * candidate;
		target_num = (uint64_t)target_rate_hz * rate_den;
		error_num = source_rate_hz > target_num ? source_rate_hz - target_num
							: target_num - source_rate_hz;
		if (!found || error_num * plan->rate_den < plan->error_num * rate_den) {
			plan->divider = candidate / 2U;
			plan->odd = (candidate & 1U) != 0U;
			plan->actual_rate_hz = DIV_ROUND_CLOSEST(source_rate_hz, rate_den);
			plan->rate_num = source_rate_hz;
			plan->rate_den = rate_den;
			plan->error_num = error_num;
			found = true;
		}
	}

	return found ? 0 : -EINVAL;
}

static bool gd32f4xx_i2s_clock_error_valid(const struct gd32f4xx_i2s_clock_plan *plan,
					   const struct gd32_i2s_clock_config *cfg)
{
	return cfg->max_error_ppm == 0U ||
	       plan->error_num * 1000000ULL <=
		       (uint64_t)cfg->target_rate_hz * plan->rate_den * cfg->max_error_ppm;
}

static int gd32f4xx_i2s_clock_configure(struct clock_control_gd32_data *data, uint16_t id,
					struct gd32_i2s_clock_config *cfg)
{
	struct gd32f4xx_i2s_clock_plan plan = {0};
	uint32_t input_rate_hz;
	int ret;

	if (cfg->action == GD32_I2S_CLOCK_RELEASE) {
		if (data->i2s_claimed && data->i2s_owner == id) {
			data->i2s_claimed = false;
		}

		return 0;
	}

	if (cfg->action != GD32_I2S_CLOCK_ACQUIRE) {
		return -EINVAL;
	}

	if (data->i2s_claimed && data->i2s_owner != id) {
		return -EBUSY;
	}

	if (cfg->source == GD32_I2S_CLOCK_SRC_EXTERNAL) {
		if (cfg->external_rate_hz == 0U) {
			return -EINVAL;
		}

		ret = gd32f4xx_i2s_external_clock_plan(cfg->external_rate_hz, cfg->target_rate_hz,
						       cfg->frame_factor, &plan);
		if (ret < 0) {
			return ret;
		}
		if (!gd32f4xx_i2s_clock_error_valid(&plan, cfg)) {
			return -ERANGE;
		}

		rcu_i2s_clock_config(RCU_I2SSRC_I2S_CKIN);
	} else if (cfg->source == GD32_I2S_CLOCK_SRC_PLLI2S) {
		ret = gd32f4xx_plli2s_input_rate_get(&input_rate_hz);
		if (ret < 0) {
			return ret;
		}

		ret = gd32f4xx_i2s_clock_plan(input_rate_hz, cfg->target_rate_hz, cfg->frame_factor,
					      &plan);
		if (ret < 0) {
			return ret;
		}
		if (!gd32f4xx_i2s_clock_error_valid(&plan, cfg)) {
			return -ERANGE;
		}

		rcu_osci_off(RCU_PLLI2S_CK);
		ret = gd32_wait_flag(RCU_FLAG_PLLI2SSTB, RESET, 10);
		if (ret < 0) {
			return ret;
		}

		if (rcu_plli2s_config(plan.plli2s_n, plan.plli2s_r) != SUCCESS) {
			return -EINVAL;
		}

		rcu_i2s_clock_config(RCU_I2SSRC_PLLI2S);
		rcu_osci_on(RCU_PLLI2S_CK);

		ret = gd32_wait_flag(RCU_FLAG_PLLI2SSTB, SET, 100);
		if (ret < 0) {
			return ret;
		}
	} else {
		return -EINVAL;
	}

	data->i2s_owner = id;
	data->i2s_claimed = true;
	cfg->divider = plan.divider;
	cfg->odd = plan.odd;
	cfg->actual_rate_hz = plan.actual_rate_hz;
	cfg->rate_num = plan.rate_num;
	cfg->rate_den = plan.rate_den;

	return 0;
}

/*
 * Configure the GD32F4xx CK48M tree to deliver a stable 48 MHz for USBFS / the
 * embedded FS PHY of USBHS. Source: PLLSAI-P. The PLLSAI input is shared with
 * the main PLL (HXTAL or IRC16M, divided by PLLM); we pick PLLSAI-N so that
 * VCO=288 MHz and divide by P=6 to get 48 MHz. Matches the vendor reference
 * which uses PLLSAI-P -> PLL48M -> CK48M.
 */
static int gd32f4xx_usb48m_configure(void)
{
	uint32_t pllm = RCU_PLL & RCU_PLL_PLLPSC;
	uint32_t input_rate_hz;
	uint32_t pllsai_n;

	if (pllm == 0U) {
		return -EINVAL;
	}

	if ((RCU_PLL & RCU_PLL_PLLSEL) == RCU_PLLSRC_HXTAL) {
		rcu_osci_on(RCU_HXTAL);

		int ret = gd32_wait_flag(RCU_FLAG_HXTALSTB, SET, 100);

		if (ret < 0) {
			return ret;
		}

		input_rate_hz = HXTAL_VALUE / pllm;
	} else {
		int ret = gd32_wait_flag(RCU_FLAG_IRC16MSTB, SET, 100);

		if (ret < 0) {
			return ret;
		}

		input_rate_hz = IRC16M_VALUE / pllm;
	}

	if ((input_rate_hz == 0U) || ((288000000U % input_rate_hz) != 0U)) {
		return -EINVAL;
	}

	pllsai_n = 288000000U / input_rate_hz;
	if ((pllsai_n < RCU_PLLSAIN_MUL_MIN) || (pllsai_n > RCU_PLLSAIN_MUL_MAX)) {
		return -EINVAL;
	}

	rcu_osci_off(RCU_PLLSAI_CK);
	(void)gd32_wait_flag(RCU_FLAG_PLLSAISTB, RESET, 10);

	if (rcu_pllsai_config(pllsai_n, 6U, 2U) != SUCCESS) {
		return -EINVAL;
	}

	rcu_osci_on(RCU_PLLSAI_CK);

	int ret = gd32_wait_flag(RCU_FLAG_PLLSAISTB, SET, 100);

	if (ret < 0) {
		return ret;
	}

	rcu_pll48m_clock_config(RCU_PLL48MSRC_PLLSAIP);
	rcu_ck48m_clock_config(RCU_CK48MSRC_PLL48M);

	return 0;
}
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_timer)
/* timer identifiers */
#define TIMER_ID_OR_NONE(nodelabel)                                            \
	COND_CODE_1(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(nodelabel)),          \
		    (DT_CLOCKS_CELL(DT_NODELABEL(nodelabel), id),), ())

static const uint16_t timer_ids[] = {
	TIMER_ID_OR_NONE(timer0)  /* */
	TIMER_ID_OR_NONE(timer1)  /* */
	TIMER_ID_OR_NONE(timer2)  /* */
	TIMER_ID_OR_NONE(timer3)  /* */
	TIMER_ID_OR_NONE(timer4)  /* */
	TIMER_ID_OR_NONE(timer5)  /* */
	TIMER_ID_OR_NONE(timer6)  /* */
	TIMER_ID_OR_NONE(timer7)  /* */
	TIMER_ID_OR_NONE(timer8)  /* */
	TIMER_ID_OR_NONE(timer9)  /* */
	TIMER_ID_OR_NONE(timer10) /* */
	TIMER_ID_OR_NONE(timer11) /* */
	TIMER_ID_OR_NONE(timer12) /* */
	TIMER_ID_OR_NONE(timer13) /* */
	TIMER_ID_OR_NONE(timer14) /* */
	TIMER_ID_OR_NONE(timer15) /* */
	TIMER_ID_OR_NONE(timer16) /* */
};
#endif /* DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_timer) */

static int clock_control_gd32_on(const struct device *dev,
				 clock_control_subsys_t sys)
{
	const struct clock_control_gd32_config *config = dev->config;
	uint16_t id = *(uint16_t *)sys;

	sys_set_bit(config->base + GD32_CLOCK_ID_OFFSET(id),
		    GD32_CLOCK_ID_BIT(id));

	return 0;
}

static int clock_control_gd32_off(const struct device *dev,
				  clock_control_subsys_t sys)
{
	const struct clock_control_gd32_config *config = dev->config;
	uint16_t id = *(uint16_t *)sys;

	sys_clear_bit(config->base + GD32_CLOCK_ID_OFFSET(id),
		      GD32_CLOCK_ID_BIT(id));

	return 0;
}

static int clock_control_gd32_get_rate(const struct device *dev,
				       clock_control_subsys_t sys,
				       uint32_t *rate)
{
	const struct clock_control_gd32_config *config = dev->config;
	uint16_t id = *(uint16_t *)sys;
	uint32_t cfg;
	uint8_t psc;
	uint8_t psc_exp;

	cfg = sys_read32(config->base + RCU_CFG0_OFFSET);

	switch (GD32_CLOCK_ID_OFFSET(id)) {
#if defined(CONFIG_SOC_SERIES_GD32F4XX)
	case RCU_AHB1EN_OFFSET:
	case RCU_AHB2EN_OFFSET:
	case RCU_AHB3EN_OFFSET:
#else
	case RCU_AHBEN_OFFSET:
#endif
		psc = (cfg & RCU_CFG0_AHBPSC_MSK) >> RCU_CFG0_AHBPSC_POS;
		psc_exp = ahb_exp[psc];
		*rate = CPU_FREQ >> psc_exp;
		break;
	case RCU_APB1EN_OFFSET:
#if !defined(CONFIG_SOC_SERIES_GD32VF103) && \
	!defined(CONFIG_SOC_SERIES_GD32A50X) && \
	!defined(CONFIG_SOC_SERIES_GD32L23X)
	case RCU_ADDAPB1EN_OFFSET:
#endif
		psc = (cfg & RCU_CFG0_APB1PSC_MSK) >> RCU_CFG0_APB1PSC_POS;
		psc_exp = apb1_exp[psc];
		*rate = CPU_FREQ >> psc_exp;
		break;
	case RCU_APB2EN_OFFSET:
		psc = (cfg & RCU_CFG0_APB2PSC_MSK) >> RCU_CFG0_APB2PSC_POS;
		psc_exp = apb2_exp[psc];
		*rate = CPU_FREQ >> psc_exp;
		break;
	default:
		return -ENOTSUP;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_timer)
	/* handle timer clocks */
	for (size_t i = 0U; i < ARRAY_SIZE(timer_ids); i++) {
		if (id != timer_ids[i]) {
			continue;
		}

#if defined(CONFIG_SOC_SERIES_GD32F4XX)
		uint32_t cfg1 = sys_read32(config->base + RCU_CFG1_OFFSET);

		/*
		 * The TIMERSEL bit in RCU_CFG1 controls the clock frequency of
		 * all the timers connected to the APB1 and APB2 domains.
		 *
		 * Up to a certain APB{1,2} divisor, the timer clock equals
		 * CK_AHB. This divisor depends on TIMERSEL (2 if TIMERSEL=0,
		 * 4 if TIMERSEL=1). Above it, the timer clock is a multiple of
		 * CK_APB{1,2} (2 if TIMERSEL=0, 4 if TIMERSEL=1).
		 */

		/* TIMERSEL = 0 */
		if ((cfg1 & RCU_CFG1_TIMERSEL_MSK) == 0U) {
			if (psc_exp <= 1U) {
				*rate = CPU_FREQ;
			} else {
				*rate *= 2U;
			}
		/* TIMERSEL = 1 */
		} else {
			if (psc_exp <= 2U) {
				*rate = CPU_FREQ;
			} else {
				*rate *= 4U;
			}
		}
#else
		/*
		 * If the APB prescaler equals 1, the timer clock frequencies
		 * are set to the same frequency as that of the APB domain.
		 * Otherwise, they are set to twice the frequency of the APB
		 * domain.
		 */
		if (psc_exp != 0U) {
			*rate *= 2U;
		}
#endif /* CONFIG_SOC_SERIES_GD32F4XX */
	}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_timer) */

	return 0;
}

static enum clock_control_status
clock_control_gd32_get_status(const struct device *dev,
			      clock_control_subsys_t sys)
{
	const struct clock_control_gd32_config *config = dev->config;
	uint16_t id = *(uint16_t *)sys;

	if (sys_test_bit(config->base + GD32_CLOCK_ID_OFFSET(id),
			 GD32_CLOCK_ID_BIT(id)) != 0) {
		return CLOCK_CONTROL_STATUS_ON;
	}

	return CLOCK_CONTROL_STATUS_OFF;
}

static int clock_control_gd32_configure(const struct device *dev,
					clock_control_subsys_t sys,
					void *data)
{
	struct clock_control_gd32_data *dev_data = dev->data;

	if (data == NULL) {
		return -EINVAL;
	}

	/* Every gd32_*_clock_config struct starts with `enum
	 * gd32_clock_config_type type`, so we can dispatch off the first
	 * field without knowing the concrete payload yet.
	 */
	enum gd32_clock_config_type type = *(enum gd32_clock_config_type *)data;

	switch (type) {
#if defined(CONFIG_SOC_SERIES_GD32F4XX)
	case GD32_CLOCK_CONFIG_TYPE_I2S: {
		uint16_t id;
		int ret;

		if (sys == NULL) {
			return -EINVAL;
		}
		id = *(uint16_t *)sys;
		k_mutex_lock(&gd32_i2s_clock_mutex, K_FOREVER);
		ret = gd32f4xx_i2s_clock_configure(dev_data, id, data);
		k_mutex_unlock(&gd32_i2s_clock_mutex);

		return ret;
	}
	case GD32_CLOCK_CONFIG_TYPE_USB48M:
		return gd32f4xx_usb48m_configure();
#endif
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(clock_control, clock_control_gd32_api) = {
	.on = clock_control_gd32_on,
	.off = clock_control_gd32_off,
	.get_rate = clock_control_gd32_get_rate,
	.get_status = clock_control_gd32_get_status,
	.configure = clock_control_gd32_configure,
};

static const struct clock_control_gd32_config config = {
	.base = DT_REG_ADDR(DT_INST_PARENT(0)),
};

static struct clock_control_gd32_data data;

DEVICE_DT_INST_DEFINE(0, NULL, NULL, &data, &config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_gd32_api);
