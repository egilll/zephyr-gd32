// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Ylhyra ehf.

#define DT_DRV_COMPAT gd_gd32_ckout

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>

#include <gd32_rcu.h>

struct gd32_ckout_config {
	const struct pinctrl_dev_config *pcfg;
	uint32_t frequency;
	uint8_t output;
	uint8_t source;
};

enum gd32_ckout_source {
	GD32_CKOUT_SOURCE_PLLP,
};

static int gd32_ckout_pllp_rate(uint32_t *rate)
{
	uint32_t pllpsc = FIELD_GET(RCU_PLL_PLLPSC, RCU_PLL);
	uint32_t plln = FIELD_GET(RCU_PLL_PLLN, RCU_PLL);
	uint32_t pllp = (FIELD_GET(RCU_PLL_PLLP, RCU_PLL) + 1U) * 2U;
	uint32_t source =
		(RCU_PLL & RCU_PLL_PLLSEL) == RCU_PLLSRC_HXTAL ? HXTAL_VALUE : IRC16M_VALUE;

	if (pllpsc == 0U || plln == 0U) {
		return -EINVAL;
	}

	*rate = (uint32_t)(((uint64_t)source * plln) / pllpsc / pllp);
	return 0;
}

static int gd32_ckout_init(const struct device *dev)
{
	static const uint32_t ckout0_dividers[] = {
		RCU_CKOUT0_DIV1, RCU_CKOUT0_DIV2, RCU_CKOUT0_DIV3, RCU_CKOUT0_DIV4, RCU_CKOUT0_DIV5,
	};
	static const uint32_t ckout1_dividers[] = {
		RCU_CKOUT1_DIV1, RCU_CKOUT1_DIV2, RCU_CKOUT1_DIV3, RCU_CKOUT1_DIV4, RCU_CKOUT1_DIV5,
	};
	const struct gd32_ckout_config *config = dev->config;
	uint32_t source_rate;
	uint32_t divider;
	int ret;

	if (config->source != GD32_CKOUT_SOURCE_PLLP) {
		return -ENOTSUP;
	}

	ret = gd32_ckout_pllp_rate(&source_rate);
	if (ret < 0) {
		return ret;
	}

	if (config->frequency == 0U || source_rate % config->frequency != 0U) {
		return -EINVAL;
	}

	divider = source_rate / config->frequency;
	if (divider == 0U || divider > ARRAY_SIZE(ckout0_dividers)) {
		return -EINVAL;
	}

	if (config->output == 0U) {
		rcu_ckout0_config(RCU_CKOUT0SRC_PLLP, ckout0_dividers[divider - 1U]);
	} else {
		rcu_ckout1_config(RCU_CKOUT1SRC_PLLP, ckout1_dividers[divider - 1U]);
	}

	return pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
}

#define GD32_CKOUT_INIT(inst)                                                                      \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const struct gd32_ckout_config gd32_ckout_config_##inst = {                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.frequency = DT_INST_PROP(inst, clock_frequency),                                  \
		.output = DT_INST_PROP(inst, output),                                              \
		.source = DT_INST_ENUM_IDX(inst, gd_clock_source),                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, gd32_ckout_init, NULL, NULL, &gd32_ckout_config_##inst,        \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(GD32_CKOUT_INIT)
