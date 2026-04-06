/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gd32_enet_platform.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_SOC_SERIES_GD32F4XX)
#include <zephyr/dt-bindings/clock/gd32f4xx-clocks.h>
#include <gd32f4xx_gpio.h>
#include <gd32f4xx_rcu.h>
#include <gd32f4xx_syscfg.h>
#elif defined(CONFIG_SOC_SERIES_GD32E50X)
#include <zephyr/dt-bindings/clock/gd32e50x-clocks.h>
#include <gd32e50x_gpio.h>
#include <gd32e50x_rcu.h>
#else
#error "Unsupported GD32 SoC series for ENET platform helpers"
#endif

static int gd32_enet_clock_id_enable(uint16_t clk)
{
	return clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&clk);
}

static int gd32_enet_phy_interface_select(bool phy_mode_mii)
{
#if defined(CONFIG_SOC_SERIES_GD32F4XX)
	int ret = gd32_enet_clock_id_enable(GD32_CLOCK_SYSCFG);

	if (ret != 0) {
		return ret;
	}

	syscfg_enet_phy_interface_config(phy_mode_mii ? SYSCFG_ENET_PHY_MII : SYSCFG_ENET_PHY_RMII);
	return 0;
#elif defined(CONFIG_SOC_SERIES_GD32E50X)
	int ret = gd32_enet_clock_id_enable(GD32_CLOCK_AFIO);

	if (ret != 0) {
		return ret;
	}

	gpio_ethernet_phy_select(phy_mode_mii ? GPIO_ENET_PHY_MII : GPIO_ENET_PHY_RMII);
	return 0;
#endif
}

static int gd32_enet_phy_clk_out_enable(bool phy_mode_mii)
{
	int ret = gd32_enet_clock_id_enable(GD32_CLOCK_GPIOA);

	if (ret != 0) {
		return ret;
	}

	gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_8);
	gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_8);

#if defined(CONFIG_SOC_SERIES_GD32F4XX)
	if (phy_mode_mii) {
		rcu_ckout0_config(RCU_CKOUT0SRC_HXTAL, RCU_CKOUT0_DIV1);
	} else {
		rcu_ckout0_config(RCU_CKOUT0SRC_PLLP, RCU_CKOUT0_DIV4);
	}
#elif defined(CONFIG_SOC_SERIES_GD32E50X)
	if (phy_mode_mii) {
		rcu_ckout0_config(RCU_CKOUT0SRC_HXTAL);
	} else {
		rcu_ckout0_config(RCU_CKOUT0SRC_CKPLL2);
	}
#endif

	return 0;
}

static int gd32_enet_soc_reset_mac(void)
{
#if defined(CONFIG_SOC_SERIES_GD32F4XX) || defined(CONFIG_SOC_SERIES_GD32E50X)
	rcu_periph_reset_enable(RCU_ENETRST);
	rcu_periph_reset_disable(RCU_ENETRST);
	return 0;
#else
	return -ENOTSUP;
#endif
}

int gd32_enet_platform_enable_clocks(const struct gd32_enet_platform_config *cfg)
{
	int ret;
	const uint16_t clks[] = {
		cfg->mac_clk,
		cfg->tx_clk,
		cfg->rx_clk,
	};

	for (size_t i = 0; i < ARRAY_SIZE(clks); i++) {
		ret = gd32_enet_clock_id_enable(clks[i]);
		if (ret != 0) {
			return ret;
		}
	}

	if (cfg->has_ptp_clk) {
		ret = gd32_enet_clock_id_enable(cfg->ptp_clk);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

int gd32_enet_platform_configure_phy(const struct gd32_enet_platform_config *cfg)
{
	int ret;

	ret = gd32_enet_phy_interface_select(cfg->phy_mode_mii);
	if (ret != 0) {
		return ret;
	}

	if (!cfg->phy_clk_internal) {
		return 0;
	}

	return gd32_enet_phy_clk_out_enable(cfg->phy_mode_mii);
}

int gd32_enet_platform_reset_mac(const struct gd32_enet_platform_config *cfg)
{
	if (cfg->has_mac_reset && (cfg->mac_reset.dev != NULL) &&
	    device_is_ready(cfg->mac_reset.dev)) {
		return reset_line_toggle_dt(&cfg->mac_reset);
	}

	return gd32_enet_soc_reset_mac();
}
