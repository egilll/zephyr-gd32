/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_GD32_ENET_PLATFORM_H_
#define ZEPHYR_DRIVERS_ETHERNET_GD32_ENET_PLATFORM_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/reset.h>

struct gd32_enet_platform_config {
	uint16_t mac_clk;
	uint16_t tx_clk;
	uint16_t rx_clk;
	uint16_t ptp_clk;
	struct reset_dt_spec mac_reset;
	bool has_ptp_clk;
	bool has_mac_reset;
	bool phy_mode_mii;
	bool phy_clk_internal;
};

int gd32_enet_platform_enable_clocks(const struct gd32_enet_platform_config *cfg);
int gd32_enet_platform_configure_phy(const struct gd32_enet_platform_config *cfg);
int gd32_enet_platform_reset_mac(const struct gd32_enet_platform_config *cfg);

#endif /* ZEPHYR_DRIVERS_ETHERNET_GD32_ENET_PLATFORM_H_ */
