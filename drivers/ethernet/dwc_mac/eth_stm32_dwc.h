/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_DWC_MAC_ETH_STM32_DWC_H_
#define ZEPHYR_DRIVERS_ETHERNET_DWC_MAC_ETH_STM32_DWC_H_

#include "eth_dwmac_platform.h"

#define ETH_STM32_PTP_CLK_IDX(n)                                                                   \
	DT_PHA_ELEM_IDX_BY_NAME(                                                                   \
		DT_DRV_INST(n), clocks,                                                            \
		COND_CODE_1(DT_INST_CLOCKS_HAS_NAME(n, mac_clk_ptp), (mac_clk_ptp), (stm_eth)))

#define ST_OUI_B0 0x00
#define ST_OUI_B1 0x80
#define ST_OUI_B2 0xE1

static inline int eth_stm32_net_eth_mac_load(const struct net_eth_mac_config *cfg,
					     uint8_t *mac_addr)
{
	static const uint8_t oui[] = {ST_OUI_B0, ST_OUI_B1, ST_OUI_B2};

	return dwmac_mac_addr_load(cfg, mac_addr, oui);
}

#endif /* ZEPHYR_DRIVERS_ETHERNET_DWC_MAC_ETH_STM32_DWC_H_ */
