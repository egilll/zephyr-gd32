/*
 * Copyright (c) 2026, Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_ethernet

#include <errno.h>
#include <string.h>

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/dt-bindings/clock/gd32f4xx-clocks.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/sys/crc.h>

#include <gd32f4xx_rcu.h>
#include <gd32f4xx_syscfg.h>

#include "eth_dwmac_priv.h"

LOG_MODULE_REGISTER(dwmac_gd32, CONFIG_ETHERNET_LOG_LEVEL);

#define DATA_BUS_WIDTH       32U
#define DESCRIPTOR_ALIGNMENT (DATA_BUS_WIDTH / BITS_PER_BYTE)

DWMAC_ASSERT_BUFFER_ALIGNMENT(DATA_BUS_WIDTH);

BUILD_ASSERT(DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, mii) ||
		     DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, rmii),
	     "Unsupported PHY connection type");

PINCTRL_DT_INST_DEFINE(0);

static const struct pinctrl_dev_config *const eth0_pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0);

static uint16_t gd32_mac_clk = DT_INST_CLOCKS_CELL_BY_NAME(0, mac, id);
static uint16_t gd32_tx_clk = DT_INST_CLOCKS_CELL_BY_NAME(0, tx, id);
static uint16_t gd32_rx_clk = DT_INST_CLOCKS_CELL_BY_NAME(0, rx, id);
static uint16_t gd32_ptp_clk = DT_INST_CLOCKS_CELL_BY_NAME(0, ptp, id);
static uint16_t gd32_syscfg_clk = GD32_CLOCK_SYSCFG;

static const struct reset_dt_spec gd32_resets[] = {
	RESET_DT_SPEC_INST_GET_BY_IDX(0, 0),
	RESET_DT_SPEC_INST_GET_BY_IDX(0, 1),
	RESET_DT_SPEC_INST_GET_BY_IDX(0, 2),
	RESET_DT_SPEC_INST_GET_BY_IDX(0, 3),
};

#if defined(CONFIG_NOCACHE_MEMORY)
#define __desc_mem __nocache_noinit __aligned(DESCRIPTOR_ALIGNMENT)
#else
#define __desc_mem __noinit __aligned(DESCRIPTOR_ALIGNMENT)
#endif

static struct dwmac_dma_desc dwmac_tx_descs[NB_TX_DESCS] __desc_mem;
static struct dwmac_dma_desc dwmac_rx_descs[NB_RX_DESCS] __desc_mem;

static int gd32_clock_enable(const struct device *clock, uint16_t *id)
{
	return clock_control_on(clock, (clock_control_subsys_t)id);
}

static void gd32_phy_interface_configure(void)
{
	uint32_t cfg = SYSCFG_CFG1;

	cfg &= ~SYSCFG_CFG1_ENET_PHY_SEL;
	if (DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, rmii)) {
		cfg |= SYSCFG_CFG1_ENET_PHY_SEL;
	}
	SYSCFG_CFG1 = cfg;
}

static void gd32_phy_clock_configure(void)
{
	if (DT_INST_PROP(0, gd_phy_clk_out)) {
		uint32_t cfg = RCU_CFG0;

		cfg &= ~(RCU_CFG0_CKOUT0SEL | RCU_CFG0_CKOUT0DIV);
		cfg |= RCU_CKOUT0SRC_PLLP | RCU_CKOUT0_DIV4;
		RCU_CFG0 = cfg;
	}
}

int dwmac_bus_init(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->clock)) {
		return -ENODEV;
	}

	ret = pinctrl_apply_state(eth0_pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ret = gd32_clock_enable(cfg->clock, &gd32_syscfg_clk);
	if (ret < 0) {
		return ret;
	}

	gd32_phy_interface_configure();
	gd32_phy_clock_configure();

	ret = gd32_clock_enable(cfg->clock, &gd32_mac_clk);
	ret = ret < 0 ? ret : gd32_clock_enable(cfg->clock, &gd32_tx_clk);
	ret = ret < 0 ? ret : gd32_clock_enable(cfg->clock, &gd32_rx_clk);
	ret = ret < 0 ? ret : gd32_clock_enable(cfg->clock, &gd32_ptp_clk);
	if (ret < 0) {
		return ret;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(gd32_resets); i++) {
		if (!device_is_ready(gd32_resets[i].dev)) {
			return -ENODEV;
		}

		ret = reset_line_toggle_dt(&gd32_resets[i]);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static bool gd32_mac_is_valid(const uint8_t mac_addr[NET_ETH_ADDR_LEN])
{
	struct net_eth_addr addr;

	memcpy(addr.addr, mac_addr, sizeof(addr.addr));
	return net_eth_is_addr_valid(&addr);
}

static bool gd32_mac_suffix_is_erased(const struct net_eth_mac_config *cfg,
				      const uint8_t mac_addr[NET_ETH_ADDR_LEN])
{
	bool all_zero = true;
	bool all_one = true;

	for (size_t i = cfg->addr_len; i < NET_ETH_ADDR_LEN; i++) {
		all_zero = all_zero && mac_addr[i] == 0U;
		all_one = all_one && mac_addr[i] == UINT8_MAX;
	}

	return all_zero || all_one;
}

static int gd32_mac_from_hwinfo(const struct net_eth_mac_config *cfg,
				uint8_t mac_addr[NET_ETH_ADDR_LEN])
{
	uint8_t unique_id[16];
	ssize_t len;
	uint32_t hash0;
	uint32_t hash1;

	len = hwinfo_get_device_id(unique_id, sizeof(unique_id));
	if (len <= 0) {
		return len < 0 ? (int)len : -ENODATA;
	}

	hash0 = crc32_ieee(unique_id, (size_t)len);
	hash1 = crc32_ieee_update(0xa5a55a5aU, unique_id, (size_t)len);
	mac_addr[0] = 0x02U;
	mac_addr[1] = (uint8_t)(hash0 >> 24);
	mac_addr[2] = (uint8_t)(hash0 >> 16);
	mac_addr[3] = (uint8_t)(hash0 >> 8);
	mac_addr[4] = (uint8_t)hash0;
	mac_addr[5] = (uint8_t)hash1;
	if (cfg->addr_len > 0U) {
		memcpy(mac_addr, cfg->addr, cfg->addr_len);
	}

	return gd32_mac_is_valid(mac_addr) ? 0 : -EINVAL;
}

static int gd32_mac_load(const struct net_eth_mac_config *cfg,
			 uint8_t mac_addr[NET_ETH_ADDR_LEN])
{
#if defined(CONFIG_NVMEM)
	if (cfg->type == NET_ETH_MAC_NVMEM && cfg->cell.size == NET_ETH_ADDR_LEN) {
		return nvmem_cell_read(&cfg->cell, mac_addr, 0, NET_ETH_ADDR_LEN);
	}
#endif

	return net_eth_mac_load(cfg, mac_addr);
}

int dwmac_platform_init(const struct device *dev)
{
	const struct net_eth_mac_config mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(0);
	struct dwmac_priv *p = dev->data;
	int ret;

	p->tx_descs = dwmac_tx_descs;
	p->rx_descs = dwmac_rx_descs;

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), dwmac_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	ret = gd32_mac_load(&mac_cfg, p->mac_addr);
	if (ret < 0 || !gd32_mac_is_valid(p->mac_addr) ||
	    gd32_mac_suffix_is_erased(&mac_cfg, p->mac_addr)) {
		ret = gd32_mac_from_hwinfo(&mac_cfg, p->mac_addr);
	}
	if (ret < 0) {
		LOG_ERR("Failed to load MAC address (%d)", ret);
		return ret;
	}

	return 0;
}

static const struct dwmac_config dwmac_config = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
	.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(0, phy_handle)),
	.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(0, mac)),
	.mac_clk = (clock_control_subsys_t)&gd32_mac_clk,
};

static struct dwmac_priv dwmac_instance;

ETH_NET_DEVICE_DT_INST_DEFINE(0, dwmac_probe, NULL, &dwmac_instance, &dwmac_config,
			      CONFIG_ETH_INIT_PRIORITY, &dwmac_api, NET_ETH_MTU);
