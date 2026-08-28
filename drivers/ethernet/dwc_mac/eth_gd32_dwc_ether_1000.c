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

#define GD32_CLOCK_ID(node_id, prop, idx)   DT_PHA_BY_IDX(node_id, prop, idx, id),
#define GD32_RESET_SPEC(node_id, prop, idx) RESET_DT_SPEC_GET_BY_IDX(node_id, idx),

static uint16_t gd32_clocks[] = {DT_FOREACH_PROP_ELEM(DT_DRV_INST(0), clocks, GD32_CLOCK_ID)};
static uint16_t gd32_syscfg_clk = GD32_CLOCK_SYSCFG;

static const struct reset_dt_spec gd32_resets[] = {
	DT_FOREACH_PROP_ELEM(DT_DRV_INST(0), resets, GD32_RESET_SPEC)};

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

	for (size_t i = 0U; i < ARRAY_SIZE(gd32_clocks); i++) {
		ret = gd32_clock_enable(cfg->clock, &gd32_clocks[i]);
		if (ret < 0) {
			LOG_ERR("Failed to enable Ethernet clock %zu (%d)", i, ret);
			return ret;
		}
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

int dwmac_platform_init(const struct device *dev)
{
	const struct net_eth_mac_config mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(0);
	struct dwmac_priv *p = dev->data;
	const char *mac_source;
	int ret;

	p->tx_descs = dwmac_tx_descs;
	p->rx_descs = dwmac_rx_descs;

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), dwmac_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	ret = net_eth_mac_load(&mac_cfg, p->mac_addr);
	if (ret == -ENODATA && mac_cfg.type == NET_ETH_MAC_DEFAULT) {
		ret = gd32_mac_from_hwinfo(&mac_cfg, p->mac_addr);
		mac_source = "device ID fallback";
	} else if (mac_cfg.type == NET_ETH_MAC_NVMEM) {
		mac_source = "NVMEM";
	} else if (mac_cfg.type == NET_ETH_MAC_RANDOM) {
		mac_source = "random";
	} else {
		mac_source = "devicetree";
	}
	if (ret < 0) {
		LOG_ERR("Failed to load MAC address (%d)", ret);
		return ret;
	}
	if (!gd32_mac_is_valid(p->mac_addr)) {
		LOG_ERR("Loaded an invalid MAC address from %s", mac_source);
		return -EINVAL;
	}

	LOG_INF("MAC %02x:%02x:%02x:%02x:%02x:%02x source=%s", p->mac_addr[0], p->mac_addr[1],
		p->mac_addr[2], p->mac_addr[3], p->mac_addr[4], p->mac_addr[5], mac_source);

	return 0;
}

BUILD_ASSERT(!DT_INST_NVMEM_CELLS_HAS_NAME(0, mac_address) || IS_ENABLED(CONFIG_NVMEM),
	     "CONFIG_NVMEM is required when GD32 Ethernet uses an NVMEM MAC address");

static const struct dwmac_config dwmac_config = {
	DEVICE_MMIO_ROM_INIT(DT_DRV_INST(0)),
	.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(0, phy_handle)),
	.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(0, mac)),
	.mac_clk = (clock_control_subsys_t)&gd32_clocks[DT_PHA_ELEM_IDX_BY_NAME(DT_DRV_INST(0),
										clocks, mac)],
};

static struct dwmac_priv dwmac_instance;

ETH_NET_DEVICE_DT_INST_DEFINE(0, dwmac_probe, NULL, &dwmac_instance, &dwmac_config,
			      CONFIG_ETH_INIT_PRIORITY, &dwmac_api, NET_ETH_MTU);
