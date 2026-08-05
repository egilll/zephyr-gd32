/*
 * Copyright (c) 2026, Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dwmac_plat, CONFIG_ETHERNET_LOG_LEVEL);

#define DT_DRV_COMPAT gd_gd32_ethernet

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/syscon.h>
#include <zephyr/irq.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/net/ethernet.h>

#include "eth_dwmac_priv.h"
#include "eth_dwmac_platform.h"

#define DATA_BUS_WIDTH       32
#define DESCRIPTOR_ALIGNMENT (DATA_BUS_WIDTH / BITS_PER_BYTE)

#if defined(CONFIG_NOCACHE_MEMORY)
#define DWMAC_DESC_MEM __nocache_noinit __aligned(DESCRIPTOR_ALIGNMENT)
#else
#define DWMAC_DESC_MEM __noinit __aligned(DESCRIPTOR_ALIGNMENT)
#endif

DWMAC_ASSERT_BUFFER_ALIGNMENT(DATA_BUS_WIDTH);

#define GD32_DWMAC_CLOCK_ID(node_id, prop, idx) DT_CLOCKS_CELL_BY_IDX(node_id, idx, id)

struct gd32_dwmac_config {
	struct dwmac_config common;
	const struct pinctrl_dev_config *pinctrl;
	const struct reset_dt_spec reset;
	const struct device *phy_mode_syscon;
	const struct device *phy_mode_clock;
	int (*mac_addr_load)(uint8_t mac_addr[NET_ETH_ADDR_LEN]);
	const uint16_t *clocks;
	size_t num_clocks;
	uint16_t phy_mode_clock_id;
	uint32_t phy_mode_offset;
	uint32_t phy_mode_mask;
	uint32_t phy_mode_value;
	struct dwmac_dma_desc *tx_descs;
	struct dwmac_dma_desc *rx_descs;
	void (*irq_config)(void);
};

static int gd32_clock_enable(const struct device *clock, const uint16_t *id)
{
	int ret = clock_control_on(clock, (clock_control_subsys_t)id);

	return (ret == -EALREADY) ? 0 : ret;
}

int dwmac_bus_init(const struct device *dev)
{
	const struct gd32_dwmac_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->common.clock)) {
		LOG_ERR("MAC clock controller is not ready");
		return -ENODEV;
	}

	if (!device_is_ready(cfg->phy_mode_clock)) {
		LOG_ERR("PHY mode selector clock controller is not ready");
		return -ENODEV;
	}

	if (!device_is_ready(cfg->phy_mode_syscon)) {
		LOG_ERR("PHY mode syscon is not ready");
		return -ENODEV;
	}

	if (!device_is_ready(cfg->reset.dev)) {
		LOG_ERR("Reset controller is not ready");
		return -ENODEV;
	}

	ret = reset_line_assert_dt(&cfg->reset);
	if (ret < 0) {
		LOG_ERR("Could not assert the ethernet MAC reset (%d)", ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Could not configure ethernet pins (%d)", ret);
		return ret;
	}

	ret = gd32_clock_enable(cfg->phy_mode_clock, &cfg->phy_mode_clock_id);
	if (ret < 0) {
		LOG_ERR("Could not enable PHY mode selector clock (%d)", ret);
		return ret;
	}

	ret = syscon_update_bits(cfg->phy_mode_syscon, cfg->phy_mode_offset, cfg->phy_mode_mask,
				 cfg->phy_mode_value);
	if (ret < 0) {
		LOG_ERR("Could not select the PHY interface (%d)", ret);
		return ret;
	}

	for (size_t i = 0U; i < cfg->num_clocks; i++) {
		ret = gd32_clock_enable(cfg->common.clock, &cfg->clocks[i]);
		if (ret < 0) {
			LOG_ERR("Could not enable ethernet clock #%zu (%d)", i, ret);
			return ret;
		}
	}
	ret = reset_line_deassert_dt(&cfg->reset);
	if (ret < 0) {
		LOG_ERR("Could not release the ethernet MAC reset (%d)", ret);
	}

	return ret;
}

int dwmac_platform_init(const struct device *dev)
{
	const struct gd32_dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;
	int ret;

	p->tx_descs = cfg->tx_descs;
	p->rx_descs = cfg->rx_descs;

	ret = cfg->mac_addr_load(p->mac_addr);
	if (ret < 0) {
		LOG_ERR("Could not determine the MAC address (%d)", ret);
		return ret;
	}

	cfg->irq_config();

	return 0;
}

#define GD32_DWMAC_INIT(n)                                                                         \
	BUILD_ASSERT(DT_INST_ENUM_HAS_VALUE(n, phy_connection_type, mii) ||                        \
			     DT_INST_ENUM_HAS_VALUE(n, phy_connection_type, rmii),                 \
		     "GD32 Ethernet supports MII and RMII PHY interfaces only");                   \
	BUILD_ASSERT(DT_INST_PROP(n, gd_phy_mode_mask) != 0U,                                      \
		     "gd,phy-mode-mask must select at least one bit");                             \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct dwmac_dma_desc dwmac_tx_descs_##n[NB_TX_DESCS] DWMAC_DESC_MEM;               \
	static struct dwmac_dma_desc dwmac_rx_descs_##n[NB_RX_DESCS] DWMAC_DESC_MEM;               \
	static const uint16_t dwmac_clocks_##n[] = {                                               \
		DT_FOREACH_PROP_ELEM_SEP(DT_DRV_INST(n), clocks, GD32_DWMAC_CLOCK_ID, (,)) \
	};                                                                                         \
	static int dwmac_mac_addr_load_##n(uint8_t mac_addr[NET_ETH_ADDR_LEN])                     \
	{                                                                                          \
		const struct net_eth_mac_config mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(n);      \
                                                                                                   \
		return dwmac_mac_addr_load(&mac_cfg, mac_addr, NULL);                              \
	}                                                                                          \
	IF_ENABLED(CONFIG_PTP_CLOCK_DWC_MAC, ( \
		BUILD_ASSERT(DT_INST_CLOCKS_HAS_NAME(n, ptp), \
			     "PTP requires a clock named \"ptp\""); \
	))                                                   \
	static void dwmac_irq_config_##n(void)                                                     \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), dwmac_isr,                  \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static const struct gd32_dwmac_config dwmac_config_##n = {                                 \
		.common = {                                                                        \
			DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                      \
			.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, phy_handle)),                  \
			.clock = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mac)),               \
			.mac_clk = (clock_control_subsys_t) &                                      \
				   dwmac_clocks_##n[DT_PHA_ELEM_IDX_BY_NAME(DT_DRV_INST(n),        \
									    clocks, mac)],         \
			.rx_chain_mode = DT_INST_PROP_OR(n, snps_rx_chain_mode, false),            \
			IF_ENABLED(CONFIG_PTP_CLOCK_DWC_MAC, ( \
				.ptp_clock = DEVICE_DT_GET(DT_INST_CHILD(n, ptp_clock)), \
					.ptp_clk = (clock_control_subsys_t)&dwmac_clocks_##n[\
					DT_PHA_ELEM_IDX_BY_NAME(DT_DRV_INST(n), clocks, ptp)], \
			)) },    \
					    .pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(n),          \
					    .reset = RESET_DT_SPEC_INST_GET(n),                    \
					    .phy_mode_syscon = DEVICE_DT_GET(                      \
						    DT_INST_PHANDLE(n, gd_phy_mode_selector)),     \
					    .phy_mode_clock = DEVICE_DT_GET(DT_CLOCKS_CTLR(        \
						    DT_INST_PHANDLE(n, gd_phy_mode_selector))),    \
					    .mac_addr_load = dwmac_mac_addr_load_##n,              \
					    .clocks = dwmac_clocks_##n,                            \
					    .num_clocks = ARRAY_SIZE(dwmac_clocks_##n),            \
					    .phy_mode_clock_id = DT_CLOCKS_CELL(                   \
						    DT_INST_PHANDLE(n, gd_phy_mode_selector), id), \
					    .phy_mode_offset =                                     \
						    DT_INST_PROP(n, gd_phy_mode_offset),           \
					    .phy_mode_mask = DT_INST_PROP(n, gd_phy_mode_mask),    \
					    .phy_mode_value = COND_CODE_1( \
			DT_INST_ENUM_HAS_VALUE(n, phy_connection_type, rmii), \
			(DT_INST_PROP(n, gd_phy_mode_mask)), (0U)),      \
						     .tx_descs = dwmac_tx_descs_##n,               \
						     .rx_descs = dwmac_rx_descs_##n,               \
						     .irq_config = dwmac_irq_config_##n,           \
	};                                                                                         \
	static struct dwmac_priv dwmac_data_##n;                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, dwmac_probe, NULL, &dwmac_data_##n, &dwmac_config_##n,    \
				      CONFIG_ETH_INIT_PRIORITY, &dwmac_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(GD32_DWMAC_INIT)
