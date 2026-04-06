/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#define DT_DRV_COMPAT gd_gd32_mdio

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "../gd32_enet_platform.h"

#if defined(CONFIG_SOC_SERIES_GD32F4XX)
#include <gd32f4xx_enet.h>
#elif defined(CONFIG_SOC_SERIES_GD32E50X)
#include <gd32e50x_enet.h>
#else
#error "Unsupported GD32 SoC series for MDIO"
#endif

LOG_MODULE_REGISTER(mdio_gd32, CONFIG_MDIO_LOG_LEVEL);

#define MDIO_GD32_OP_TIMEOUT_MS 10
#define MDIO_GD32_MAC_PHY_CTL_OFFSET  0x00U
#define MDIO_GD32_MAC_PHY_DATA_OFFSET 0x04U

struct mdio_gd32_config {
	uintptr_t base;
	const struct pinctrl_dev_config *pincfg;
	struct gd32_enet_platform_config platform;
	uint32_t mdc_freq;
};

struct mdio_gd32_data {
	struct k_sem sem;
};

static inline uint32_t mdio_gd32_reg_read(const struct mdio_gd32_config *cfg, uint32_t offset)
{
	return sys_read32(cfg->base + offset);
}

static inline void mdio_gd32_reg_write(const struct mdio_gd32_config *cfg, uint32_t offset,
				       uint32_t value)
{
	sys_write32(value, cfg->base + offset);
}

static int mdio_gd32_wait_ready(const struct device *dev)
{
	const struct mdio_gd32_config *cfg = dev->config;
	int64_t deadline = k_uptime_get() + MDIO_GD32_OP_TIMEOUT_MS;

	while ((mdio_gd32_reg_read(cfg, MDIO_GD32_MAC_PHY_CTL_OFFSET) & ENET_MAC_PHY_CTL_PB) != 0U) {
		if (k_uptime_get() > deadline) {
			return -ETIMEDOUT;
		}

		k_usleep(10);
	}

	return 0;
}

static int mdio_gd32_set_mdc_clock(const struct device *dev)
{
	const struct mdio_gd32_config *cfg = dev->config;
	uint32_t mdc_freq = cfg->mdc_freq;
	uint32_t ahbclk;
	uint32_t reg;
	uint32_t sel_bits = ENET_MDC_HCLK_DIV102;
	int ret;

	if (mdc_freq == 0U || mdc_freq > 2500000U) {
		mdc_freq = 2500000U;
	}

	ret = clock_control_get_rate(GD32_CLOCK_CONTROLLER,
				     (clock_control_subsys_t)&cfg->platform.mac_clk, &ahbclk);
	if (ret != 0) {
		return ret;
	}

	reg = mdio_gd32_reg_read(cfg, MDIO_GD32_MAC_PHY_CTL_OFFSET);
	reg &= ~ENET_MAC_PHY_CTL_CLR;

	const struct {
		uint32_t div;
		uint32_t bits;
	} divs[] = {
		{ 16U, ENET_MDC_HCLK_DIV16 },
		{ 26U, ENET_MDC_HCLK_DIV26 },
		{ 42U, ENET_MDC_HCLK_DIV42 },
		{ 62U, ENET_MDC_HCLK_DIV62 },
		{ 102U, ENET_MDC_HCLK_DIV102 },
	};

	for (size_t i = 0U; i < ARRAY_SIZE(divs); i++) {
		if ((uint64_t)ahbclk <= (uint64_t)mdc_freq * (uint64_t)divs[i].div) {
			sel_bits = divs[i].bits;
			break;
		}
	}

	mdio_gd32_reg_write(cfg, MDIO_GD32_MAC_PHY_CTL_OFFSET, reg | sel_bits);
	return 0;
}

static int mdio_gd32_prepare_hw(const struct device *dev)
{
	const struct mdio_gd32_config *cfg = dev->config;
	int ret;

	ret = gd32_enet_platform_enable_clocks(&cfg->platform);
	if (ret != 0) {
		return ret;
	}

	ret = gd32_enet_platform_configure_phy(&cfg->platform);
	if (ret == 0) {
		ret = mdio_gd32_set_mdc_clock(dev);
	}

	return ret;
}

static int mdio_gd32_clause22_read(const struct device *dev, uint8_t prtad, uint8_t regad,
				   uint16_t *data)
{
	const struct mdio_gd32_config *cfg = dev->config;
	uint32_t clk_bits;
	int ret;

	/*
	 * ENET and MDIO share the same hardware block, and ENET resets clear the
	 * MDC divider bits. Refresh them before each transaction.
	 */
	ret = mdio_gd32_set_mdc_clock(dev);
	if (ret != 0) {
		return ret;
	}

	ret = mdio_gd32_wait_ready(dev);
	if (ret != 0) {
		return ret;
	}

	clk_bits = mdio_gd32_reg_read(cfg, MDIO_GD32_MAC_PHY_CTL_OFFSET) & ENET_MAC_PHY_CTL_CLR;
	mdio_gd32_reg_write(cfg, MDIO_GD32_MAC_PHY_CTL_OFFSET,
			    clk_bits | MAC_PHY_CTL_PA(prtad & 0x1FU) |
				    MAC_PHY_CTL_PR(regad & 0x1FU) | ENET_MAC_PHY_CTL_PB);

	ret = mdio_gd32_wait_ready(dev);
	if (ret != 0) {
		return ret;
	}

	*data = (uint16_t)(mdio_gd32_reg_read(cfg, MDIO_GD32_MAC_PHY_DATA_OFFSET) & 0xFFFFU);
	return 0;
}

static int mdio_gd32_clause22_write(const struct device *dev, uint8_t prtad, uint8_t regad,
				    uint16_t data)
{
	const struct mdio_gd32_config *cfg = dev->config;
	uint32_t clk_bits;
	int ret;

	ret = mdio_gd32_set_mdc_clock(dev);
	if (ret != 0) {
		return ret;
	}

	ret = mdio_gd32_wait_ready(dev);
	if (ret != 0) {
		return ret;
	}

	clk_bits = mdio_gd32_reg_read(cfg, MDIO_GD32_MAC_PHY_CTL_OFFSET) & ENET_MAC_PHY_CTL_CLR;
	mdio_gd32_reg_write(cfg, MDIO_GD32_MAC_PHY_DATA_OFFSET, data);
	mdio_gd32_reg_write(cfg, MDIO_GD32_MAC_PHY_CTL_OFFSET,
			    clk_bits | MAC_PHY_CTL_PA(prtad & 0x1FU) |
				    MAC_PHY_CTL_PR(regad & 0x1FU) | ENET_MAC_PHY_CTL_PW |
				    ENET_MAC_PHY_CTL_PB);

	return mdio_gd32_wait_ready(dev);
}

static int mdio_gd32_read(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t *data)
{
	struct mdio_gd32_data *d = dev->data;
	int ret;

	k_sem_take(&d->sem, K_FOREVER);
	ret = mdio_gd32_clause22_read(dev, prtad, regad, data);
	k_sem_give(&d->sem);

	return ret;
}

static int mdio_gd32_write(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t data)
{
	struct mdio_gd32_data *d = dev->data;
	int ret;

	k_sem_take(&d->sem, K_FOREVER);
	ret = mdio_gd32_clause22_write(dev, prtad, regad, data);
	k_sem_give(&d->sem);

	return ret;
}

static int mdio_gd32_init(const struct device *dev)
{
	const struct mdio_gd32_config *cfg = dev->config;
	struct mdio_gd32_data *d = dev->data;
	int ret;

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	k_sem_init(&d->sem, 1, 1);

	ret = mdio_gd32_prepare_hw(dev);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static DEVICE_API(mdio, mdio_gd32_api) = {
	.read = mdio_gd32_read,
	.write = mdio_gd32_write,
};

#define MDIO_GD32_DEVICE(inst)                                                                 \
	PINCTRL_DT_INST_DEFINE(inst);                                                        \
	static const struct mdio_gd32_config mdio_gd32_config_##inst = {                    \
		.base = DT_INST_REG_ADDR(inst),                                             \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                            \
		.platform = {                                                              \
			.mac_clk = DT_INST_CLOCKS_CELL_BY_IDX(inst, 0, id),                \
			.tx_clk = DT_INST_CLOCKS_CELL_BY_IDX(inst, 1, id),                 \
			.rx_clk = DT_INST_CLOCKS_CELL_BY_IDX(inst, 2, id),                 \
			.phy_mode_mii =                                                    \
				DT_ENUM_HAS_VALUE(DT_INST_PARENT(inst), phy_connection_type, \
						  mii),                               \
			.phy_clk_internal =                                                \
				DT_ENUM_HAS_VALUE(DT_INST_PARENT(inst), phy_clock_type,    \
						  internal),                          \
		},                                                                         \
		.mdc_freq = DT_INST_PROP(inst, clock_frequency),                           \
	};                                                                                   \
	static struct mdio_gd32_data mdio_gd32_data_##inst;                                  \
	DEVICE_DT_INST_DEFINE(inst, mdio_gd32_init, NULL, &mdio_gd32_data_##inst,            \
			      &mdio_gd32_config_##inst, POST_KERNEL,                       \
			      CONFIG_MDIO_INIT_PRIORITY, &mdio_gd32_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_GD32_DEVICE)
