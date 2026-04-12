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

#define MDIO_GD32_OP_TIMEOUT_US       (10U * USEC_PER_MSEC)
#define MDIO_GD32_READY_POLL_DELAY_US 10U
#define MDIO_GD32_MAX_MDC_FREQ        2500000U
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
	uint32_t clk_bits;
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

	if (!WAIT_FOR((mdio_gd32_reg_read(cfg, MDIO_GD32_MAC_PHY_CTL_OFFSET) &
		       ENET_MAC_PHY_CTL_PB) == 0U,
		      MDIO_GD32_OP_TIMEOUT_US, k_usleep(MDIO_GD32_READY_POLL_DELAY_US))) {
		return -ETIMEDOUT;
	}

	return 0;
}

static int mdio_gd32_calc_mdc_clock(const struct device *dev, uint32_t *clk_bits)
{
	const struct mdio_gd32_config *cfg = dev->config;
	uint32_t mdc_freq = cfg->mdc_freq;
	uint32_t ahbclk;
	uint32_t sel_bits = ENET_MDC_HCLK_DIV102;
	int ret;

	if (mdc_freq == 0U || mdc_freq > MDIO_GD32_MAX_MDC_FREQ) {
		mdc_freq = MDIO_GD32_MAX_MDC_FREQ;
	}

	ret = clock_control_get_rate(GD32_CLOCK_CONTROLLER,
				     (clock_control_subsys_t)&cfg->platform.mac_clk, &ahbclk);
	if (ret != 0) {
		return ret;
	}

	const struct {
		uint32_t div;
		uint32_t bits;
	} divs[] = {
		{16U, ENET_MDC_HCLK_DIV16},   {26U, ENET_MDC_HCLK_DIV26},
		{42U, ENET_MDC_HCLK_DIV42},   {62U, ENET_MDC_HCLK_DIV62},
		{102U, ENET_MDC_HCLK_DIV102},
	};

	for (size_t i = 0U; i < ARRAY_SIZE(divs); i++) {
		if ((uint64_t)ahbclk <= (uint64_t)mdc_freq * (uint64_t)divs[i].div) {
			sel_bits = divs[i].bits;
			break;
		}
	}

	*clk_bits = sel_bits;
	return 0;
}

static int mdio_gd32_prepare_hw(const struct device *dev)
{
	const struct mdio_gd32_config *cfg = dev->config;
	struct mdio_gd32_data *d = dev->data;
	int ret;

	ret = gd32_enet_platform_enable_clocks(&cfg->platform);
	if (ret != 0) {
		return ret;
	}

	ret = gd32_enet_platform_configure_phy(&cfg->platform);
	if (ret == 0) {
		ret = mdio_gd32_calc_mdc_clock(dev, &d->clk_bits);
		if (ret == 0) {
			mdio_gd32_reg_write(cfg, MDIO_GD32_MAC_PHY_CTL_OFFSET, d->clk_bits);
		}
	}

	return ret;
}

static uint32_t mdio_gd32_clause22_ctl(const struct mdio_gd32_data *data, uint8_t prtad,
				       uint8_t regad, bool write)
{
	uint32_t ctl = data->clk_bits | MAC_PHY_CTL_PA(prtad & 0x1FU) | MAC_PHY_CTL_PR(regad & 0x1FU);

	if (write) {
		ctl |= ENET_MAC_PHY_CTL_PW;
	}

	return ctl | ENET_MAC_PHY_CTL_PB;
}

static int mdio_gd32_clause22_transfer(const struct device *dev, uint8_t prtad, uint8_t regad,
				       uint16_t *data, bool write)
{
	const struct mdio_gd32_config *cfg = dev->config;
	const struct mdio_gd32_data *mdio_data = dev->data;
	int ret;

	/*
	 * ENET resets clear MAC_PHY_CTL, including the MDC divider selection.
	 * Re-apply the cached divider in the same write that starts the
	 * transaction so the bus never sees a divider-only register update.
	 */
	ret = mdio_gd32_wait_ready(dev);
	if (ret != 0) {
		return ret;
	}

	if (write) {
		mdio_gd32_reg_write(cfg, MDIO_GD32_MAC_PHY_DATA_OFFSET, *data);
	}

	mdio_gd32_reg_write(cfg, MDIO_GD32_MAC_PHY_CTL_OFFSET,
			    mdio_gd32_clause22_ctl(mdio_data, prtad, regad, write));

	ret = mdio_gd32_wait_ready(dev);
	if (ret != 0) {
		return ret;
	}

	if (!write) {
		*data = mdio_gd32_reg_read(cfg, MDIO_GD32_MAC_PHY_DATA_OFFSET) & UINT16_MAX;
	}

	return 0;
}

static int mdio_gd32_read(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t *data)
{
	struct mdio_gd32_data *data_ctx = dev->data;
	int ret;

	k_sem_take(&data_ctx->sem, K_FOREVER);
	ret = mdio_gd32_clause22_transfer(dev, prtad, regad, data, false);
	k_sem_give(&data_ctx->sem);

	return ret;
}

static int mdio_gd32_write(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t data)
{
	struct mdio_gd32_data *data_ctx = dev->data;
	int ret;

	k_sem_take(&data_ctx->sem, K_FOREVER);
	ret = mdio_gd32_clause22_transfer(dev, prtad, regad, &data, true);
	k_sem_give(&data_ctx->sem);

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

#define MDIO_GD32_DEVICE(inst)                                                                     \
	PINCTRL_DT_INST_DEFINE(inst);                                                            \
                                                                                                   \
	static const struct mdio_gd32_config mdio_gd32_config_##inst = {                         \
		.base = DT_INST_REG_ADDR(inst),                                                  \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                  \
		.platform = {                                                                    \
			.mac_clk = DT_INST_CLOCKS_CELL_BY_IDX(inst, 0, id),                      \
			.tx_clk = DT_INST_CLOCKS_CELL_BY_IDX(inst, 1, id),                       \
			.rx_clk = DT_INST_CLOCKS_CELL_BY_IDX(inst, 2, id),                       \
			.phy_mode_mii = DT_ENUM_HAS_VALUE(DT_INST_PARENT(inst),                  \
							  phy_connection_type, mii), \
			.phy_clk_internal = DT_ENUM_HAS_VALUE(DT_INST_PARENT(inst),              \
							      phy_clock_type, internal), \
		},                                                                               \
		.mdc_freq = DT_INST_PROP(inst, clock_frequency),                                 \
	};                                                                                         \
                                                                                                   \
	static struct mdio_gd32_data mdio_gd32_data_##inst;                                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, mdio_gd32_init, NULL, &mdio_gd32_data_##inst,                \
			      &mdio_gd32_config_##inst, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,  \
			      &mdio_gd32_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_GD32_DEVICE)
