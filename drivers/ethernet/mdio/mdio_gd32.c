/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_mdio

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <gd32f4xx_enet.h>
#include <gd32f4xx_rcu.h>
#include <gd32f4xx_gpio.h>
#include <gd32f4xx_syscfg.h>

LOG_MODULE_REGISTER(mdio_gd32, CONFIG_MDIO_LOG_LEVEL);

#define MDIO_GD32_OP_TIMEOUT_MS 10

struct mdio_gd32_config {
	const struct pinctrl_dev_config *pincfg;
	uint32_t mdc_freq;
	bool phy_clk_out;
	bool phy_mode_mii;
};

struct mdio_gd32_data {
	struct k_sem sem;
};

static int mdio_gd32_wait_ready(void)
{
	int64_t deadline = k_uptime_get() + MDIO_GD32_OP_TIMEOUT_MS;

	while ((ENET_MAC_PHY_CTL & ENET_MAC_PHY_CTL_PB) != 0U) {
		if (k_uptime_get() > deadline) {
			return -ETIMEDOUT;
		}

		k_usleep(10);
	}

	return 0;
}

static int mdio_gd32_clause22_read(uint8_t prtad, uint8_t regad, uint16_t *data)
{
	uint32_t clk_bits = ENET_MAC_PHY_CTL & ENET_MAC_PHY_CTL_CLR;
	int ret;

	ret = mdio_gd32_wait_ready();
	if (ret < 0) {
		return ret;
	}

	ENET_MAC_PHY_CTL = clk_bits | MAC_PHY_CTL_PA(prtad & 0x1FU) | MAC_PHY_CTL_PR(regad & 0x1FU) |
			   ENET_MAC_PHY_CTL_PB;

	ret = mdio_gd32_wait_ready();
	if (ret < 0) {
		return ret;
	}

	*data = (uint16_t)(ENET_MAC_PHY_DATA & 0xFFFFU);
	return 0;
}

static int mdio_gd32_clause22_write(uint8_t prtad, uint8_t regad, uint16_t data)
{
	uint32_t clk_bits = ENET_MAC_PHY_CTL & ENET_MAC_PHY_CTL_CLR;
	int ret;

	ret = mdio_gd32_wait_ready();
	if (ret < 0) {
		return ret;
	}

	ENET_MAC_PHY_DATA = (uint32_t)data;
	ENET_MAC_PHY_CTL = clk_bits | MAC_PHY_CTL_PA(prtad & 0x1FU) | MAC_PHY_CTL_PR(regad & 0x1FU) |
			   ENET_MAC_PHY_CTL_PW | ENET_MAC_PHY_CTL_PB;

	return mdio_gd32_wait_ready();
}

static void mdio_gd32_phy_clk_out_enable(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);

	gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_8);
	gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_8);

	rcu_ckout0_config(RCU_CKOUT0SRC_PLLP, RCU_CKOUT0_DIV4);
}

static void mdio_gd32_phy_interface_config(bool phy_mode_mii)
{
	rcu_periph_clock_enable(RCU_SYSCFG);

	syscfg_enet_phy_interface_config(phy_mode_mii ? SYSCFG_ENET_PHY_MII : SYSCFG_ENET_PHY_RMII);
}

static void mdio_gd32_prepare_hw(const struct mdio_gd32_config *cfg)
{
	mdio_gd32_phy_interface_config(cfg->phy_mode_mii);

	/* Ensure ENET is not held in reset so the MAC management station is accessible. */
	rcu_periph_reset_disable(RCU_ENETRST);

	if (cfg->phy_clk_out) {
		mdio_gd32_phy_clk_out_enable();
	}
}

static void mdio_gd32_set_mdc_clock(uint32_t mdc_freq)
{
	/* IEEE 802.3: MDC max is 2.5 MHz for clause 22. */
	if (mdc_freq == 0U || mdc_freq > 2500000U) {
		mdc_freq = 2500000U;
	}

	uint32_t ahbclk = rcu_clock_freq_get(CK_AHB);
	uint32_t reg = ENET_MAC_PHY_CTL;

	reg &= ~ENET_MAC_PHY_CTL_CLR;

	/*
	 * GD32 ENET MDC divider is selected by an encoded value:
	 * HCLK/(16, 26, 42, 62, 102). Pick the fastest divider that keeps
	 * MDC <= requested frequency.
	 */
	const struct {
		uint32_t div;
		uint32_t bits;
	} divs[] = {
		{16U, ENET_MDC_HCLK_DIV16},   {26U, ENET_MDC_HCLK_DIV26},
		{42U, ENET_MDC_HCLK_DIV42},   {62U, ENET_MDC_HCLK_DIV62},
		{102U, ENET_MDC_HCLK_DIV102},
	};

	uint32_t sel_bits = ENET_MDC_HCLK_DIV102;
	for (size_t i = 0U; i < ARRAY_SIZE(divs); i++) {
		if ((uint64_t)ahbclk <= (uint64_t)mdc_freq * (uint64_t)divs[i].div) {
			sel_bits = divs[i].bits;
			break;
		}
	}

	reg |= sel_bits;

	ENET_MAC_PHY_CTL = reg;
}

static void mdio_gd32_hw_enable(const struct device *dev)
{
	const struct mdio_gd32_config *cfg = dev->config;

	/*
	 * ENET clocks must be enabled for the management station interface.
	 * On GD32F4xx, enabling only RCU_ENET can leave the MDIO busy bit stuck
	 * (PB never clears); enable TX/RX domains as well.
	 */
	rcu_periph_clock_enable(RCU_ENET);
	rcu_periph_clock_enable(RCU_ENETTX);
	rcu_periph_clock_enable(RCU_ENETRX);

	k_sleep(K_MSEC(1));

	mdio_gd32_set_mdc_clock(cfg->mdc_freq);
}

static int mdio_gd32_read(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t *data)
{
	struct mdio_gd32_data *d = dev->data;
	uint16_t val = 0U;
	int ret;

	k_sem_take(&d->sem, K_FOREVER);
	ret = mdio_gd32_clause22_read(prtad, regad, &val);
	k_sem_give(&d->sem);

	if (ret < 0) {
		return ret;
	}

	*data = val;
	return 0;
}

static int mdio_gd32_write(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t data)
{
	struct mdio_gd32_data *d = dev->data;
	int ret;

	k_sem_take(&d->sem, K_FOREVER);
	ret = mdio_gd32_clause22_write(prtad, regad, data);
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

	mdio_gd32_prepare_hw(cfg);

	mdio_gd32_hw_enable(dev);

	k_sem_init(&d->sem, 1, 1);
	return 0;
}

static DEVICE_API(mdio, mdio_gd32_api) = {
	.read = mdio_gd32_read,
	.write = mdio_gd32_write,
};

#define MDIO_GD32_DEVICE(inst)                                                                     \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static struct mdio_gd32_config mdio_gd32_config_##inst = {                                 \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.mdc_freq = DT_INST_PROP(inst, clock_frequency),                                   \
		.phy_clk_out = DT_INST_PROP(inst, gd_phy_clk_out),                                  \
		.phy_mode_mii = DT_INST_ENUM_HAS_VALUE(inst, phy_connection_type, mii),             \
	};                                                                                         \
	static struct mdio_gd32_data mdio_gd32_data_##inst;                                        \
	DEVICE_DT_INST_DEFINE(inst, &mdio_gd32_init, NULL, &mdio_gd32_data_##inst,                 \
			      &mdio_gd32_config_##inst, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,    \
			      &mdio_gd32_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_GD32_DEVICE)
