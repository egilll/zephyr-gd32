/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC2_VENDOR_QUIRKS_H
#define ZEPHYR_DRIVERS_USB_UDC_DWC2_VENDOR_QUIRKS_H

#include "udc_dwc2.h"

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/usb/udc.h>

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32f4_fsotg)

#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <usb_dwc2_hw.h>

struct usb_dw_stm32_clk {
	const struct device *const dev;
	const struct stm32_pclken *const pclken;
	size_t pclken_len;
};

static inline int stm32f4_fsotg_enable_clk(const struct usb_dw_stm32_clk *const clk)
{
	int ret;

	if (!device_is_ready(clk->dev)) {
		return -ENODEV;
	}

	if (clk->pclken_len > 1) {
		uint32_t clk_rate;

		ret = clock_control_configure(clk->dev,
					      (void *)&clk->pclken[1],
					      NULL);
		if (ret) {
			return ret;
		}

		ret = clock_control_get_rate(clk->dev,
					     (void *)&clk->pclken[1],
					     &clk_rate);
		if (ret) {
			return ret;
		}

		if (clk_rate != MHZ(48)) {
			return -ENOTSUP;
		}
	}

	return clock_control_on(clk->dev, (void *)&clk->pclken[0]);
}

static inline int stm32f4_fsotg_enable_phy(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	mem_addr_t ggpio_reg = (mem_addr_t)&config->base->ggpio;

	sys_set_bits(ggpio_reg, USB_DWC2_GGPIO_STM32_PWRDWN | USB_DWC2_GGPIO_STM32_VBDEN);

	return 0;
}

static inline int stm32f4_fsotg_disable_phy(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	mem_addr_t ggpio_reg = (mem_addr_t)&config->base->ggpio;

	sys_clear_bits(ggpio_reg, USB_DWC2_GGPIO_STM32_PWRDWN | USB_DWC2_GGPIO_STM32_VBDEN);

	return 0;
}

#define QUIRK_STM32F4_FSOTG_DEFINE(n)						\
	static const struct stm32_pclken pclken_##n[] = STM32_DT_INST_CLOCKS(n);\
										\
	static const struct usb_dw_stm32_clk stm32f4_clk_##n = {		\
		.dev = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),			\
		.pclken = pclken_##n,						\
		.pclken_len = DT_INST_NUM_CLOCKS(n),				\
	};									\
										\
	static int stm32f4_fsotg_enable_clk_##n(const struct device *dev)	\
	{									\
		return stm32f4_fsotg_enable_clk(&stm32f4_clk_##n);		\
	}									\
										\
	const struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {		\
		.pre_enable = stm32f4_fsotg_enable_clk_##n,			\
		.post_enable = stm32f4_fsotg_enable_phy,			\
		.disable = stm32f4_fsotg_disable_phy,				\
		.irq_clear = NULL,						\
	};


DT_INST_FOREACH_STATUS_OKAY(QUIRK_STM32F4_FSOTG_DEFINE)

#endif /*DT_HAS_COMPAT_STATUS_OKAY(st_stm32f4_fsotg) */

#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_usbhs)

#include <zephyr/logging/log.h>
#include <nrfs_backend_ipc_service.h>
#include <nrfs_usb.h>

#define USBHS_DT_WRAPPER_REG_ADDR(n) UINT_TO_POINTER(DT_INST_REG_ADDR_BY_NAME(n, wrapper))

/*
 * On USBHS, we cannot access the DWC2 register until VBUS is detected and
 * valid. If the user tries to force usbd_enable() and the corresponding
 * udc_enable() without a "VBUS ready" notification, the event wait will block
 * until a valid VBUS signal is detected or until the
 * CONFIG_UDC_DWC2_USBHS_VBUS_READY_TIMEOUT timeout expires.
 */
static K_EVENT_DEFINE(usbhs_events);
#define USBHS_VBUS_READY	BIT(0)

static void usbhs_vbus_handler(nrfs_usb_evt_t const *p_evt, void *const context)
{
	LOG_MODULE_DECLARE(udc_dwc2, CONFIG_UDC_DRIVER_LOG_LEVEL);
	const struct device *dev = context;

	switch (p_evt->type) {
	case NRFS_USB_EVT_VBUS_STATUS_CHANGE:
		LOG_DBG("USBHS new status, pll_ok = %d vreg_ok = %d vbus_detected = %d",
			p_evt->usbhspll_ok, p_evt->vregusb_ok, p_evt->vbus_detected);

		if (p_evt->usbhspll_ok && p_evt->vregusb_ok && p_evt->vbus_detected) {
			k_event_post(&usbhs_events, USBHS_VBUS_READY);
			udc_submit_event(dev, UDC_EVT_VBUS_READY, 0);
		} else {
			k_event_set_masked(&usbhs_events, 0, USBHS_VBUS_READY);
			udc_submit_event(dev, UDC_EVT_VBUS_REMOVED, 0);
		}

		break;
	case NRFS_USB_EVT_REJECT:
		LOG_ERR("Request rejected");
		break;
	default:
		LOG_ERR("Unknown event type 0x%x", p_evt->type);
		break;
	}
}

static inline int usbhs_enable_nrfs_service(const struct device *dev)
{
	LOG_MODULE_DECLARE(udc_dwc2, CONFIG_UDC_DRIVER_LOG_LEVEL);
	nrfs_err_t nrfs_err;
	int err;

	err = nrfs_backend_wait_for_connection(K_MSEC(1000));
	if (err) {
		LOG_INF("NRFS backend connection timeout");
		return err;
	}

	nrfs_err = nrfs_usb_init(usbhs_vbus_handler);
	if (nrfs_err != NRFS_SUCCESS) {
		LOG_ERR("Failed to init NRFS VBUS handler: %d", nrfs_err);
		return -EIO;
	}

	nrfs_err = nrfs_usb_enable_request((void *)dev);
	if (nrfs_err != NRFS_SUCCESS) {
		LOG_ERR("Failed to enable NRFS VBUS service: %d", nrfs_err);
		return -EIO;
	}

	return 0;
}

static inline int usbhs_enable_core(const struct device *dev)
{
	LOG_MODULE_DECLARE(udc_dwc2, CONFIG_UDC_DRIVER_LOG_LEVEL);
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);
	k_timeout_t timeout = K_FOREVER;

	#if CONFIG_NRFS_HAS_VBUS_DETECTOR_SERVICE
	if (CONFIG_UDC_DWC2_USBHS_VBUS_READY_TIMEOUT) {
		timeout = K_MSEC(CONFIG_UDC_DWC2_USBHS_VBUS_READY_TIMEOUT);
	}
	#endif

	if (!k_event_wait(&usbhs_events, USBHS_VBUS_READY, false, K_NO_WAIT)) {
		LOG_WRN("VBUS is not ready, block udc_enable()");
		if (!k_event_wait(&usbhs_events, USBHS_VBUS_READY, false, timeout)) {
			return -ETIMEDOUT;
		}
	}

	wrapper->ENABLE = USBHS_ENABLE_PHY_Msk | USBHS_ENABLE_CORE_Msk;

	/* Wait for PHY clock to start */
	k_busy_wait(45);

	/* Release DWC2 reset */
	wrapper->TASKS_START = 1UL;

	/* Wait for clock to start to avoid hang on too early register read */
	k_busy_wait(1);

	/* Enable interrupts */
	wrapper->INTENSET = 1UL;

	return 0;
}

static inline int usbhs_enable_pullup(const struct device *dev)
{
	/* Core is ready to handle connection, enable D+ pull-up */
	nrfs_usb_dplus_pullup_enable((void *)dev);

	return 0;
}

static inline int usbhs_disable_core(const struct device *dev)
{
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);

	/* Disable D+ pull-up until next post enable quirk */
	nrfs_usb_dplus_pullup_disable((void *)dev);

	/* Disable interrupts */
	wrapper->INTENCLR = 1UL;

	wrapper->ENABLE = 0UL;

	return 0;
}

static inline int usbhs_disable_nrfs_service(const struct device *dev)
{
	LOG_MODULE_DECLARE(udc_dwc2, CONFIG_UDC_DRIVER_LOG_LEVEL);
	nrfs_err_t nrfs_err;

	nrfs_err = nrfs_usb_disable_request((void *)dev);
	if (nrfs_err != NRFS_SUCCESS) {
		LOG_ERR("Failed to disable NRFS VBUS service: %d", nrfs_err);
		return -EIO;
	}

	nrfs_usb_uninit();

	return 0;
}

static inline int usbhs_irq_clear(const struct device *dev)
{
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);

	wrapper->EVENTS_CORE = 0UL;

	return 0;
}

static inline int usbhs_init_caps(const struct device *dev)
{
	struct udc_data *data = dev->data;

	data->caps.can_detect_vbus = true;
	data->caps.hs = true;

	return 0;
}

static inline int usbhs_is_phy_clk_off(const struct device *dev)
{
	return !k_event_test(&usbhs_events, USBHS_VBUS_READY);
}

static inline int usbhs_post_hibernation_entry(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = config->base;
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);

	sys_set_bits((mem_addr_t)&base->pcgcctl, USB_DWC2_PCGCCTL_GATEHCLK);

	sys_write32(0x87, (mem_addr_t)wrapper + 0xC80);
	sys_write32(0x87, (mem_addr_t)wrapper + 0xC84);
	sys_write32(1, (mem_addr_t)wrapper + 0x004);

	return 0;
}

static inline int usbhs_pre_hibernation_exit(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = config->base;
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);

	sys_clear_bits((mem_addr_t)&base->pcgcctl, USB_DWC2_PCGCCTL_GATEHCLK);

	wrapper->TASKS_START = 1;
	sys_write32(0, (mem_addr_t)wrapper + 0xC80);
	sys_write32(0, (mem_addr_t)wrapper + 0xC84);

	return 0;
}

#define QUIRK_NRF_USBHS_DEFINE(n)						\
	const struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {		\
		.init = usbhs_enable_nrfs_service,				\
		.pre_enable = usbhs_enable_core,				\
		.post_enable = usbhs_enable_pullup,				\
		.disable = usbhs_disable_core,					\
		.shutdown = usbhs_disable_nrfs_service,				\
		.irq_clear = usbhs_irq_clear,					\
		.caps = usbhs_init_caps,					\
		.is_phy_clk_off = usbhs_is_phy_clk_off,				\
		.post_hibernation_entry = usbhs_post_hibernation_entry,		\
		.pre_hibernation_exit = usbhs_pre_hibernation_exit,		\
	};

DT_INST_FOREACH_STATUS_OKAY(QUIRK_NRF_USBHS_DEFINE)

#endif /*DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_usbhs) */

#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_usbhs_nrf54l)

#define USBHS_DT_WRAPPER_REG_ADDR(n) UINT_TO_POINTER(DT_INST_REG_ADDR_BY_NAME(n, wrapper))

#include <nrfx.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#define NRF_DEFAULT_IRQ_PRIORITY 1

/*
 * On USBHS, we cannot access the DWC2 register until VBUS is detected and
 * valid. If the user tries to force usbd_enable() and the corresponding
 * udc_enable() without a "VBUS ready" notification, the event wait will block
 * until a valid VBUS signal is detected or until the
 * CONFIG_UDC_DWC2_USBHS_VBUS_READY_TIMEOUT timeout expires.
 */
static K_EVENT_DEFINE(usbhs_events);
#define USBHS_VBUS_READY	BIT(0)

static struct onoff_manager *pclk24m_mgr;
static struct onoff_client pclk24m_cli;

static void vregusb_isr(const void *arg)
{
	const struct device *dev = arg;

	if (NRF_VREGUSB->EVENTS_VBUSDETECTED) {
		NRF_VREGUSB->EVENTS_VBUSDETECTED = 0;
		k_event_post(&usbhs_events, USBHS_VBUS_READY);
		udc_submit_event(dev, UDC_EVT_VBUS_READY, 0);
	}

	if (NRF_VREGUSB->EVENTS_VBUSREMOVED) {
		NRF_VREGUSB->EVENTS_VBUSREMOVED = 0;
		k_event_set_masked(&usbhs_events, 0, USBHS_VBUS_READY);
		udc_submit_event(dev, UDC_EVT_VBUS_REMOVED, 0);
	}
}

static inline int usbhs_init_vreg_and_clock(const struct device *dev)
{
	IRQ_CONNECT(VREGUSB_IRQn, NRF_DEFAULT_IRQ_PRIORITY,
		    vregusb_isr, DEVICE_DT_INST_GET(0), 0);

	NRF_VREGUSB->INTEN = VREGUSB_INTEN_VBUSDETECTED_Msk |
			     VREGUSB_INTEN_VBUSREMOVED_Msk;
	NRF_VREGUSB->TASKS_START = 1;

	/* TODO: Determine conditions when VBUSDETECTED is not generated */
	if (sys_read32((mem_addr_t)NRF_VREGUSB + 0x400) & BIT(2)) {
		k_event_post(&usbhs_events, USBHS_VBUS_READY);
		udc_submit_event(dev, UDC_EVT_VBUS_READY, 0);
	}

	irq_enable(VREGUSB_IRQn);
	pclk24m_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF24M);

	return 0;
}

static inline int usbhs_enable_core(const struct device *dev)
{
	LOG_MODULE_DECLARE(udc_dwc2, CONFIG_UDC_DRIVER_LOG_LEVEL);
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);
	k_timeout_t timeout = K_FOREVER;
	int err;

	if (CONFIG_UDC_DWC2_USBHS_VBUS_READY_TIMEOUT) {
		timeout = K_MSEC(CONFIG_UDC_DWC2_USBHS_VBUS_READY_TIMEOUT);
	}

	if (!k_event_wait(&usbhs_events, USBHS_VBUS_READY, false, K_NO_WAIT)) {
		LOG_WRN("VBUS is not ready, block udc_enable()");
		if (!k_event_wait(&usbhs_events, USBHS_VBUS_READY, false, timeout)) {
			return -ETIMEDOUT;
		}
	}

	/* Request PCLK24M using clock control driver */
	sys_notify_init_spinwait(&pclk24m_cli.notify);
	err = onoff_request(pclk24m_mgr, &pclk24m_cli);
	if (err < 0) {
		LOG_ERR("Failed to start PCLK24M %d", err);
		return err;
	}

	/* Power up peripheral */
	wrapper->ENABLE = USBHS_ENABLE_CORE_Msk;

	/* Set ID to Device and force D+ pull-up off for now */
	wrapper->PHY.OVERRIDEVALUES = (1 << 31);
	wrapper->PHY.INPUTOVERRIDE = (1 << 31) | USBHS_PHY_INPUTOVERRIDE_VBUSVALID_Msk;

	/* Release PHY power-on reset */
	wrapper->ENABLE = USBHS_ENABLE_PHY_Msk | USBHS_ENABLE_CORE_Msk;

	/* Wait for PHY clock to start */
	k_busy_wait(45);

	/* Release DWC2 reset */
	wrapper->TASKS_START = 1UL;

	/* Wait for clock to start to avoid hang on too early register read */
	k_busy_wait(1);

	/* DWC2 opmode is now guaranteed to be Non-Driving, allow D+ pull-up to
	 * become active once driver clears DCTL SftDiscon bit.
	 */
	wrapper->PHY.INPUTOVERRIDE = (1 << 31);

	return 0;
}

static inline int usbhs_disable_core(const struct device *dev)
{
	LOG_MODULE_DECLARE(udc_dwc2, CONFIG_UDC_DRIVER_LOG_LEVEL);
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);
	int err;

	/* Set ID to Device and forcefully disable D+ pull-up */
	wrapper->PHY.OVERRIDEVALUES = (1 << 31);
	wrapper->PHY.INPUTOVERRIDE = (1 << 31) | USBHS_PHY_INPUTOVERRIDE_VBUSVALID_Msk;

	wrapper->ENABLE = 0UL;

	/* Release PCLK24M using clock control driver */
	err = onoff_cancel_or_release(pclk24m_mgr, &pclk24m_cli);
	if (err < 0) {
		LOG_ERR("Failed to stop PCLK24M %d", err);
		return err;
	}

	return 0;
}

static inline int usbhs_disable_vreg(const struct device *dev)
{
	NRF_VREGUSB->INTEN = 0;
	NRF_VREGUSB->TASKS_STOP = 1;

	return 0;
}

static inline int usbhs_init_caps(const struct device *dev)
{
	struct udc_data *data = dev->data;

	data->caps.can_detect_vbus = true;
	data->caps.hs = true;

	return 0;
}

static inline int usbhs_is_phy_clk_off(const struct device *dev)
{
	return !k_event_test(&usbhs_events, USBHS_VBUS_READY);
}

static inline int usbhs_post_hibernation_entry(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = config->base;
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);

	sys_set_bits((mem_addr_t)&base->pcgcctl, USB_DWC2_PCGCCTL_GATEHCLK);

	wrapper->TASKS_STOP = 1;

	return 0;
}

static inline int usbhs_pre_hibernation_exit(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = config->base;
	NRF_USBHS_Type *wrapper = USBHS_DT_WRAPPER_REG_ADDR(0);

	sys_clear_bits((mem_addr_t)&base->pcgcctl, USB_DWC2_PCGCCTL_GATEHCLK);

	wrapper->TASKS_START = 1;

	return 0;
}

#define QUIRK_NRF_USBHS_DEFINE(n)						\
	struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {			\
		.init = usbhs_init_vreg_and_clock,				\
		.pre_enable = usbhs_enable_core,				\
		.disable = usbhs_disable_core,					\
		.shutdown = usbhs_disable_vreg,					\
		.caps = usbhs_init_caps,					\
		.is_phy_clk_off = usbhs_is_phy_clk_off,				\
		.post_hibernation_entry = usbhs_post_hibernation_entry,		\
		.pre_hibernation_exit = usbhs_pre_hibernation_exit,		\
	};

DT_INST_FOREACH_STATUS_OKAY(QUIRK_NRF_USBHS_DEFINE)

#endif /*DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_usbhs_nrf54l) */

#if DT_HAS_COMPAT_STATUS_OKAY(espressif_esp32_usb_otg)

#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>

#include <esp_err.h>
#include <esp_private/usb_phy.h>
#include <hal/usb_wrap_hal.h>
#include <soc/usb_dwc_periph.h>

#include <esp_rom_gpio.h>
#include <driver/gpio.h>
#include <soc/usb_pins.h>
#include <soc/gpio_sig_map.h>

struct phy_context_t {
	usb_phy_target_t target;
	usb_phy_controller_t controller;
	usb_phy_status_t status;
	usb_otg_mode_t otg_mode;
	usb_phy_speed_t otg_speed;
	usb_phy_ext_io_conf_t *iopins;
	usb_wrap_hal_context_t wrap_hal;
};

struct usb_dw_esp32_config {
	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	int irq_source;
	int irq_priority;
	int irq_flags;
	struct phy_context_t *phy_ctx;
};

struct usb_dw_esp32_data {
	struct intr_handle_data_t *int_handle;
};

static void udc_dwc2_isr_handler(const struct device *dev);

static inline int esp32_usb_otg_init(const struct device *dev,
				     const struct usb_dw_esp32_config *cfg,
				     struct usb_dw_esp32_data *data)
{
	int ret;

	if (!device_is_ready(cfg->clock_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);

	if (ret != 0) {
		return ret;
	}

	/* pinout config to work in USB_OTG_MODE_DEVICE */
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, USB_OTG_IDDIG_IN_IDX, false);
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, USB_SRP_BVALID_IN_IDX, false);
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, USB_OTG_VBUSVALID_IN_IDX,
				       false);
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, USB_OTG_AVALID_IN_IDX, false);

	if (cfg->phy_ctx->target == USB_PHY_TARGET_INT) {
		gpio_set_drive_capability(USBPHY_DM_NUM, GPIO_DRIVE_CAP_3);
		gpio_set_drive_capability(USBPHY_DP_NUM, GPIO_DRIVE_CAP_3);
	}

	/* allocate interrupt but keep it disabled to avoid
	 * spurious suspend/resume event at enumeration phase
	 */
	ret = esp_intr_alloc(cfg->irq_source,
			     ESP_INTR_FLAG_INTRDISABLED |
			     ESP_PRIO_TO_FLAGS(cfg->irq_priority) |
				     ESP_INT_FLAGS_CHECK(cfg->irq_flags),
			     (intr_handler_t)udc_dwc2_isr_handler, (void *)dev, &data->int_handle);

	return ret;
}

static inline int esp32_usb_otg_enable_clk(struct phy_context_t *phy_ctx)
{
	usb_wrap_ll_enable_bus_clock(true);

	usb_wrap_ll_reset_register();
	usb_wrap_hal_init(&phy_ctx->wrap_hal);

#if USB_WRAP_LL_EXT_PHY_SUPPORTED
	usb_wrap_hal_phy_set_external(&phy_ctx->wrap_hal, (phy_ctx->target == USB_PHY_TARGET_EXT));
#endif

	return 0;
}

static inline int esp32_usb_otg_enable_phy(struct phy_context_t *phy_ctx, bool enable)
{
	LOG_MODULE_DECLARE(udc_dwc2, CONFIG_UDC_DRIVER_LOG_LEVEL);

	if (enable) {
		usb_wrap_ll_phy_enable_pad(phy_ctx->wrap_hal.dev, true);
		LOG_DBG("PHY enabled");
	} else {
		usb_wrap_ll_phy_enable_pad(phy_ctx->wrap_hal.dev, false);
		LOG_DBG("PHY disabled");
	}

	return 0;
}

static inline int esp32_usb_otg_shutdown(struct phy_context_t *phy_ctx)
{
	usb_wrap_ll_enable_bus_clock(false);

	return 0;
}

#define QUIRK_ESP32_USB_OTG_DEFINE(n)						\
										\
	static struct phy_context_t phy_ctx_##n = {				\
		.target = USB_PHY_TARGET_INT,					\
		.controller = USB_PHY_CTRL_OTG,					\
		.otg_mode = USB_OTG_MODE_DEVICE,				\
		.otg_speed = USB_PHY_SPEED_FULL,				\
		.iopins = NULL,							\
		.wrap_hal = {},							\
	};									\
										\
	static const struct usb_dw_esp32_config usb_otg_config_##n = {		\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		\
		.clock_subsys = (clock_control_subsys_t)			\
			DT_INST_CLOCKS_CELL(n, offset),				\
		.irq_source = DT_INST_IRQ_BY_IDX(n, 0, irq),			\
		.irq_priority = DT_INST_IRQ_BY_IDX(n, 0, priority),		\
		.irq_flags = DT_INST_IRQ_BY_IDX(n, 0, flags),			\
		.phy_ctx = &phy_ctx_##n,					\
	};									\
										\
	static struct usb_dw_esp32_data usb_otg_data_##n;			\
										\
	static int esp32_usb_otg_init_##n(const struct device *dev)		\
	{									\
		return esp32_usb_otg_init(dev,					\
			&usb_otg_config_##n, &usb_otg_data_##n);		\
	}									\
										\
	static int esp32_usb_otg_enable_clk_##n(const struct device *dev)	\
	{									\
		return esp32_usb_otg_enable_clk(&phy_ctx_##n);			\
	}									\
										\
	static int esp32_usb_otg_enable_phy_##n(const struct device *dev)	\
	{									\
		return esp32_usb_otg_enable_phy(&phy_ctx_##n, true);		\
	}									\
										\
	static int esp32_usb_otg_disable_phy_##n(const struct device *dev)	\
	{									\
		return esp32_usb_otg_enable_phy(&phy_ctx_##n, false);		\
	}									\
	static int esp32_usb_otg_shutdown_##n(const struct device *dev)		\
	{									\
		esp_intr_free(usb_otg_data_##n.int_handle);			\
		return esp32_usb_otg_shutdown(&phy_ctx_##n);			\
	}									\
										\
	const struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {		\
		.init = esp32_usb_otg_init_##n,					\
		.pre_enable = esp32_usb_otg_enable_clk_##n,			\
		.post_enable = esp32_usb_otg_enable_phy_##n,			\
		.disable = esp32_usb_otg_disable_phy_##n,			\
		.shutdown = esp32_usb_otg_shutdown_##n,				\
	};									\

#define UDC_DWC2_IRQ_DT_INST_DEFINE(n)						\
	static void udc_dwc2_irq_enable_func_##n(const struct device *dev)	\
	{									\
		esp_intr_enable(usb_otg_data_##n.int_handle);			\
	}									\
										\
	static void udc_dwc2_irq_disable_func_##n(const struct device *dev)	\
	{									\
		esp_intr_disable(usb_otg_data_##n.int_handle);			\
	}

DT_INST_FOREACH_STATUS_OKAY(QUIRK_ESP32_USB_OTG_DEFINE)

#endif /*DT_HAS_COMPAT_STATUS_OKAY(espressif_esp32_usb_otg) */


#if DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbfs) || DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbhs)

#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/dt-bindings/clock/gd32f4xx-clocks.h>
#if defined(CONFIG_SOC_SERIES_GD32F4XX)
#include <gd32f4xx.h>
#include <gd32f4xx_rcu.h>
#endif

/* USBFS/USBHS Global Core Configuration register (GCCFG), offset 0x0038. */
#define USB_DWC2_GCCFG_GD32_PWRON	BIT(16)
#define USB_DWC2_GCCFG_GD32_VBUSACEN	BIT(18)
#define USB_DWC2_GCCFG_GD32_VBUSBCEN	BIT(19)
#define USB_DWC2_GCCFG_GD32_VBUSIG	BIT(21)
/* GD32 uses GUSBCFG[13:10] as USB turnaround time, vendor USBFS code sets 5. */
#define USB_DWC2_GUSBCFG_GD32_UTT_MASK	GENMASK(13, 10)
#define USB_DWC2_GUSBCFG_GD32_UTT_FS	(5U << 10)

/* RCU_ADDCTL bits used to provide a stable 48 MHz clock for the embedded PHY. */
#define GD32_RCU_ADDCTL_IRC48MEN	BIT(16)
#define GD32_RCU_ADDCTL_IRC48MSTB	BIT(17)

static inline int gd32_usb_wait_irc48m_stable(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(rcu))
	mem_addr_t addctl_reg = (mem_addr_t)(DT_REG_ADDR(DT_NODELABEL(rcu)) + GD32_ADDCTL_OFFSET);
	k_timepoint_t timeout;

	/* If IRC48M is not enabled, there is nothing to wait for. */
	if ((sys_read32(addctl_reg) & GD32_RCU_ADDCTL_IRC48MEN) == 0U) {
		return 0;
	}

	timeout = sys_timepoint_calc(K_MSEC(10));
	while ((sys_read32(addctl_reg) & GD32_RCU_ADDCTL_IRC48MSTB) == 0U) {
		if (sys_timepoint_expired(timeout)) {
			return -ETIMEDOUT;
		}

		k_usleep(50);
	}
#endif

	return 0;
}

#if defined(CONFIG_SOC_SERIES_GD32F4XX)
static inline int gd32f4xx_wait_flag(rcu_flag_enum flag, FlagStatus target, uint32_t timeout_ms)
{
	k_timepoint_t timeout = sys_timepoint_calc(K_MSEC(timeout_ms));

	while (rcu_flag_get(flag) != target) {
		if (sys_timepoint_expired(timeout)) {
			return -ETIMEDOUT;
		}

		k_usleep(50);
	}

	return 0;
}

static inline int gd32f4xx_usbfs_configure_ck48m_from_pllsai(void)
{
	uint32_t pllm = RCU_PLL & RCU_PLL_PLLPSC;
	uint32_t input_rate_hz;
	uint32_t pllsai_n;
	int ret;

	if (pllm == 0U) {
		return -EINVAL;
	}

	if ((RCU_PLL & RCU_PLL_PLLSEL) == RCU_PLLSRC_HXTAL) {
		rcu_osci_on(RCU_HXTAL);
		ret = gd32f4xx_wait_flag(RCU_FLAG_HXTALSTB, SET, 100);
		if (ret < 0) {
			return ret;
		}

		input_rate_hz = HXTAL_VALUE / pllm;
	} else {
		ret = gd32f4xx_wait_flag(RCU_FLAG_IRC16MSTB, SET, 100);
		if (ret < 0) {
			return ret;
		}

		input_rate_hz = IRC16M_VALUE / pllm;
	}

	if ((input_rate_hz == 0U) || ((288000000U % input_rate_hz) != 0U)) {
		return -EINVAL;
	}

	pllsai_n = 288000000U / input_rate_hz;
	if ((pllsai_n < RCU_PLLSAIN_MUL_MIN) || (pllsai_n > RCU_PLLSAIN_MUL_MAX)) {
		return -EINVAL;
	}

	rcu_osci_off(RCU_PLLSAI_CK);
	(void)gd32f4xx_wait_flag(RCU_FLAG_PLLSAISTB, RESET, 10);

	if (rcu_pllsai_config(pllsai_n, 6U, 2U) != SUCCESS) {
		return -EINVAL;
	}

	rcu_osci_on(RCU_PLLSAI_CK);
	ret = gd32f4xx_wait_flag(RCU_FLAG_PLLSAISTB, SET, 100);
	if (ret < 0) {
		return ret;
	}

	rcu_pll48m_clock_config(RCU_PLL48MSRC_PLLSAIP);
	rcu_ck48m_clock_config(RCU_CK48MSRC_PLL48M);

	return 0;
}
#endif

static inline int gd32_usb_enable_embedded_phy(const struct device *dev, bool vbus_sensing)
{
	const struct udc_dwc2_config *const config = dev->config;
	mem_addr_t gccfg_reg = (mem_addr_t)&config->base->ggpio;
	mem_addr_t gusbcfg_reg = (mem_addr_t)&config->base->gusbcfg;
	uint32_t gccfg_bits = USB_DWC2_GCCFG_GD32_PWRON |
			      USB_DWC2_GCCFG_GD32_VBUSACEN |
			      USB_DWC2_GCCFG_GD32_VBUSBCEN;
	uint32_t gusbcfg;

	/*
	 * Reference: GD32_28_USBFS.md and GD32_29_USBHS.md document USBx_GCCFG
	 * fields (PWRON/VBUSACEN/VBUSBCEN/VBUSIG) and that VBUSIG can be set to
	 * ignore VBUS in device mode.
	 */
	if (!vbus_sensing) {
		gccfg_bits |= USB_DWC2_GCCFG_GD32_VBUSIG;
	}

	sys_set_bits(gccfg_reg, gccfg_bits);

	/*
	 * The GD32 vendor USBFS device driver programs the embedded-FS PHY
	 * turnaround time to 5. Leaving it at reset value 0 causes reset and
	 * speed detection to work, but control traffic never becomes valid.
	 */
	gusbcfg = sys_read32(gusbcfg_reg);
	gusbcfg &= ~USB_DWC2_GUSBCFG_GD32_UTT_MASK;
	gusbcfg |= USB_DWC2_GUSBCFG_GD32_UTT_FS;
	sys_write32(gusbcfg, gusbcfg_reg);

	/* Vendor reference uses ~20ms delay after transceiver power-on. */
	k_msleep(20);

	return 0;
}

static inline int gd32_usb_disable_embedded_phy(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	mem_addr_t gccfg_reg = (mem_addr_t)&config->base->ggpio;

	sys_clear_bits(gccfg_reg, USB_DWC2_GCCFG_GD32_PWRON);

	return 0;
}

static inline int gd32_usbhs_init_caps(const struct device *dev)
{
	struct udc_data *data = dev->data;

	data->caps.hs = true;

	return 0;
}

/*
 * TODO EGILL!
 * GD32 USBFS appears to assert IN endpoint "NAK effective" (INEPNAKEFF) as a
 * level condition that can lead to an interrupt storm when masked/enabled like
 * other DWC2 integrations do. Mask it out and clear any latched INEPNAKEFF
 * flags after ISR handling.
 */
static inline int gd32_dwc2_irq_clear(const struct device *dev)
{
	const struct udc_dwc2_config *const config = dev->config;
	struct usb_dwc2_reg *const base = config->base;
	mem_addr_t diepmsk_reg = (mem_addr_t)&base->diepmsk;

	sys_clear_bits(diepmsk_reg, USB_DWC2_DIEPINT_INEPNAKEFF);

	/* Clear any latched INEPNAKEFF for all configured IN endpoints. */
	for (size_t i = 0; i < config->num_in_eps; i++) {
		sys_write32(USB_DWC2_DIEPINT_INEPNAKEFF,
			    (mem_addr_t)&base->in_ep[i].diepint);
	}

	return 0;
}

#define _GD32_DWC2_CLK_ON(node_id, prop, idx)					\
	do {									\
		uint16_t clk_id = DT_CLOCKS_CELL_BY_IDX(node_id, idx, id);	\
		ret = clock_control_on(GD32_CLOCK_CONTROLLER,			\
				       (clock_control_subsys_t)&clk_id);	\
		if (ret) {							\
			return ret;						\
		}								\
	} while (0);

#define QUIRK_GD32_USBFS_DEFINE(n)						\
	static const struct reset_dt_spec reset_##n =				\
		RESET_DT_SPEC_INST_GET_OR(n, {0});				\
										\
	static int gd32_usbfs_pre_enable_##n(const struct device *dev)		\
	{									\
		const struct udc_dwc2_config *const config = dev->config;	\
		int ret;							\
										\
		/* Enable all clocks listed in devicetree. */			\
		DT_INST_FOREACH_PROP_ELEM(n, clocks,				\
			_GD32_DWC2_CLK_ON)					\
										\
		ret = gd32_usb_wait_irc48m_stable();				\
		if (ret) {							\
			return ret;						\
		}								\
										\
		if (reset_##n.dev != NULL && device_is_ready(reset_##n.dev)) {	\
			ret = reset_line_toggle_dt(&reset_##n);			\
			if (ret && ret != -ENOSYS) {				\
				return ret;					\
			}							\
		}								\
										\
		/* GD32F4 USBFS examples drive CK48M from PLL48M/PLLSAIP. */	\
		IF_ENABLED(CONFIG_SOC_SERIES_GD32F4XX, (			\
			ret = gd32f4xx_usbfs_configure_ck48m_from_pllsai();	\
			if (ret) {						\
				return ret;					\
			}							\
		))								\
										\
		/* Ensure PHY clock is not gated. */				\
		sys_write32(0, (mem_addr_t)&config->base->pcgcctl);		\
		return 0;							\
	}									\
										\
	static int gd32_usbfs_post_enable_##n(const struct device *dev)		\
	{									\
		return gd32_usb_enable_embedded_phy(dev,				\
						    DT_INST_PROP(n, vbus_sensing));\
	}									\
										\
	const struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {		\
		.pre_enable = gd32_usbfs_pre_enable_##n,			\
		.post_enable = gd32_usbfs_post_enable_##n,			\
		.disable = gd32_usb_disable_embedded_phy,			\
		.irq_clear = gd32_dwc2_irq_clear,					\
	};

#define QUIRK_GD32_USBHS_DEFINE(n)						\
	static const struct reset_dt_spec reset_##n =				\
		RESET_DT_SPEC_INST_GET_OR(n, {0});				\
										\
	static int gd32_usbhs_pre_enable_##n(const struct device *dev)		\
	{									\
		const struct udc_dwc2_config *const config = dev->config;	\
		int ret;							\
										\
		/* Enable all clocks listed in devicetree. */			\
		DT_INST_FOREACH_PROP_ELEM(n, clocks,				\
			_GD32_DWC2_CLK_ON)					\
										\
		if (reset_##n.dev != NULL && device_is_ready(reset_##n.dev)) {	\
			ret = reset_line_toggle_dt(&reset_##n);			\
			if (ret && ret != -ENOSYS) {				\
				return ret;					\
			}							\
		}								\
										\
		/* Ensure PHY clock is not gated. */				\
		sys_write32(0, (mem_addr_t)&config->base->pcgcctl);		\
		return 0;							\
	}									\
										\
	static int gd32_usbhs_post_enable_##n(const struct device *dev)		\
	{									\
		bool ulpi = (DT_INST_NUM_CLOCKS(n) > 1) &&			\
			    (DT_INST_CLOCKS_CELL_BY_IDX(n, 1, id) == GD32_CLOCK_USBHSULPI);\
										\
		if (ulpi) {							\
			return 0;						\
		}								\
										\
		return gd32_usb_enable_embedded_phy(dev,				\
						    DT_INST_PROP(n, vbus_sensing));\
	}									\
										\
	const struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {		\
		.pre_enable = gd32_usbhs_pre_enable_##n,			\
		.post_enable = gd32_usbhs_post_enable_##n,			\
		.caps = gd32_usbhs_init_caps,					\
		.irq_clear = gd32_dwc2_irq_clear,					\
	};

#define QUIRK_GD32_DWC2_DEFINE(n)						\
	COND_CODE_1(DT_INST_NODE_HAS_COMPAT(n, gd_gd32_usbfs),			\
		    (QUIRK_GD32_USBFS_DEFINE(n)),				\
		    (COND_CODE_1(DT_INST_NODE_HAS_COMPAT(n, gd_gd32_usbhs),	\
				 (QUIRK_GD32_USBHS_DEFINE(n)), ())))

DT_INST_FOREACH_STATUS_OKAY(QUIRK_GD32_DWC2_DEFINE)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbfs) || DT_HAS_COMPAT_STATUS_OKAY(gd_gd32_usbhs) */

/* Add next vendor quirks definition above this line */

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC2_VENDOR_QUIRKS_H */
