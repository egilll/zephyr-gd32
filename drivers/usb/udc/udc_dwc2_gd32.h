/*
 * Copyright (c) 2026 Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC2_GD32_H
#define ZEPHYR_DRIVERS_USB_UDC_DWC2_GD32_H

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/dt-bindings/clock/gd32f4xx-clocks.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

/*
 * GD32 USBFS and USBHS are Synopsys DWC2 derivatives. The standard DWC2 GGPIO
 * register at offset 0x038 is overlaid as GCCFG, controlling the embedded PHY
 * and VBUS comparators. Reuse the ggpio field in struct usb_dwc2_reg.
 */
#define GD32_GCCFG_PWRON   BIT(16)
#define GD32_GCCFG_VBUSACEN BIT(18)
#define GD32_GCCFG_VBUSBCEN BIT(19)
#define GD32_GCCFG_VBUSIG BIT(21)

/* GUSBCFG[13:10] is the embedded-PHY turnaround time. */
#define GD32_GUSBCFG_UTT_MASK GENMASK(13, 10)
#define GD32_GUSBCFG_UTT_FS   (5U << 10)

static inline int gd32_usb_request_ck48m(void)
{
	struct gd32_usb48m_clock_config cfg = {
		.type = GD32_CLOCK_CONFIG_TYPE_USB48M,
	};

	return clock_control_configure(GD32_CLOCK_CONTROLLER, NULL, &cfg);
}

static inline void gd32_usb_select_embedded_phy(const struct device *dev, bool embedded_phy)
{
	struct usb_dwc2_reg *const base = dwc2_get_base(dev);
	mem_addr_t gusbcfg_reg = (mem_addr_t)&base->gusbcfg;

	if (embedded_phy) {
		sys_set_bits(gusbcfg_reg, USB_DWC2_GUSBCFG_PHYSEL_USB11);
	} else {
		sys_clear_bits(gusbcfg_reg, USB_DWC2_GUSBCFG_PHYSEL_USB11);
	}
}

static inline void gd32_usb_enable_embedded_phy(const struct device *dev, bool vbus_sensing,
						bool program_utt)
{
	struct usb_dwc2_reg *const base = dwc2_get_base(dev);
	mem_addr_t gccfg_reg = (mem_addr_t)&base->ggpio;
	uint32_t gccfg_bits = GD32_GCCFG_PWRON | GD32_GCCFG_VBUSACEN | GD32_GCCFG_VBUSBCEN;

	if (!vbus_sensing) {
		gccfg_bits |= GD32_GCCFG_VBUSIG;
	}

	sys_set_bits(gccfg_reg, gccfg_bits);

	if (program_utt) {
		mem_addr_t gusbcfg_reg = (mem_addr_t)&base->gusbcfg;
		uint32_t gusbcfg = sys_read32(gusbcfg_reg);

		gusbcfg = (gusbcfg & ~GD32_GUSBCFG_UTT_MASK) | GD32_GUSBCFG_UTT_FS;
		sys_write32(gusbcfg, gusbcfg_reg);
	}

	k_msleep(20);
}

static inline int gd32_usb_disable_embedded_phy(const struct device *dev)
{
	struct usb_dwc2_reg *const base = dwc2_get_base(dev);

	sys_clear_bits((mem_addr_t)&base->ggpio, GD32_GCCFG_PWRON);

	return 0;
}

/* The variable ret is provided by the enclosing pre-enable function. */
#define GD32_DWC2_CLK_ON(node_id, prop, idx)                                      \
	do {                                                                       \
		uint16_t clk_id = DT_CLOCKS_CELL_BY_IDX(node_id, idx, id);           \
		ret = clock_control_on(GD32_CLOCK_CONTROLLER,                         \
				       (clock_control_subsys_t)&clk_id);                \
		if (ret != 0) {                                                       \
			return ret;                                                    \
		}                                                                      \
	} while (0);

#define QUIRK_GD32_USBFS_DEFINE(n)                                                \
	static const struct reset_dt_spec gd32_dwc2_reset_##n =                    \
		RESET_DT_SPEC_INST_GET_OR(n, {0});                                  \
	                                                                               \
	static int gd32_usbfs_pre_enable_##n(const struct device *dev)              \
	{                                                                              \
		struct usb_dwc2_reg *const base = dwc2_get_base(dev);                 \
		int ret;                                                               \
	                                                                               \
		DT_INST_FOREACH_PROP_ELEM(n, clocks, GD32_DWC2_CLK_ON)                \
	                                                                               \
		ret = gd32_usb_request_ck48m();                                       \
		if (ret != 0) {                                                       \
			return ret;                                                    \
		}                                                                      \
	                                                                               \
		if (gd32_dwc2_reset_##n.dev != NULL &&                                \
		    device_is_ready(gd32_dwc2_reset_##n.dev)) {                       \
			ret = reset_line_toggle_dt(&gd32_dwc2_reset_##n);              \
			if (ret != 0 && ret != -ENOSYS) {                              \
				return ret;                                              \
			}                                                              \
		}                                                                      \
	                                                                               \
		gd32_usb_select_embedded_phy(dev, true);                               \
		sys_write32(0, (mem_addr_t)&base->pcgcctl);                            \
		return 0;                                                              \
	}                                                                              \
	                                                                               \
	static int gd32_usbfs_post_enable_##n(const struct device *dev)             \
	{                                                                              \
		gd32_usb_enable_embedded_phy(dev, DT_INST_PROP(n, vbus_sensing), true);\
		return 0;                                                              \
	}                                                                              \
	                                                                               \
	const struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {                  \
		.pre_enable = gd32_usbfs_pre_enable_##n,                                \
		.post_enable = gd32_usbfs_post_enable_##n,                              \
		.disable = gd32_usb_disable_embedded_phy,                               \
		.inepnakeff_level = true,                                               \
	};

#define GD32_USBHS_HAS_ULPI(n) DT_INST_CLOCKS_HAS_IDX(n, 1)

#define QUIRK_GD32_USBHS_DEFINE(n)                                                \
	static const struct reset_dt_spec gd32_dwc2_reset_##n =                    \
		RESET_DT_SPEC_INST_GET_OR(n, {0});                                  \
	                                                                               \
	static int gd32_usbhs_pre_enable_##n(const struct device *dev)              \
	{                                                                              \
		struct usb_dwc2_reg *const base = dwc2_get_base(dev);                 \
		int ret;                                                               \
	                                                                               \
		DT_INST_FOREACH_PROP_ELEM(n, clocks, GD32_DWC2_CLK_ON)                \
	                                                                               \
		IF_DISABLED(GD32_USBHS_HAS_ULPI(n), (                                  \
			ret = gd32_usb_request_ck48m();                                 \
			if (ret != 0) {                                                 \
				return ret;                                              \
			}                                                              \
		))                                                                     \
	                                                                               \
		if (gd32_dwc2_reset_##n.dev != NULL &&                                \
		    device_is_ready(gd32_dwc2_reset_##n.dev)) {                       \
			ret = reset_line_toggle_dt(&gd32_dwc2_reset_##n);              \
			if (ret != 0 && ret != -ENOSYS) {                              \
				return ret;                                              \
			}                                                              \
		}                                                                      \
	                                                                               \
		gd32_usb_select_embedded_phy(dev, !GD32_USBHS_HAS_ULPI(n));            \
		sys_write32(0, (mem_addr_t)&base->pcgcctl);                            \
		return 0;                                                              \
	}                                                                              \
	                                                                               \
	static int gd32_usbhs_post_enable_##n(const struct device *dev)             \
	{                                                                              \
		IF_DISABLED(GD32_USBHS_HAS_ULPI(n), (                                  \
			gd32_usb_enable_embedded_phy(                                    \
				dev, DT_INST_PROP(n, vbus_sensing), false);             \
		))                                                                     \
		return 0;                                                              \
	}                                                                              \
	                                                                               \
	static int gd32_usbhs_caps_##n(const struct device *dev)                    \
	{                                                                              \
		struct udc_data *data = dev->data;                                    \
	                                                                               \
		data->caps.hs = GD32_USBHS_HAS_ULPI(n);                               \
		return 0;                                                              \
	}                                                                              \
	                                                                               \
	const struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {                  \
		.pre_enable = gd32_usbhs_pre_enable_##n,                                \
		.post_enable = gd32_usbhs_post_enable_##n,                              \
		.disable = gd32_usb_disable_embedded_phy,                               \
		.caps = gd32_usbhs_caps_##n,                                            \
		.inepnakeff_level = true,                                               \
	};

#define QUIRK_GD32_DWC2_DEFINE(n)                                                \
	COND_CODE_1(DT_INST_NODE_HAS_COMPAT(n, gd_gd32_usbfs),                    \
		    (QUIRK_GD32_USBFS_DEFINE(n)),                                  \
		    (COND_CODE_1(DT_INST_NODE_HAS_COMPAT(n, gd_gd32_usbhs),         \
				 (QUIRK_GD32_USBHS_DEFINE(n)), ())))

DT_INST_FOREACH_STATUS_OKAY(QUIRK_GD32_DWC2_DEFINE)

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC2_GD32_H */
