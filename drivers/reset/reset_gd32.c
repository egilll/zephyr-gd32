/*
 * Copyright (c) 2022 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_rctl

#include <errno.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/sys/util.h>

#define GD32_RESET_BIT_MASK     0x1FU
#define GD32_RESET_OFFSET_SHIFT 6U
#define GD32_RESET_OFFSET_MASK  0xFFU
#define GD32_RESET_VALID_MASK                                                                      \
	((GD32_RESET_OFFSET_MASK << GD32_RESET_OFFSET_SHIFT) | GD32_RESET_BIT_MASK)

/** RCU offset (from id field) */
#define GD32_RESET_ID_OFFSET(id) (((id) >> GD32_RESET_OFFSET_SHIFT) & GD32_RESET_OFFSET_MASK)
/** RCU configuration bit (from id field) */
#define GD32_RESET_ID_BIT(id)    ((id) & GD32_RESET_BIT_MASK)

struct reset_gd32_config {
	uintptr_t base;
};

BUILD_ASSERT(DT_REG_SIZE(DT_INST_PARENT(0)) >= sizeof(uint32_t));

static int reset_gd32_validate_id(uint32_t id)
{
	if ((id & ~GD32_RESET_VALID_MASK) != 0U || (GD32_RESET_ID_OFFSET(id) & 0x3U) != 0U ||
	    GD32_RESET_ID_OFFSET(id) > DT_REG_SIZE(DT_INST_PARENT(0)) - sizeof(uint32_t)) {
		return -EINVAL;
	}

	return 0;
}

static int reset_gd32_status(const struct device *dev, uint32_t id,
			     uint8_t *status)
{
	const struct reset_gd32_config *config = dev->config;
	int err;

	if (status == NULL) {
		return -EINVAL;
	}

	err = reset_gd32_validate_id(id);
	if (err != 0) {
		return err;
	}

	*status = !!(sys_read32(config->base + GD32_RESET_ID_OFFSET(id)) &
		     BIT(GD32_RESET_ID_BIT(id)));

	return 0;
}

static int reset_gd32_line_assert(const struct device *dev, uint32_t id)
{
	const struct reset_gd32_config *config = dev->config;
	int err;

	err = reset_gd32_validate_id(id);
	if (err != 0) {
		return err;
	}

	sys_set_bit(config->base + GD32_RESET_ID_OFFSET(id),
		    GD32_RESET_ID_BIT(id));

	return 0;
}

static int reset_gd32_line_deassert(const struct device *dev, uint32_t id)
{
	const struct reset_gd32_config *config = dev->config;
	int err;

	err = reset_gd32_validate_id(id);
	if (err != 0) {
		return err;
	}

	sys_clear_bit(config->base + GD32_RESET_ID_OFFSET(id),
		      GD32_RESET_ID_BIT(id));

	return 0;
}

static int reset_gd32_line_toggle(const struct device *dev, uint32_t id)
{
	int err;

	err = reset_gd32_line_assert(dev, id);
	if (err != 0) {
		return err;
	}

	return reset_gd32_line_deassert(dev, id);
}

static DEVICE_API(reset, reset_gd32_driver_api) = {
	.status = reset_gd32_status,
	.line_assert = reset_gd32_line_assert,
	.line_deassert = reset_gd32_line_deassert,
	.line_toggle = reset_gd32_line_toggle,
};

static const struct reset_gd32_config config = {
	.base = DT_REG_ADDR(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &config, PRE_KERNEL_1,
		      CONFIG_RESET_INIT_PRIORITY, &reset_gd32_driver_api);
