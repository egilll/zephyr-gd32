/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <gd32_regs.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <string.h>

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint32_t dev_id[3];

	dev_id[0] = sys_cpu_to_be32(sys_read32(GD32_UID_WORD0));
	dev_id[1] = sys_cpu_to_be32(sys_read32(GD32_UID_WORD1));
	dev_id[2] = sys_cpu_to_be32(sys_read32(GD32_UID_WORD2));

	length = MIN(length, sizeof(dev_id));

	memcpy(buffer, dev_id, length);

	return length;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	uint32_t flags = 0;
	uint32_t rstsck = sys_read32(RCU_BASE + RCU_RSTSCK_OFFSET);

#ifdef RCU_RSTSCK_EPRSTF
	if (rstsck & RCU_RSTSCK_EPRSTF) {
		flags |= RESET_PIN;
	}
#endif

#ifdef RCU_RSTSCK_SWRSTF
	if (rstsck & RCU_RSTSCK_SWRSTF) {
		flags |= RESET_SOFTWARE;
	}
#endif

#ifdef RCU_RSTSCK_FWDGTRSTF
	if (rstsck & RCU_RSTSCK_FWDGTRSTF) {
		flags |= RESET_WATCHDOG;
	}
#endif

#ifdef RCU_RSTSCK_WWDGTRSTF
	if (rstsck & RCU_RSTSCK_WWDGTRSTF) {
		flags |= RESET_WATCHDOG;
	}
#endif

#ifdef RCU_RSTSCK_PORRSTF
	if (rstsck & RCU_RSTSCK_PORRSTF) {
		flags |= RESET_POR;
	}
#endif

#ifdef RCU_RSTSCK_BORRSTF
	if (rstsck & RCU_RSTSCK_BORRSTF) {
		flags |= RESET_BROWNOUT;
	}
#endif

#ifdef RCU_RSTSCK_LVDRSTF
	if (rstsck & RCU_RSTSCK_LVDRSTF) {
		flags |= RESET_BROWNOUT;
	}
#endif

#ifdef RCU_RSTSCK_V12RSTF
	if (rstsck & RCU_RSTSCK_V12RSTF) {
		flags |= RESET_BROWNOUT;
	}
#endif

#ifdef RCU_RSTSCK_V11RSTF
	if (rstsck & RCU_RSTSCK_V11RSTF) {
		flags |= RESET_BROWNOUT;
	}
#endif

#ifdef RCU_RSTSCK_LPRSTF
	if (rstsck & RCU_RSTSCK_LPRSTF) {
		flags |= RESET_LOW_POWER_WAKE;
	}
#endif

#ifdef RCU_RSTSCK_LOCKUPRSTF
	if (rstsck & RCU_RSTSCK_LOCKUPRSTF) {
		flags |= RESET_CPU_LOCKUP;
	}
#endif

#ifdef RCU_RSTSCK_ECCRSTF
	if (rstsck & RCU_RSTSCK_ECCRSTF) {
		flags |= RESET_PARITY;
	}
#endif

#ifdef RCU_RSTSCK_OBLRSTF
	if (rstsck & RCU_RSTSCK_OBLRSTF) {
		flags |= RESET_BOOTLOADER;
	}
#endif

#ifdef RCU_RSTSCK_LOPRSTF
	if (rstsck & RCU_RSTSCK_LOPRSTF) {
		flags |= RESET_PLL;
	}
#endif

#ifdef RCU_RSTSCK_LOHRSTF
	if (rstsck & RCU_RSTSCK_LOHRSTF) {
		flags |= RESET_CLOCK;
	}
#endif

	*cause = flags;

	return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
	uint32_t rstsck_addr = RCU_BASE + RCU_RSTSCK_OFFSET;

	sys_write32(sys_read32(rstsck_addr) | RCU_RSTSCK_RSTFC, rstsck_addr);
	return 0;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	uint32_t flags = 0;

#ifdef RCU_RSTSCK_EPRSTF
	flags |= RESET_PIN;
#endif
#ifdef RCU_RSTSCK_SWRSTF
	flags |= RESET_SOFTWARE;
#endif
#ifdef RCU_RSTSCK_FWDGTRSTF
	flags |= RESET_WATCHDOG;
#endif
#ifdef RCU_RSTSCK_WWDGTRSTF
	flags |= RESET_WATCHDOG;
#endif
#ifdef RCU_RSTSCK_PORRSTF
	flags |= RESET_POR;
#endif
#if defined(RCU_RSTSCK_BORRSTF) || defined(RCU_RSTSCK_LVDRSTF) || defined(RCU_RSTSCK_V12RSTF) || \
	defined(RCU_RSTSCK_V11RSTF)
	flags |= RESET_BROWNOUT;
#endif
#ifdef RCU_RSTSCK_LPRSTF
	flags |= RESET_LOW_POWER_WAKE;
#endif
#ifdef RCU_RSTSCK_LOCKUPRSTF
	flags |= RESET_CPU_LOCKUP;
#endif
#ifdef RCU_RSTSCK_ECCRSTF
	flags |= RESET_PARITY;
#endif
#ifdef RCU_RSTSCK_OBLRSTF
	flags |= RESET_BOOTLOADER;
#endif
#ifdef RCU_RSTSCK_LOPRSTF
	flags |= RESET_PLL;
#endif
#ifdef RCU_RSTSCK_LOHRSTF
	flags |= RESET_CLOCK;
#endif

	*supported = flags;
	return 0;
}
