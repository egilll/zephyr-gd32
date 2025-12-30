/*
 * Copyright (c) 2025 Meshium
 *               Aleksandr Senin <al@meshium.net>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/hwinfo.h>
#include <string.h>

#include <soc.h>
#include <gd32f4xx.h>
#include <zephyr/sys/byteorder.h>

/* GD32F4xx Device electronic signature (Unique ID) base address.
 * According to GD32F4xx reference manual, the 96-bit UID is located at:
 * 0x1FFF7A10 (word0), 0x1FFF7A14 (word1), 0x1FFF7A18 (word2).
 */
#ifndef GD32_UID_BASE
#define GD32_UID_BASE 0x1FFF7A10U
#endif

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	uint32_t uid_words[3];

	const volatile uint32_t *uid = (const volatile uint32_t *)GD32_UID_BASE;
	size_t out_len = sizeof(uid_words);

	uid_words[0] = sys_cpu_to_be32(uid[0]);
	uid_words[1] = sys_cpu_to_be32(uid[1]);
	uid_words[2] = sys_cpu_to_be32(uid[2]);

	if (length < out_len) {
		out_len = length;
	}

	memcpy(buffer, uid_words, out_len);
	return out_len;
}

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	uint32_t sup = 0U;

#if defined(RCU_RSTSCK_PINRSTF) || defined(RCU_RSTSCK_EPRSTF)
	sup |= RESET_PIN;
#endif
#if defined(RCU_RSTSCK_SWRSTF)
	sup |= RESET_SOFTWARE;
#endif
#if defined(RCU_RSTSCK_BORRSTF)
	sup |= RESET_BROWNOUT;
#endif
#if defined(RCU_RSTSCK_PORRSTF)
	sup |= RESET_POR;
#endif
#if defined(RCU_RSTSCK_WWDGRSTF) || defined(RCU_RSTSCK_IWDGRSTF) || defined(RCU_RSTSCK_FWDGTRSTF)
	sup |= RESET_WATCHDOG;
#endif

	*supported = sup;

	return 0;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	uint32_t flags = 0U;
	uint32_t rsts = RCU_RSTSCK;

#if defined(RCU_RSTSCK_PINRSTF)
	if (rsts & RCU_RSTSCK_PINRSTF) {
		flags |= RESET_PIN;
	}
#endif
#if defined(RCU_RSTSCK_EPRSTF)
	if (rsts & RCU_RSTSCK_EPRSTF) {
		flags |= RESET_PIN;
	}
#endif
#if defined(RCU_RSTSCK_WWDGRSTF)
	if (rsts & RCU_RSTSCK_WWDGRSTF) {
		flags |= RESET_WATCHDOG;
	}
#endif
#if defined(RCU_RSTSCK_IWDGRSTF)
	if (rsts & RCU_RSTSCK_IWDGRSTF) {
		flags |= RESET_WATCHDOG;
	}
#endif
#if defined(RCU_RSTSCK_FWDGTRSTF)
	if (rsts & RCU_RSTSCK_FWDGTRSTF) {
		flags |= RESET_WATCHDOG;
	}
#endif
#if defined(RCU_RSTSCK_SWRSTF)
	if (rsts & RCU_RSTSCK_SWRSTF) {
		flags |= RESET_SOFTWARE;
	}
#endif
#if defined(RCU_RSTSCK_BORRSTF)
	if (rsts & RCU_RSTSCK_BORRSTF) {
		flags |= RESET_BROWNOUT;
	}
#endif
#if defined(RCU_RSTSCK_PORRSTF)
	if (rsts & RCU_RSTSCK_PORRSTF) {
		flags |= RESET_POR;
	}
#endif

	*cause = flags;
	return 0;
}

int z_impl_hwinfo_clear_reset_cause(void)
{
	/* Writing 1 to RSTFC clears all reset flags */
#if defined(RCU_RSTSCK_RSTFC)
	RCU_RSTSCK |= RCU_RSTSCK_RSTFC;
	return 0;
#else
	return -ENOSYS;
#endif
}
