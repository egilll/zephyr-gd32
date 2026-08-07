/*
 * Copyright (c) 2026, Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_DWC_MAC_ETH_DWMAC_PLATFORM_H_
#define ZEPHYR_DRIVERS_ETHERNET_DWC_MAC_ETH_DWMAC_PLATFORM_H_

#include <errno.h>
#include <string.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/sys/crc.h>

static inline int dwmac_mac_addr_load(const struct net_eth_mac_config *cfg, uint8_t *mac_addr,
				      const uint8_t oui[3])
{
	uint8_t device_id[12];
	uint32_t hash;
	ssize_t length;
	int ret;

	ret = net_eth_mac_load(cfg, mac_addr);
	if (ret != -ENODATA) {
		return ret;
	}

	length = hwinfo_get_device_id(device_id, sizeof(device_id));
	if (length <= 0) {
		return length < 0 ? (int)length : -ENODATA;
	}

	hash = crc32_ieee(device_id, (size_t)length);
	if (oui != NULL) {
		mac_addr[0] = oui[0] | 0x02U;
		mac_addr[1] = oui[1];
		mac_addr[2] = oui[2];
		memcpy(&mac_addr[3], &hash, 3);
	} else {
		mac_addr[0] = 0x02U;
		mac_addr[1] = 0x00U;
		memcpy(&mac_addr[2], &hash, sizeof(hash));
	}

	return 0;
}

#endif /* ZEPHYR_DRIVERS_ETHERNET_DWC_MAC_ETH_DWMAC_PLATFORM_H_ */
