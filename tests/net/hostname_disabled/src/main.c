/*
 * Copyright (c) 2026, Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/hostname.h>

BUILD_ASSERT(NET_HOSTNAME_MAX_LEN == sizeof("zephyr") - 1);
BUILD_ASSERT(NET_HOSTNAME_SIZE == 1);

int main(void)
{
	return 0;
}
