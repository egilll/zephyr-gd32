/*
 * Copyright (c) 2019 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_POSIX_POLL_H_
#define ZEPHYR_INCLUDE_POSIX_POLL_H_

#include <zephyr/sys/fdtable.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef	unsigned int nfds_t;

#define pollfd zvfs_pollfd

#define POLLIN   ZVFS_POLLIN
#define POLLPRI  ZVFS_POLLPRI
#define POLLOUT  ZVFS_POLLOUT
#define POLLERR  ZVFS_POLLERR
#define POLLHUP  ZVFS_POLLHUP
#define POLLNVAL ZVFS_POLLNVAL

int poll(struct pollfd *fds, int nfds, int timeout);

#ifdef __cplusplus
}
#endif

#endif	/* ZEPHYR_INCLUDE_POSIX_POLL_H_ */
