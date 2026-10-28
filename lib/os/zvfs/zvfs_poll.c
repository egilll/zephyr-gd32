/*
 * Copyright (c) 2017-2018 Linaro Limited
 * Copyright (c) 2021 Nordic Semiconductor
 * Copyright (c) 2023 Arm Limited (or its affiliates). All rights reserved.
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/internal/syscall_handler.h>
#include <zephyr/sys/fdtable.h>

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
bool net_socket_is_tls(void *obj);
#else
#define net_socket_is_tls(obj) false
#endif

/*
 * How long a wait cut short by a descriptor that is already ready may last.
 * The first is immediate. Once short waits have come up empty, a descriptor
 * may be claiming to be ready without ever reporting it, so the waits that
 * follow last a little longer each time, up to 100 ms, rather than make poll()
 * spin: one that does become ready is still reported at the end of the wait,
 * or sooner if another descriptor wakes it.
 */
/* Enough empty short waits for the next one to last its longest */
#define SHORT_WAITS_MAX 8

static k_timeout_t short_wait(k_timepoint_t end, int empty_short_waits)
{
	k_timeout_t left = sys_timepoint_timeout(end);
	k_timeout_t pause;

	if (empty_short_waits == 0) {
		return K_NO_WAIT;
	}

	pause = K_MSEC(MIN(1 << (empty_short_waits - 1), 100));
	if (K_TIMEOUT_EQ(left, K_FOREVER) || left.ticks > pause.ticks) {
		return pause;
	}

	return left;
}

int zvfs_poll_internal(struct zvfs_pollfd *fds, int nfds, k_timeout_t timeout)
{
	bool retry;
	bool ready;
	bool cancelled;
	int empty_short_waits = 0;
	k_timeout_t wait;
	int ret = 0;
	int i;
	struct zvfs_pollfd *pfd;
	struct k_poll_event poll_events[CONFIG_ZVFS_POLL_MAX];
	struct k_poll_event *pev;
	struct k_poll_event *pev_end = poll_events + ARRAY_SIZE(poll_events);
	const struct fd_op_vtable *vtable;
	struct k_mutex *lock;
	k_timepoint_t end;
	bool offload = false;
	const struct fd_op_vtable *offl_vtable = NULL;
	void *offl_ctx = NULL;

	end = sys_timepoint_calc(timeout);

again:
	ready = false;
	pev = poll_events;
	for (pfd = fds, i = nfds; i--; pfd++) {
		void *ctx;
		int result;

		/* Per POSIX, negative fd's are just ignored */
		if (pfd->fd < 0) {
			continue;
		}

		ctx = zvfs_get_fd_obj_and_vtable(pfd->fd, &vtable, &lock);
		if (ctx == NULL) {
			/* Will set POLLNVAL in return loop */
			continue;
		}

		(void)k_mutex_lock(lock, K_FOREVER);

		result = zvfs_fdtable_call_ioctl(vtable, ctx, ZFD_IOCTL_POLL_PREPARE, pfd, &pev,
						 pev_end);
		if (result == -EALREADY) {
			/* If POLL_PREPARE returned with EALREADY, it means
			 * it already detected that some socket is ready. In
			 * this case, we still perform a k_poll to pick up
			 * as many events as possible, but without any wait.
			 */
			ready = true;
			result = 0;
		} else if (result == -EXDEV) {
			/* If POLL_PREPARE returned EXDEV, it means
			 * it detected an offloaded socket.
			 * If offloaded socket is used with native TLS, the TLS
			 * wrapper for the offloaded poll will be used.
			 * In case the fds array contains a mixup of offloaded
			 * and non-offloaded sockets, the offloaded poll handler
			 * shall return an error.
			 */
			offload = true;
			if (offl_vtable == NULL || net_socket_is_tls(ctx)) {
				offl_vtable = vtable;
				offl_ctx = ctx;
			}

			result = 0;
		}

		k_mutex_unlock(lock);

		if (result < 0) {
			errno = -result;
			return -1;
		}
	}

	if (offload) {
		int poll_timeout;

		/* A retry below may get here with part of the time used up */
		wait = sys_timepoint_timeout(end);

		if (ready) {
			poll_timeout = 0;
		} else if (K_TIMEOUT_EQ(wait, K_FOREVER)) {
			poll_timeout = SYS_FOREVER_MS;
		} else {
			poll_timeout = k_ticks_to_ms_floor32(wait.ticks);
		}

		return zvfs_fdtable_call_ioctl(offl_vtable, offl_ctx, ZFD_IOCTL_POLL_OFFLOAD, fds,
					       nfds, poll_timeout);
	}

	wait = ready ? short_wait(end, empty_short_waits) : sys_timepoint_timeout(end);
	cancelled = false;

	do {
		ret = k_poll(poll_events, pev - poll_events, wait);
		/* EAGAIN when timeout expired, EINTR when cancelled (i.e. EOF) */
		if (ret != 0 && ret != -EAGAIN && ret != -EINTR) {
			errno = -ret;
			return -1;
		}

		cancelled = cancelled || ret == -EINTR;

		retry = false;
		ret = 0;

		pev = poll_events;
		for (pfd = fds, i = nfds; i--; pfd++) {
			void *ctx;
			int result;

			pfd->revents = 0;

			if (pfd->fd < 0) {
				continue;
			}

			ctx = zvfs_get_fd_obj_and_vtable(pfd->fd, &vtable, &lock);
			if (ctx == NULL) {
				pfd->revents = ZVFS_POLLNVAL;
				ret++;
				continue;
			}

			(void)k_mutex_lock(lock, K_FOREVER);

			result = zvfs_fdtable_call_ioctl(vtable, ctx, ZFD_IOCTL_POLL_UPDATE, pfd,
							 &pev);
			k_mutex_unlock(lock);

			if (result == -EAGAIN) {
				retry = true;
				continue;
			} else if (result != 0) {
				errno = -result;
				return -1;
			}

			if (pfd->revents != 0) {
				ret++;
			}
		}

		if (retry) {
			if (ret > 0) {
				break;
			}

			/* A descriptor that is ready without an event to wait on
			 * is still only checked at the end of each wait.
			 */
			if (ready) {
				empty_short_waits = MIN(empty_short_waits + 1, SHORT_WAITS_MAX);
				wait = short_wait(end, empty_short_waits);
			} else {
				wait = sys_timepoint_timeout(end);
			}

			if (K_TIMEOUT_EQ(wait, K_NO_WAIT)) {
				break;
			}
		}
	} while (retry);

	/*
	 * A wait that was cancelled, or cut short by a descriptor that was
	 * already ready, can end with nothing to report: a polled socket was
	 * closed and its descriptor reused by one that is not ready, or what
	 * was ready is gone by now. poll() returns 0 only once the timeout has
	 * expired, so prepare and wait again until then.
	 */
	if (ret == 0 && (ready || cancelled) && !sys_timepoint_expired(end)) {
		if (ready) {
			empty_short_waits = MIN(empty_short_waits + 1, SHORT_WAITS_MAX);
		}
		goto again;
	}

	return ret;
}

int z_impl_zvfs_poll(struct zvfs_pollfd *fds, int nfds, int poll_timeout)
{
	k_timeout_t timeout;

	if (poll_timeout < 0) {
		timeout = K_FOREVER;
	} else {
		timeout = K_MSEC(poll_timeout);
	}

	return zvfs_poll_internal(fds, nfds, timeout);
}

#ifdef CONFIG_USERSPACE
static inline int z_vrfy_zvfs_poll(struct zvfs_pollfd *fds, int nfds, int timeout)
{
	struct zvfs_pollfd *fds_copy;
	size_t fds_size;
	int ret;

	/* Copy fds array from user mode */
	if (size_mul_overflow(nfds, sizeof(struct zvfs_pollfd), &fds_size)) {
		errno = EFAULT;
		return -1;
	}
	fds_copy = k_usermode_alloc_from_copy((void *)fds, fds_size);
	if (!fds_copy) {
		errno = ENOMEM;
		return -1;
	}

	ret = z_impl_zvfs_poll(fds_copy, nfds, timeout);

	if (ret >= 0) {
		k_usermode_to_copy((void *)fds, fds_copy, fds_size);
	}
	k_free(fds_copy);

	return ret;
}
#include <zephyr/syscalls/zvfs_poll_mrsh.c>
#endif
