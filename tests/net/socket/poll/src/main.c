/*
 * Copyright (c) 2019 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_test, CONFIG_NET_SOCKETS_LOG_LEVEL);

#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/ztest_assert.h>

#include <zephyr/net/socket.h>
#include <zephyr/sys/fdtable.h>

#include "../../socket_helpers.h"

#define BUF_AND_SIZE(buf) buf, sizeof(buf) - 1
#define STRLEN(buf) (sizeof(buf) - 1)

#define TEST_STR_SMALL "test"

#define MY_IPV6_ADDR "::1"

#define ANY_PORT 0
#define SERVER_PORT 4242
#define CLIENT_PORT 9898

/* On QEMU, poll() which waits takes +10ms from the requested time. */
#define FUZZ 10

#define TCP_TEARDOWN_TIMEOUT K_SECONDS(3)

ZTEST(net_socket_poll, test_poll)
{
	int res;
	int c_sock;
	int s_sock;
	int c_sock_tcp;
	int s_sock_tcp;
	struct net_sockaddr_in6 c_addr;
	struct net_sockaddr_in6 s_addr;
	struct zsock_pollfd pollfds[2];
	struct zsock_pollfd pollout[1];
	uint32_t tstamp;
	ssize_t len;
	char buf[10];

	prepare_sock_udp_v6(MY_IPV6_ADDR, CLIENT_PORT, &c_sock, &c_addr);
	prepare_sock_udp_v6(MY_IPV6_ADDR, SERVER_PORT, &s_sock, &s_addr);
	prepare_sock_tcp_v6(MY_IPV6_ADDR, CLIENT_PORT, &c_sock_tcp, &c_addr);
	prepare_sock_tcp_v6(MY_IPV6_ADDR, SERVER_PORT, &s_sock_tcp, &s_addr);

	res = zsock_bind(s_sock, (struct net_sockaddr *)&s_addr, sizeof(s_addr));
	zassert_equal(res, 0, "bind failed");

	res = zsock_connect(c_sock, (struct net_sockaddr *)&s_addr, sizeof(s_addr));
	zassert_equal(res, 0, "connect failed");

	memset(pollfds, 0, sizeof(pollfds));
	pollfds[0].fd = c_sock;
	pollfds[0].events = ZSOCK_POLLIN;
	pollfds[1].fd = s_sock;
	pollfds[1].events = ZSOCK_POLLIN;

	/* Poll non-ready fd's with timeout of 0 */
	tstamp = k_uptime_get_32();
	res = zsock_poll(pollfds, ARRAY_SIZE(pollfds), 0);
	zassert_true(k_uptime_get_32() - tstamp <= FUZZ, "");
	zassert_equal(res, 0, "");

	zassert_equal(pollfds[0].fd, c_sock, "");
	zassert_equal(pollfds[0].events, ZSOCK_POLLIN, "");
	zassert_equal(pollfds[0].revents, 0, "");
	zassert_equal(pollfds[1].fd, s_sock, "");
	zassert_equal(pollfds[1].events, ZSOCK_POLLIN, "");
	zassert_equal(pollfds[1].revents, 0, "");


	/* Poll non-ready fd's with timeout of 30 */
	tstamp = k_uptime_get_32();
	res = zsock_poll(pollfds, ARRAY_SIZE(pollfds), 30);
	tstamp = k_uptime_get_32() - tstamp;
	zassert_true(tstamp >= 30U && tstamp <= 30 + FUZZ * 2, "tstamp %d",
		     tstamp);
	zassert_equal(res, 0, "");

	/* Send pkt for s_sock and poll with timeout of 10 */
	len = zsock_send(c_sock, BUF_AND_SIZE(TEST_STR_SMALL), 0);
	zassert_equal(len, STRLEN(TEST_STR_SMALL), "invalid send len");

	tstamp = k_uptime_get_32();
	res = zsock_poll(pollfds, ARRAY_SIZE(pollfds), 30);
	tstamp = k_uptime_get_32() - tstamp;
	zassert_true(tstamp <= FUZZ, "");
	zassert_equal(res, 1, "");

	zassert_equal(pollfds[0].fd, c_sock, "");
	zassert_equal(pollfds[0].events, ZSOCK_POLLIN, "");
	zassert_equal(pollfds[0].revents, 0, "");
	zassert_equal(pollfds[1].fd, s_sock, "");
	zassert_equal(pollfds[1].events, ZSOCK_POLLIN, "");
	zassert_equal(pollfds[1].revents, ZSOCK_POLLIN, "");


	/* Recv pkt from s_sock and ensure no poll events happen */
	len = zsock_recv(s_sock, BUF_AND_SIZE(buf), 0);
	zassert_equal(len, STRLEN(TEST_STR_SMALL), "invalid recv len");

	tstamp = k_uptime_get_32();
	res = zsock_poll(pollfds, ARRAY_SIZE(pollfds), 0);
	zassert_true(k_uptime_get_32() - tstamp <= FUZZ, "");
	zassert_equal(res, 0, "");
	zassert_equal(pollfds[1].revents, 0, "");

	/* Make sure that POLLOUT does not wait if not really needed */
	memset(pollout, 0, sizeof(pollout));
	pollout[0].fd = c_sock;
	pollout[0].events = ZSOCK_POLLOUT;

	res = zsock_connect(c_sock, (const struct net_sockaddr *)&s_addr,
			    sizeof(s_addr));
	zassert_equal(res, 0, "");

	tstamp = k_uptime_get_32();
	res = zsock_poll(pollout, ARRAY_SIZE(pollout), 200);
	zassert_true(k_uptime_get_32() - tstamp < 100, "");
	zassert_equal(res, 1, "");
	zassert_equal(pollout[0].revents, ZSOCK_POLLOUT, "");

	/* First test that TCP POLLOUT will not wait if there is enough
	 * room in TCP window
	 */
	memset(pollout, 0, sizeof(pollout));
	pollout[0].fd = c_sock_tcp;
	pollout[0].events = ZSOCK_POLLOUT;

	res = zsock_bind(s_sock_tcp, (struct net_sockaddr *)&s_addr, sizeof(s_addr));
	zassert_equal(res, 0, "");
	res = zsock_listen(s_sock_tcp, 0);
	zassert_equal(res, 0, "");

	res = zsock_connect(c_sock_tcp, (const struct net_sockaddr *)&s_addr,
			    sizeof(s_addr));
	zassert_equal(res, 0, "");

	tstamp = k_uptime_get_32();
	res = zsock_poll(pollout, ARRAY_SIZE(pollout), 200);
	zassert_true(k_uptime_get_32() - tstamp < 100, "");
	zassert_equal(res, 1, "");
	zassert_equal(pollout[0].revents, ZSOCK_POLLOUT, "");

	res = zsock_close(c_sock_tcp);
	zassert_equal(res, 0, "close failed");

	res = zsock_close(s_sock_tcp);
	zassert_equal(res, 0, "close failed");

	/* Close one socket and ensure POLLNVAL happens */
	res = zsock_close(c_sock);
	zassert_equal(res, 0, "close failed");

	tstamp = k_uptime_get_32();
	res = zsock_poll(pollfds, ARRAY_SIZE(pollfds), 0);
	zassert_true(k_uptime_get_32() - tstamp <= FUZZ, "");
	zassert_equal(res, 1, "");
	zassert_equal(pollfds[0].revents, ZSOCK_POLLNVAL, "");
	zassert_equal(pollfds[1].revents, 0, "");

	res = zsock_close(s_sock);
	zassert_equal(res, 0, "close failed");

	k_sleep(TCP_TEARDOWN_TIMEOUT);
}

#define TEST_SNDBUF_SIZE CONFIG_NET_TCP_MAX_RECV_WINDOW_SIZE

ZTEST(net_socket_poll, test_pollout_tcp)
{
	int res;
	int c_sock;
	int s_sock;
	int new_sock;
	struct net_sockaddr_in6 c_addr;
	struct net_sockaddr_in6 s_addr;
	struct zsock_pollfd pollout[1];
	char buf[TEST_SNDBUF_SIZE] = { };

	prepare_sock_tcp_v6(MY_IPV6_ADDR, CLIENT_PORT, &c_sock, &c_addr);
	prepare_sock_tcp_v6(MY_IPV6_ADDR, SERVER_PORT, &s_sock, &s_addr);

	res = zsock_bind(s_sock, (struct net_sockaddr *)&s_addr, sizeof(s_addr));
	zassert_equal(res, 0, "");
	res = zsock_listen(s_sock, 0);
	zassert_equal(res, 0, "");
	res = zsock_connect(c_sock, (const struct net_sockaddr *)&s_addr,
			    sizeof(s_addr));
	zassert_equal(res, 0, "");
	new_sock = zsock_accept(s_sock, NULL, NULL);
	zassert_true(new_sock >= 0, "");

	k_msleep(10);

	/* POLLOUT should be reported after connecting */
	memset(pollout, 0, sizeof(pollout));
	pollout[0].fd = c_sock;
	pollout[0].events = ZSOCK_POLLOUT;

	res = zsock_poll(pollout, ARRAY_SIZE(pollout), 10);
	zassert_equal(res, 1, "");
	zassert_equal(pollout[0].revents, ZSOCK_POLLOUT, "");

	/* POLLOUT should not be reported after filling the window */
	res = zsock_send(c_sock, buf, sizeof(buf), 0);
	zassert_equal(res, sizeof(buf), "");

	memset(pollout, 0, sizeof(pollout));
	pollout[0].fd = c_sock;
	pollout[0].events = ZSOCK_POLLOUT;

	res = zsock_poll(pollout, ARRAY_SIZE(pollout), 10);
	zassert_equal(res, 0, "%d", pollout[0].revents);
	zassert_equal(pollout[0].revents, 0, "");

	/* POLLOUT should be reported again after consuming the data server
	 * side.
	 */
	res = zsock_recv(new_sock, buf, sizeof(buf), 0);
	zassert_equal(res, sizeof(buf), "");

	memset(pollout, 0, sizeof(pollout));
	pollout[0].fd = c_sock;
	pollout[0].events = ZSOCK_POLLOUT;

	/* Wait longer this time to give TCP stack a chance to send ZWP. */
	res = zsock_poll(pollout, ARRAY_SIZE(pollout), 500);
	zassert_equal(res, 1, "");
	zassert_equal(pollout[0].revents, ZSOCK_POLLOUT, "");

	k_msleep(10);

	/* Finalize the test */
	res = zsock_close(c_sock);
	zassert_equal(res, 0, "close failed");
	res = zsock_close(s_sock);
	zassert_equal(res, 0, "close failed");
	res = zsock_close(new_sock);
	zassert_equal(res, 0, "close failed");
}

/*
 * A descriptor whose poll preparation and update are scripted, to put poll()
 * in the states a closing and reused socket leaves it in, without depending
 * on how the threads involved get scheduled. Readiness is a FIFO, as for a
 * socket: an item queued means readable, and cancelling the FIFO's wait is
 * what closing a socket does to a poll() waiting on it.
 */
struct scripted_fd {
	struct k_fifo fifo;
	/* Preparations still to answer -EALREADY, without an event */
	int already;
	/* Updates still to report nothing, whatever the FIFO holds */
	int silent;
	/* Updates still to answer -EAGAIN, as TLS does without a whole record */
	int again;
	int prepares;
	bool event_prepared;
};

static int scripted_ioctl(void *obj, unsigned int request, va_list args)
{
	struct scripted_fd *sfd = obj;
	struct zsock_pollfd *pfd;
	struct k_poll_event **pev;

	switch (request) {
	case ZFD_IOCTL_POLL_PREPARE: {
		struct k_poll_event *pev_end;

		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);
		pev_end = va_arg(args, struct k_poll_event *);

		sfd->prepares++;
		sfd->event_prepared = false;
		if (sfd->already > 0) {
			sfd->already--;
			return -EALREADY;
		}
		if (*pev == pev_end) {
			return -ENOMEM;
		}
		k_poll_event_init(*pev, K_POLL_TYPE_FIFO_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
				  &sfd->fifo);
		(*pev)++;
		sfd->event_prepared = true;
		return 0;
	}
	case ZFD_IOCTL_POLL_UPDATE:
		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);
		if (sfd->event_prepared) {
			(*pev)++;
		}
		if (sfd->again > 0) {
			sfd->again--;
			return -EAGAIN;
		}
		if (sfd->silent > 0) {
			sfd->silent--;
			return 0;
		}
		if ((pfd->events & ZSOCK_POLLIN) && !k_fifo_is_empty(&sfd->fifo)) {
			pfd->revents |= ZSOCK_POLLIN;
		}
		return 0;
	case ZFD_IOCTL_SET_LOCK:
		return 0;
	default:
		errno = EOPNOTSUPP;
		return -1;
	}
}

static const struct fd_op_vtable scripted_vtable = {
	.ioctl = scripted_ioctl,
};

static struct scripted_fd scripted[2];
static int scripted_fds[2] = {-1, -1};
static struct {
	void *fifo_reserved;
} scripted_item[2];

static int scripted_open(struct scripted_fd *sfd)
{
	int fd;

	memset(sfd, 0, sizeof(*sfd));
	k_fifo_init(&sfd->fifo);
	fd = zvfs_reserve_fd();
	zassert_true(fd >= 0, "no descriptor (%d)", errno);
	zvfs_finalize_fd(fd, sfd, &scripted_vtable);
	return fd;
}

static K_THREAD_STACK_DEFINE(poller_stack, 1024 + CONFIG_TEST_EXTRA_STACK_SIZE);
static struct k_thread poller_thread;
static bool poller_running;
static struct zsock_pollfd poller_fds[2];
static int poller_nfds;
static int poller_timeout;
static int poller_res;
static int64_t poller_elapsed;
static K_SEM_DEFINE(poller_done, 0, 1);

static void poller(void *p1, void *p2, void *p3)
{
	int64_t start = k_uptime_get();

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	poller_res = zsock_poll(poller_fds, poller_nfds, poller_timeout);
	poller_elapsed = k_uptime_get() - start;
	k_sem_give(&poller_done);
}

static bool poller_waiting_or_done(void)
{
	char state[32];

	if (k_sem_count_get(&poller_done) > 0) {
		return true;
	}

	return scripted[0].prepares > 0 &&
	       strstr(k_thread_state_str(&poller_thread, state, sizeof(state)), "pending") != NULL;
}

/* Polls the scripted descriptors for reading in a thread of its own */
static void poller_start(int nfds, int timeout)
{
	for (int i = 0; i < nfds; i++) {
		poller_fds[i].fd = scripted_fds[i];
		poller_fds[i].events = ZSOCK_POLLIN;
		poller_fds[i].revents = 0;
	}
	poller_nfds = nfds;
	poller_timeout = timeout;
	k_sem_reset(&poller_done);
	/* Below the test thread, so that a poll() that spins cannot keep the
	 * test from checking on it
	 */
	k_thread_create(&poller_thread, poller_stack, K_THREAD_STACK_SIZEOF(poller_stack),
			poller, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	poller_running = true;

	for (int i = 0; i < 100 && !poller_waiting_or_done(); i++) {
		k_msleep(1);
	}
	zassert_true(poller_waiting_or_done(), "poll() did not get to wait");
}

static void poller_finish(void)
{
	zassert_ok(k_sem_take(&poller_done, K_SECONDS(1)), "poll() did not return");
	k_thread_join(&poller_thread, K_FOREVER);
	poller_running = false;
}

static void scripted_after(void *fixture)
{
	ARG_UNUSED(fixture);

	if (poller_running) {
		(void)k_thread_abort(&poller_thread);
		poller_running = false;
	}
	ARRAY_FOR_EACH(scripted_fds, i) {
		if (scripted_fds[i] >= 0) {
			zvfs_free_fd(scripted_fds[i]);
			scripted_fds[i] = -1;
		}
	}
}

/* Make a scripted descriptor readable */
static void scripted_ready(int i)
{
	k_fifo_put(&scripted[i].fifo, &scripted_item[i]);
}

/*
 * A descriptor that was ready when poll() prepared cuts the wait short. If it
 * has nothing to report by the update, a poll() without a timeout must not
 * return 0: it waits on, and reports the descriptor once it is readable.
 */
ZTEST(net_socket_poll, test_poll_waits_on_after_short_wait_reports_nothing)
{
	scripted_fds[0] = scripted_open(&scripted[0]);
	scripted[0].already = 1;

	poller_start(1, -1);
	zassert_equal(k_sem_take(&poller_done, K_MSEC(50)), -EAGAIN,
		      "poll() returned %d with nothing to report", poller_res);

	scripted_ready(0);
	poller_finish();
	zassert_equal(poller_res, 1, "poll() returned %d", poller_res);
	zassert_equal(poller_fds[0].revents, ZSOCK_POLLIN, "revents 0x%x", poller_fds[0].revents);
}

/*
 * Closing a socket cancels a poll() waiting on it. When the descriptor then
 * has nothing to report, as one reused by an idle socket does, poll() without
 * a timeout must wait on rather than return 0.
 */
ZTEST(net_socket_poll, test_poll_waits_on_after_cancelled_wait_reports_nothing)
{
	scripted_fds[0] = scripted_open(&scripted[0]);

	poller_start(1, -1);
	k_fifo_cancel_wait(&scripted[0].fifo);
	zassert_equal(k_sem_take(&poller_done, K_MSEC(50)), -EAGAIN,
		      "poll() returned %d with nothing to report", poller_res);
	zassert_true(scripted[0].prepares >= 2, "poll() did not prepare again");

	scripted_ready(0);
	poller_finish();
	zassert_equal(poller_res, 1, "poll() returned %d", poller_res);
	zassert_equal(poller_fds[0].revents, ZSOCK_POLLIN, "revents 0x%x", poller_fds[0].revents);
}

/* A short wait that reports nothing does not use up a finite timeout */
ZTEST(net_socket_poll, test_poll_keeps_its_timeout_after_short_wait_reports_nothing)
{
	scripted_fds[0] = scripted_open(&scripted[0]);
	scripted[0].already = 1;

	poller_start(1, 100);
	poller_finish();
	zassert_equal(poller_res, 0, "poll() returned %d", poller_res);
	zassert_true(poller_elapsed >= 100, "poll() returned after %lld ms", poller_elapsed);
}

/*
 * Readiness that is gone by the update must not hide readiness that is real:
 * a descriptor that is ready when prepared again is reported promptly.
 */
ZTEST(net_socket_poll, test_poll_reports_readiness_after_short_wait_reports_nothing)
{
	scripted_fds[0] = scripted_open(&scripted[0]);
	scripted[0].already = 2;
	scripted[0].silent = 1;
	/* Readable from the start, but the first update does not see it */
	scripted_ready(0);

	poller_start(1, -1);
	poller_finish();
	zassert_equal(poller_res, 1, "poll() returned %d", poller_res);
	zassert_equal(poller_fds[0].revents, ZSOCK_POLLIN, "revents 0x%x", poller_fds[0].revents);
	zassert_true(poller_elapsed < 500, "poll() took %lld ms", poller_elapsed);
}

/*
 * The same, while another descriptor's update keeps asking to be retried:
 * the retries must not wait without bound for that descriptor's event and
 * leave the one that is ready unreported.
 */
ZTEST(net_socket_poll, test_poll_reports_readiness_while_another_descriptor_retries)
{
	scripted_fds[0] = scripted_open(&scripted[0]);
	scripted_fds[1] = scripted_open(&scripted[1]);
	scripted[0].already = INT_MAX;
	scripted[0].silent = 1;
	scripted_ready(0);
	scripted[1].again = INT_MAX;

	poller_start(2, -1);
	poller_finish();
	zassert_equal(poller_res, 1, "poll() returned %d", poller_res);
	zassert_equal(poller_fds[0].revents, ZSOCK_POLLIN, "revents 0x%x", poller_fds[0].revents);
}

/*
 * A descriptor that always claims to be ready when prepared but never reports
 * anything must not make poll() spin: it waits on the other descriptors.
 */
ZTEST(net_socket_poll, test_poll_does_not_spin_on_a_descriptor_ready_without_events)
{
	int64_t start;
	int64_t elapsed;
	int prepares;

	scripted_fds[0] = scripted_open(&scripted[0]);
	scripted_fds[1] = scripted_open(&scripted[1]);
	scripted[0].already = INT_MAX;
	scripted[0].silent = INT_MAX;

	start = k_uptime_get();
	poller_start(2, -1);
	k_msleep(50);
	prepares = scripted[0].prepares;
	elapsed = k_uptime_get() - start;
	/* The waits double up to 100 ms: a handful, then ten a second */
	zassert_true(prepares <= 8 + elapsed / 100, "poll() spun: %d preparations in %lld ms",
		     prepares, elapsed);

	scripted_ready(1);
	poller_finish();
	zassert_equal(poller_res, 1, "poll() returned %d", poller_res);
	zassert_equal(poller_fds[1].revents, ZSOCK_POLLIN, "revents 0x%x", poller_fds[1].revents);
}

ZTEST_SUITE(net_socket_poll, NULL, NULL, NULL, scripted_after, NULL);
