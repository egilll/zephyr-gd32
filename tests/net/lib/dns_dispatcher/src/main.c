/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_test, CONFIG_DNS_RESOLVER_LOG_LEVEL);

#include <zephyr/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/random/random.h>

#include <zephyr/ztest.h>

#include <zephyr/net/ethernet.h>
#include <zephyr/net/dummy.h>
#include <zephyr/net_buf.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socket_service.h>

#define NET_LOG_ENABLED 1
#include "net_private.h"

#if defined(CONFIG_DNS_RESOLVER_LOG_LEVEL_DBG)
#define DBG(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define DBG(fmt, ...)
#endif

extern void dns_dispatcher_svc_handler(struct net_socket_service_event *pev);

NET_SOCKET_SERVICE_SYNC_DEFINE_STATIC(test_pair_svc, dns_dispatcher_svc_handler, 1);

static int test_dispatch_cb(struct dns_socket_dispatcher *ctx, int sock,
			    struct net_sockaddr *addr, size_t addrlen,
			    struct net_buf *buf, size_t data_len)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(sock);
	ARG_UNUSED(addr);
	ARG_UNUSED(addrlen);
	ARG_UNUSED(buf);
	ARG_UNUSED(data_len);

	return 0;
}

#define NAME4 "4.zephyr.test"
#define NAME6 "6.zephyr.test"
#define NAME_IPV4 "192.0.2.1"
#define NAME_IPV6 "2001:db8::1"

#define DNS_NAME_IPV4 "192.0.2.4"
#define DNS2_NAME_IPV4 "192.0.2.5"
#define DNS_NAME_IPV6 "2001:db8::4"

#define DNS_TIMEOUT 500 /* ms */

#if defined(CONFIG_NET_IPV6)
/* Interface 1 addresses */
static struct net_in6_addr my_addr1 = { { { 0x20, 0x01, 0x0d, 0xb8, 1, 0, 0, 0,
					    0, 0, 0, 0, 0, 0, 0, 0x1 } } };
#endif

#if defined(CONFIG_NET_IPV4)
/* Interface 1 addresses */
static struct net_in_addr my_addr2 = { { { 192, 0, 2, 1 } } };
#endif

static struct net_if *iface1;

/* this must be higher that the DNS_TIMEOUT */
#define WAIT_TIME K_MSEC((DNS_TIMEOUT + 300) * 3)

struct net_if_test {
	uint8_t idx;
	uint8_t mac_addr[sizeof(struct net_eth_addr)];
};

static uint8_t *net_iface_get_mac(const struct device *dev)
{
	struct net_if_test *data = dev->data;

	if (data->mac_addr[2] == 0x00) {
		/* 00-00-5E-00-53-xx Documentation RFC 7042 */
		data->mac_addr[0] = 0x00;
		data->mac_addr[1] = 0x00;
		data->mac_addr[2] = 0x5E;
		data->mac_addr[3] = 0x00;
		data->mac_addr[4] = 0x53;
		data->mac_addr[5] = sys_rand8_get();
	}

	return data->mac_addr;
}

static void net_iface_init(struct net_if *iface)
{
	uint8_t *mac = net_iface_get_mac(net_if_get_device(iface));

	net_if_set_link_addr(iface, mac, sizeof(struct net_eth_addr),
			     NET_LINK_ETHERNET);
}

static int sender_iface(const struct device *dev, struct net_pkt *pkt)
{
	if (!pkt->frags) {
		DBG("No data to send!\n");
		return -ENODATA;
	}

	return 0;
}

struct net_if_test net_iface1_data;

static struct dummy_api net_iface_api = {
	.iface_api.init = net_iface_init,
	.send = sender_iface,
};

#define _ETH_L2_LAYER DUMMY_L2
#define _ETH_L2_CTX_TYPE NET_L2_GET_CTX_TYPE(DUMMY_L2)

NET_DEVICE_INIT_INSTANCE(net_iface1_test,
			 "iface1",
			 iface1,
			 NULL,
			 NULL,
			 &net_iface1_data,
			 NULL,
			 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
			 &net_iface_api,
			 _ETH_L2_LAYER,
			 _ETH_L2_CTX_TYPE,
			 127);

static void *test_init(void)
{
	struct net_if_addr *ifaddr;

	iface1 = net_if_get_by_index(0);
	zassert_is_null(iface1, "iface1");

	iface1 = net_if_get_by_index(1);

	((struct net_if_test *) net_if_get_device(iface1)->data)->idx =
		net_if_get_by_iface(iface1);

#if defined(CONFIG_NET_IPV6)
	ifaddr = net_if_ipv6_addr_add(iface1, &my_addr1,
				      NET_ADDR_MANUAL, 0);
	if (!ifaddr) {
		DBG("Cannot add IPv6 address %s\n",
		       net_sprint_ipv6_addr(&my_addr1));
		zassert_not_null(ifaddr, "addr1");

		return NULL;
	}

	/* For testing purposes we need to set the addresses preferred */
	ifaddr->addr_state = NET_ADDR_PREFERRED;
#endif

#if defined(CONFIG_NET_IPV4)
	ifaddr = net_if_ipv4_addr_add(iface1, &my_addr2,
				      NET_ADDR_MANUAL, 0);
	if (!ifaddr) {
		DBG("Cannot add IPv4 address %s\n",
		       net_sprint_ipv4_addr(&my_addr2));
		zassert_not_null(ifaddr, "addr2");

		return NULL;
	}

	ifaddr->addr_state = NET_ADDR_PREFERRED;
#endif

	net_if_up(iface1);

	return NULL;
}

ZTEST(dns_dispatcher, test_dns_dispatcher)
{
	struct dns_resolve_context *ctx;
	int ret, sock1, sock2 = -1;

	ctx = dns_resolve_get_default();

	dns_resolve_close(ctx);

	ret = dns_resolve_init_default(ctx);
	zassert_equal(ret, 0, "Cannot initialize DNS resolver (%d)", ret);

	sock1 = ctx->servers[0].sock;

	for (int i = 0; i < ctx->servers[0].dispatcher.fds_len; i++) {
		if (ctx->servers[0].dispatcher.fds[i].fd == sock1) {
			sock2 = i;
			break;
		}
	}

	zassert_not_equal(sock2, -1, "Cannot find socket");

	k_sleep(K_MSEC(10));

	dns_resolve_close(ctx);

	zassert_equal(ctx->servers[0].dispatcher.fds[sock2].fd, -1, "Socket not closed");
	zassert_equal(ctx->servers[0].dispatcher.sock, -1, "Dispatcher still registered");
}

/* Register a responder and a resolver on the same family/port so the
 * dispatcher pairs them (responder->pair points at the resolver). Closing the
 * resolver must clear that back-reference, otherwise the surviving responder
 * would later delegate traffic to an unregistered context.
 */
ZTEST(dns_dispatcher, test_dispatcher_pair_cleanup)
{
	static struct dns_socket_dispatcher responder;
	static struct dns_socket_dispatcher resolver;
	static struct zsock_pollfd responder_fds[1];
	static struct zsock_pollfd resolver_fds[1];
	struct net_sockaddr_in local = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(65123),
	};
	int responder_sock, resolver_sock;

	responder_sock = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, NET_IPPROTO_UDP);
	zassert_true(responder_sock >= 0, "Cannot create responder socket");

	resolver_sock = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, NET_IPPROTO_UDP);
	zassert_true(resolver_sock >= 0, "Cannot create resolver socket");

	responder_fds[0].fd = responder_sock;
	resolver_fds[0].fd = resolver_sock;

	responder.type = DNS_SOCKET_RESPONDER;
	responder.cb = test_dispatch_cb;
	responder.fds = responder_fds;
	responder.fds_len = 1;
	responder.sock = responder_sock;
	responder.svc = &test_pair_svc;
	memcpy(&responder.local_addr_storage, &local, sizeof(local));

	resolver.type = DNS_SOCKET_RESOLVER;
	resolver.cb = test_dispatch_cb;
	resolver.fds = resolver_fds;
	resolver.fds_len = 1;
	resolver.sock = resolver_sock;
	resolver.svc = &test_pair_svc;
	memcpy(&resolver.local_addr_storage, &local, sizeof(local));

	zassert_ok(dns_dispatcher_register(&responder), "Cannot register responder");
	zassert_ok(dns_dispatcher_register(&resolver), "Cannot register resolver");

	zassert_equal(responder.pair, &resolver, "Dispatchers were not paired");

	zassert_ok(dns_dispatcher_unregister(&resolver), "Cannot unregister resolver");
	zassert_is_null(responder.pair, "Pair back-reference was not cleared");

	zassert_ok(dns_dispatcher_unregister(&responder), "Cannot unregister responder");

	(void)zsock_close(responder_sock);
	(void)zsock_close(resolver_sock);
}

/* A poll event may still be in flight for a socket whose dispatch slot was
 * cleared concurrently (e.g. the server was just closed). Emulate that by
 * handing the handler a dispatch table whose slot is NULL and verify the event
 * is dropped instead of dereferencing a NULL dispatcher.
 */
ZTEST(dns_dispatcher, test_dispatcher_null_slot_dropped)
{
	/* dispatch_table entries start with the dispatcher pointer, so a plain
	 * NULL pointer slot is layout-compatible for index 0.
	 */
	struct dns_socket_dispatcher *table[1] = { NULL };
	struct net_socket_service_event pev = {
		.event = {
			.fd = 0,
			.revents = ZSOCK_POLLIN,
		},
		.user_data = table,
	};

	/* Prior to the NULL guard this dereferenced a NULL dispatcher and
	 * crashed; reaching the next statement proves the event was dropped.
	 */
	dns_dispatcher_svc_handler(&pev);
}

static atomic_t stale_hits;
static atomic_t fresh_hits;

static int stale_cb(struct dns_socket_dispatcher *ctx, int sock, struct net_sockaddr *addr,
		    size_t addrlen, struct net_buf *buf, size_t data_len)
{
	atomic_inc(&stale_hits);
	return test_dispatch_cb(ctx, sock, addr, addrlen, buf, data_len);
}

static int fresh_cb(struct dns_socket_dispatcher *ctx, int sock, struct net_sockaddr *addr,
		    size_t addrlen, struct net_buf *buf, size_t data_len)
{
	atomic_inc(&fresh_hits);
	return test_dispatch_cb(ctx, sock, addr, addrlen, buf, data_len);
}

/* A dispatcher whose registration fails must leave no dispatch slot behind:
 * once its socket is closed and the descriptor reused, data on that descriptor
 * belongs to the dispatcher registered with it.
 */
ZTEST(dns_dispatcher, test_failed_register_leaves_no_slot)
{
	static struct dns_socket_dispatcher failed;
	static struct dns_socket_dispatcher fresh;
	/* More descriptors than test_pair_svc holds, so that registering fails */
	static struct zsock_pollfd failed_fds[2];
	static struct zsock_pollfd fresh_fds[1];
	/* Identifier, then flags with QR set: an answer, which goes to a resolver */
	static const uint8_t answer[] = {0x12, 0x34, 0x80, 0x00};
	struct net_sockaddr_in failed_local = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(65124),
	};
	struct net_sockaddr_in fresh_local = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(65125),
	};
	struct net_sockaddr_in to = fresh_local;
	int sock, fresh_sock, client, ret;

	atomic_clear(&stale_hits);
	atomic_clear(&fresh_hits);

	sock = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, NET_IPPROTO_UDP);
	zassert_true(sock >= 0, "Cannot create socket");
	failed_fds[0].fd = sock;
	failed_fds[0].events = ZSOCK_POLLIN;
	failed_fds[1].fd = -1;
	failed.type = DNS_SOCKET_RESOLVER;
	failed.cb = stale_cb;
	failed.fds = failed_fds;
	failed.fds_len = ARRAY_SIZE(failed_fds);
	failed.sock = sock;
	failed.svc = &test_pair_svc;
	memcpy(&failed.local_addr_storage, &failed_local, sizeof(failed_local));

	ret = dns_dispatcher_register(&failed);
	zassert_equal(ret, -ENOMEM, "Registration did not fail (%d)", ret);
	(void)zsock_close(sock);

	fresh_sock = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, NET_IPPROTO_UDP);
	zassert_equal(fresh_sock, sock, "Descriptor %d not reused (%d)", sock, fresh_sock);
	fresh_fds[0].fd = fresh_sock;
	fresh_fds[0].events = ZSOCK_POLLIN;
	fresh.type = DNS_SOCKET_RESOLVER;
	fresh.cb = fresh_cb;
	fresh.fds = fresh_fds;
	fresh.fds_len = ARRAY_SIZE(fresh_fds);
	fresh.sock = fresh_sock;
	fresh.svc = &test_pair_svc;
	memcpy(&fresh.local_addr_storage, &fresh_local, sizeof(fresh_local));
	zassert_ok(dns_dispatcher_register(&fresh), "Cannot register dispatcher");

	client = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, NET_IPPROTO_UDP);
	zassert_true(client >= 0, "Cannot create client socket");
	to.sin_addr = my_addr2;
	ret = zsock_sendto(client, answer, sizeof(answer), 0, (struct net_sockaddr *)&to,
			   sizeof(to));

	for (int i = 0; i < 50 && atomic_get(&fresh_hits) == 0 && atomic_get(&stale_hits) == 0;
	     i++) {
		k_msleep(10);
	}

	(void)dns_dispatcher_unregister(&fresh);
	(void)zsock_close(fresh_sock);
	(void)zsock_close(client);

	zassert_equal(ret, sizeof(answer), "Cannot send (%d)", errno);
	zassert_equal(atomic_get(&stale_hits), 0, "Data went to the failed dispatcher");
	zassert_true(atomic_get(&fresh_hits) > 0, "Data did not reach the dispatcher");
}

/* A datagram on a socket whose dispatch slot is empty (the socket was just
 * unregistered, or its descriptor left polled) must still be read. Left
 * queued, it keeps the socket readable, so the socket service calls the
 * handler again at once, in a loop that starves every thread of lower
 * priority.
 */
ZTEST(dns_dispatcher, test_null_slot_datagram_is_consumed)
{
	static const uint8_t answer[] = {0x12, 0x34, 0x80, 0x00};
	struct net_sockaddr_in local = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(65127),
	};
	struct net_sockaddr_in to = local;
	struct zsock_pollfd pollfd = {.events = ZSOCK_POLLIN};
	struct net_socket_service_event pev = {
		.event = {
			.events = ZSOCK_POLLIN,
			.revents = ZSOCK_POLLIN,
		},
	};
	uint8_t buf[sizeof(answer)];
	int sock, client, ready, ret, err;

	sock = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, NET_IPPROTO_UDP);
	zassert_true(sock >= 0, "Cannot create socket");
	zassert_ok(zsock_bind(sock, (struct net_sockaddr *)&local, sizeof(local)),
		   "Cannot bind (%d)", errno);
	client = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, NET_IPPROTO_UDP);
	zassert_true(client >= 0, "Cannot create client socket");
	to.sin_addr = my_addr2;
	(void)zsock_sendto(client, answer, sizeof(answer), 0, (struct net_sockaddr *)&to,
			   sizeof(to));
	pollfd.fd = sock;
	ready = zsock_poll(&pollfd, 1, 1000);

	/* No dispatcher in the socket's slot */
	struct dns_socket_dispatcher *table[sock + 1];

	memset(table, 0, sizeof(table));
	pev.event.fd = sock;
	pev.user_data = table;
	dns_dispatcher_svc_handler(&pev);

	ret = zsock_recv(sock, buf, sizeof(buf), ZSOCK_MSG_DONTWAIT);
	err = errno;
	(void)zsock_close(client);
	(void)zsock_close(sock);

	zassert_equal(ready, 1, "The datagram did not arrive");
	zassert_true(ret < 0 && err == EAGAIN, "The datagram was left queued (%d)", ret);
}

#define STALE_HANDLER_STACK_SIZE 2048

static K_THREAD_STACK_DEFINE(stale_handler_stack, STALE_HANDLER_STACK_SIZE);
static struct k_thread stale_handler_thread;

static void stale_handler(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	dns_dispatcher_svc_handler(p1);
}

/* A socket polled readable may be closed, and its descriptor reused by one
 * with nothing queued, before the socket service dispatches the event. The
 * handler must then return rather than wait for data on the new socket: it
 * runs on the socket service thread, which would stall every socket it serves.
 */
ZTEST(dns_dispatcher, test_stale_readiness_does_not_block)
{
	static struct dns_socket_dispatcher fresh;
	static struct zsock_pollfd fresh_fds[1];
	struct net_sockaddr_in local = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(65126),
	};
	struct net_socket_service_event pev = {
		.event = {
			.revents = ZSOCK_POLLIN,
		},
	};
	/* Longer than the test waits, so that a handler that does wait still
	 * returns, and releases the dispatcher lock, before unregistering.
	 */
	struct zsock_timeval rcvtimeo = {
		.tv_sec = 2,
	};
	int sock, ret;

	sock = zsock_socket(NET_AF_INET, NET_SOCK_DGRAM, NET_IPPROTO_UDP);
	zassert_true(sock >= 0, "Cannot create socket");
	ret = zsock_setsockopt(sock, ZSOCK_SOL_SOCKET, ZSOCK_SO_RCVTIMEO, &rcvtimeo,
			       sizeof(rcvtimeo));
	if (ret != 0) {
		(void)zsock_close(sock);
	}
	zassert_ok(ret, "Cannot set receive timeout");
	fresh_fds[0].fd = sock;
	fresh_fds[0].events = ZSOCK_POLLIN;
	fresh.type = DNS_SOCKET_RESOLVER;
	fresh.cb = test_dispatch_cb;
	fresh.fds = fresh_fds;
	fresh.fds_len = ARRAY_SIZE(fresh_fds);
	fresh.sock = sock;
	fresh.svc = &test_pair_svc;
	memcpy(&fresh.local_addr_storage, &local, sizeof(local));
	ret = dns_dispatcher_register(&fresh);
	if (ret != 0) {
		(void)zsock_close(sock);
	}
	zassert_ok(ret, "Cannot register dispatcher");

	/* What the socket service would hand over for the old socket */
	pev.event.fd = sock;
	pev.user_data = test_pair_svc.pev[0].user_data;

	k_thread_create(&stale_handler_thread, stale_handler_stack,
			K_THREAD_STACK_SIZEOF(stale_handler_stack), stale_handler, &pev, NULL,
			NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	ret = k_thread_join(&stale_handler_thread, K_SECONDS(1));
	if (ret != 0) {
		/* Its receive timeout ends the wait */
		zassert_ok(k_thread_join(&stale_handler_thread, K_SECONDS(3)),
			   "The handler kept waiting after its receive timeout");
	}

	(void)dns_dispatcher_unregister(&fresh);
	(void)zsock_close(sock);

	zassert_ok(ret, "The handler waited for data on a socket with nothing queued");
}

ZTEST(dns_dispatcher, test_dns_dispatcher_ephemeral_ports)
{
	static const char * const servers[] = { DNS_NAME_IPV4, DNS2_NAME_IPV4, NULL };
	struct dns_resolve_context *ctx;
	uint16_t port0, port1;
	int ret;

	ctx = dns_resolve_get_default();

	dns_resolve_close(ctx);

	/* Two DNS servers of the same address family, no explicit interface
	 * and the default local port 0. Each resolver socket bind() picks a
	 * distinct OS-assigned ephemeral port, but the dispatcher matches and
	 * deduplicates registrations by the stored local port. Unless the
	 * dispatcher reads back the bound port with getsockname(), both
	 * sockets look like the same port-0 resolver socket, and the second
	 * registration is rejected as a duplicate (swallowed by resolve.c as
	 * -EALREADY), leaving its socket undispatched.
	 */
	ret = dns_resolve_init(ctx, (const char **)servers, NULL);
	zassert_equal(ret, 0, "Cannot initialize DNS resolver (%d)", ret);

	zassert_true(ctx->servers[0].sock >= 0, "First server socket not open");
	zassert_true(ctx->servers[1].sock >= 0, "Second server socket not open");

	port0 = net_sin(net_sad(&ctx->servers[0].dispatcher.local_addr_storage))->sin_port;
	port1 = net_sin(net_sad(&ctx->servers[1].dispatcher.local_addr_storage))->sin_port;

	/* Both registrations must have captured their real ephemeral port. */
	zassert_not_equal(port0, 0, "First dispatcher port not resolved");
	zassert_not_equal(port1, 0, "Second dispatcher port not resolved");

	/* Distinct sockets must end up with distinct ports so that neither is
	 * treated as a duplicate of the other.
	 */
	zassert_not_equal(port0, port1,
			  "Both dispatchers share the same local port");

	dns_resolve_close(ctx);
}

ZTEST_SUITE(dns_dispatcher, NULL, test_init, NULL, NULL, NULL);
