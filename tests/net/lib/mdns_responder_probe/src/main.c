/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdns_resp_probe_test);

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <ipv6.h>

#include <zephyr/net/dns_sd.h>
#include <zephyr/net/dummy.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/mdns_responder.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/socket.h>
#include <zephyr/ztest.h>

#include "dns_pack.h"
#include "dns_sd.h"

/* Why this is a separate test suite/directory instead of living in
 * tests/net/lib/mdns_responder/:
 *
 * That suite's prj.conf does not enable CONFIG_MDNS_RESPONDER_PROBE, and its
 * tests drive dns_read() directly against hand-built packets on a two-iface
 * setup -- synchronous, with no real RFC 6762 probe/announce timing at all.
 *
 * This suite exists to exercise the real, timed probe -> init_listener ->
 * announce sequence (CONFIG_MDNS_RESPONDER_PROBE), on a single iface, with
 * outbound packets captured via the iface's .send callback instead of being
 * fed synchronously to dns_read(). test_announce_includes_dns_sd_records()
 * below needs that real sequence: it registers a DNS-SD record and then
 * waits out PROBE_SEQUENCE_TIMEOUT for a real unsolicited announce to be
 * sent, to confirm the announce includes the service's PTR/SRV/TXT records
 * (RFC 6762 8.3), not just its address records.
 *
 * Folding this into tests/net/lib/mdns_responder/'s existing ZTEST_SUITE
 * would mean bolting this single-iface, real-timing, packet-capture fixture
 * onto that suite's incompatible two-iface, synchronous one in the same
 * main.c, and adding a multi-second real sleep to any scenario that also
 * builds today's fast unit tests unless they're kept in separate tests.yaml
 * scenarios anyway -- at which point little is actually saved by sharing the
 * directory.
 */

#define RESPONSE_TIMEOUT_MS 250
#define RESPONSE_TIMEOUT    (K_MSEC(RESPONSE_TIMEOUT_MS))

/* RFC 6762 8.1 PROBE_TIMEOUT is 1750ms, plus up to a 250ms random initial
 * delay (see mdns_addr_event_handler()'s probe_delay) before the probe even
 * starts, so the responder cannot become reachable, and the announce cannot
 * be sent, before ~2s. Instead of sleeping the full worst case
 * unconditionally (which needlessly stretches every CI run), poll for the
 * actual response/announce and stop as soon as it arrives, bounding the
 * total wait at this worst-case cap.
 */
#define PROBE_SEQUENCE_TIMEOUT_MS 4000
#define PROBE_MAX_POLLS           (PROBE_SEQUENCE_TIMEOUT_MS / RESPONSE_TIMEOUT_MS)

static struct net_if *iface1;

static struct net_if_test {
	uint8_t idx; /* not used for anything, just a dummy value */
	uint8_t mac_addr[sizeof(struct net_eth_addr)];
	struct net_linkaddr ll_addr;
} net_iface1_data;

static const uint8_t ipv6_hdr_start[] = {0x60, 0x05, 0xe7, 0x00};

static const uint8_t ipv6_hdr_rest[] = {0x11, 0xff, 0xfe, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x9f, 0x74, 0x88, 0x9c, 0x1b, 0x44, 0x72, 0x39,
					0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb};

static uint8_t mdns_server_ipv6_addr[] = {0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xfb};

static struct net_in6_addr ll_addr = {
	{{0xfe, 0x80, 0x43, 0xb8, 0, 0, 0, 0, 0x9f, 0x74, 0x88, 0x9c, 0x1b, 0x44, 0x72, 0x39}}};

static struct net_in6_addr sender_ll_addr = {
	{{0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0x9f, 0x74, 0x88, 0x9c, 0x1b, 0x44, 0x72, 0x39}}};

static bool test_started;
static struct k_sem wait_data;
static struct net_pkt *response_pkts[8];
static size_t responses_count;

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
		data->mac_addr[5] = 0x01;
	}

	memcpy(data->ll_addr.addr, data->mac_addr, sizeof(data->mac_addr));
	data->ll_addr.len = 6U;

	return data->mac_addr;
}

static void net_iface_init(struct net_if *iface)
{
	uint8_t *mac = net_iface_get_mac(net_if_get_device(iface));

	net_if_set_link_addr(iface, mac, sizeof(struct net_eth_addr), NET_LINK_ETHERNET);
	net_if_flag_set(iface, NET_IF_IPV6_NO_ND);
}

static int sender_iface(const struct device *dev, struct net_pkt *pkt)
{
	struct net_ipv6_hdr *hdr;

	if (!pkt->buffer) {
		return -ENODATA;
	}

	if (test_started && net_pkt_family(pkt) == NET_AF_INET6) {
		hdr = NET_IPV6_HDR(pkt);

		if (net_ipv6_addr_cmp_raw(hdr->dst, mdns_server_ipv6_addr) &&
		    responses_count < ARRAY_SIZE(response_pkts)) {
			net_pkt_ref(pkt);
			response_pkts[responses_count++] = pkt;
			k_sem_give(&wait_data);
		}
	}

	return 0;
}

static struct dummy_api net_iface_api = {
	.iface_api.init = net_iface_init,
	.send = sender_iface,
};

#define _ETH_L2_LAYER    DUMMY_L2
#define _ETH_L2_CTX_TYPE NET_L2_GET_CTX_TYPE(DUMMY_L2)

NET_DEVICE_INIT_INSTANCE(net_iface1_test, "iface1", iface1, NULL, NULL, &net_iface1_data, NULL,
			 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &net_iface_api, _ETH_L2_LAYER,
			 _ETH_L2_CTX_TYPE, 127);

static void *test_setup(void)
{
	struct net_if_addr *ifaddr;

	memset(response_pkts, 0, sizeof(response_pkts));

	k_sem_init(&wait_data, 0, UINT_MAX);

	iface1 = net_if_get_by_index(1);
	zassert_not_null(iface1, "Iface1 is NULL");

	((struct net_if_test *)net_if_get_device(iface1)->data)->idx = net_if_get_by_iface(iface1);

	/* An IPv6 address is set up unconditionally (regardless of which test
	 * runs) so that the "is the responder now reachable" check -- an AAAA
	 * query for the hostname, same pattern proven in
	 * tests/net/lib/mdns_responder/ -- can be reused as-is. init_listener()
	 * opens both the IPv4 and IPv6 listeners together in one call, so this
	 * is a valid way to observe whether init_listener() ran at all,
	 * regardless of which address family triggered the race being tested.
	 * On its own it's also enough to drive a full, non-racing
	 * probe -> init_listener -> announce cycle -- no need for IPv4/DHCP
	 * involvement in the DNS-SD announce test.
	 */
	ifaddr = net_if_ipv6_addr_add(iface1, &ll_addr, NET_ADDR_MANUAL, 0);
	zassert_not_null(ifaddr, "Failed to add LL-addr");
	ifaddr->addr_state = NET_ADDR_PREFERRED;

	net_ipv6_nbr_add(iface1, &sender_ll_addr, net_if_get_link_addr(iface1), false,
			 NET_IPV6_NBR_STATE_STATIC);

	net_if_up(iface1);

	return NULL;
}

static void before(void *d)
{
	ARG_UNUSED(d);

	responses_count = 0;
	test_started = true;
}

static void cleanup(void *d)
{
	ARG_UNUSED(d);

	test_started = false;

	for (size_t i = 0; i < responses_count; ++i) {
		if (response_pkts[i]) {
			net_pkt_unref(response_pkts[i]);
			response_pkts[i] = NULL;
		}
	}

	while (k_sem_take(&wait_data, K_NO_WAIT) == 0) {
		/* NOP */
	}
}

static void send_msg(const uint8_t *data, size_t len)
{
	struct net_pkt *pkt;
	int res;

	pkt = net_pkt_alloc_with_buffer(iface1, NET_IPV6UDPH_LEN + len, NET_AF_UNSPEC, 0,
					K_FOREVER);
	zassert_not_null(pkt, "PKT is null");

	res = net_pkt_write(pkt, ipv6_hdr_start, sizeof(ipv6_hdr_start));
	zassert_equal(res, 0, "pkt write for header start failed");

	res = net_pkt_write_be16(pkt, len + NET_UDPH_LEN);
	zassert_equal(res, 0, "pkt write for header length failed");

	res = net_pkt_write(pkt, ipv6_hdr_rest, sizeof(ipv6_hdr_rest));
	zassert_equal(res, 0, "pkt write for rest of the header failed");

	res = net_pkt_write_be16(pkt, 5353);
	zassert_equal(res, 0, "pkt write for UDP src port failed");

	res = net_pkt_write_be16(pkt, 5353);
	zassert_equal(res, 0, "pkt write for UDP dst port failed");

	res = net_pkt_write_be16(pkt, len + NET_UDPH_LEN);
	zassert_equal(res, 0, "pkt write for UDP length failed");

	/* to simplify testing checking of UDP checksum is disabled in prj.conf */
	res = net_pkt_write_be16(pkt, 0);
	zassert_equal(res, 0, "net_pkt_write_be16() for UDP checksum failed");

	res = net_pkt_write(pkt, data, len);
	zassert_equal(res, 0, "net_pkt_write() for data failed");

	res = net_recv_data(iface1, pkt);
	zassert_equal(res, 0, "net_recv_data() failed");
}

/* Basic mDNS query for zephyr.local (AAAA) -- same query used to probe
 * responder reachability in tests/net/lib/mdns_responder/.
 */
static const uint8_t zephyr_local_query[] = {
	/* Header */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* zephyr.local */
	0x06, 0x7a, 0x65, 0x70, 0x68, 0x79, 0x72, 0x05, 0x6c, 0x6f, 0x63, 0x61, 0x6c, 0x00,
	/* AAAA record */
	0x00, 0x1c, 0x00, 0x01};

/* True only for a genuine mDNS response to our query: the QR bit is set and
 * at least one answer is present. The responder also multicasts its probe
 * queries (QR=0) and unsolicited announces to ff02::fb, which the iface .send
 * hook captures too -- those must not be mistaken for reachability, otherwise
 * the test could pass off probe/announce traffic before init_listener() has
 * actually opened the listener socket.
 */
static bool pkt_is_query_response(struct net_pkt *pkt)
{
	uint16_t flags;
	uint16_t ancount;

	net_pkt_cursor_init(pkt);
	net_pkt_set_overwrite(pkt, true);

	/* DNS header: id(2), flags(2), qdcount(2), ancount(2), ... */
	if (net_pkt_skip(pkt, NET_IPV6UDPH_LEN) || net_pkt_skip(pkt, 2) ||
	    net_pkt_read_be16(pkt, &flags) || net_pkt_skip(pkt, 2) ||
	    net_pkt_read_be16(pkt, &ancount)) {
		return false;
	}

	return (flags & BIT(15)) != 0 && ancount > 0;
}

static bool responder_answers_query(void)
{
	for (size_t i = 0; i < responses_count; i++) {
		if (response_pkts[i] != NULL) {
			net_pkt_unref(response_pkts[i]);
			response_pkts[i] = NULL;
		}
	}
	responses_count = 0;
	while (k_sem_take(&wait_data, K_NO_WAIT) == 0) {
		/* drain stale responses */
	}

	send_msg(zephyr_local_query, sizeof(zephyr_local_query));

	/* Accept only a real response; keep draining any probe/announce noise
	 * captured on the mDNS group until a genuine answer arrives or the
	 * per-poll window elapses.
	 */
	while (k_sem_take(&wait_data, RESPONSE_TIMEOUT) == 0) {
		for (size_t i = 0; i < responses_count; i++) {
			if (response_pkts[i] != NULL &&
			    pkt_is_query_response(response_pkts[i])) {
				return true;
			}
		}
	}

	return false;
}

static bool wait_for_responder(void)
{
	for (int i = 0; i < PROBE_MAX_POLLS; ++i) {
		if (responder_answers_query()) {
			return true;
		}
	}

	return false;
}

/* Peeks at a captured packet's first Answer record without any fatal
 * assertions (unlike validate_label()/check_*_resp() elsewhere in this
 * codebase) -- needed here because this test doesn't know in advance which of
 * several captured announce packets (address-only vs. DNS-SD) it's looking
 * at, and a non-matching one must not abort the test.
 */
static bool packet_has_ptr_answer(struct net_pkt *pkt, const char *service, const char *proto,
				  const char *domain)
{
	const char *expect[3] = {service, proto, domain};
	uint8_t label_len;
	char buf[DNS_SD_INSTANCE_MAX_SIZE + 1];

	net_pkt_cursor_init(pkt);
	net_pkt_set_overwrite(pkt, true);

	if (net_pkt_skip(pkt, NET_IPV6UDPH_LEN) || net_pkt_skip(pkt, sizeof(struct dns_header))) {
		return false;
	}

	for (int i = 0; i < 3; i++) {
		if (net_pkt_read_u8(pkt, &label_len) != 0 || label_len == 0 ||
		    label_len != strlen(expect[i])) {
			return false;
		}
		if (net_pkt_read(pkt, buf, label_len) != 0) {
			return false;
		}
		buf[label_len] = '\0';
		if (strcmp(buf, expect[i]) != 0) {
			return false;
		}
	}

	/* terminating root label */
	if (net_pkt_read_u8(pkt, &label_len) != 0 || label_len != 0) {
		return false;
	}

	return true;
}

static bool skip_dns_name(struct net_pkt *pkt)
{
	uint8_t label_len;

	while (net_pkt_read_u8(pkt, &label_len) == 0) {
		if (label_len == 0U) {
			return true;
		}

		if ((label_len & 0xc0) == 0xc0) {
			return net_pkt_skip(pkt, 1U) == 0;
		}

		if ((label_len & 0xc0) != 0U || label_len > DNS_LABEL_MAX_SIZE ||
		    net_pkt_skip(pkt, label_len) != 0) {
			return false;
		}
	}

	return false;
}

static void check_complete_dns_sd_announce(struct net_pkt *pkt)
{
	struct dns_header header;
	uint16_t answer_count;
	bool found_address = false;
	bool found_ptr = false;
	bool found_srv = false;
	bool found_txt = false;

	net_pkt_cursor_init(pkt);
	net_pkt_set_overwrite(pkt, true);
	zassert_ok(net_pkt_skip(pkt, NET_IPV6UDPH_LEN), "Missing IP/UDP header");
	zassert_ok(net_pkt_read(pkt, &header, sizeof(header)), "Missing DNS header");
	zassert_equal(net_ntohs(header.id), 0U, "Announcement has a transaction ID");
	zassert_equal(net_ntohs(header.flags), BIT(15) | BIT(10), "Unexpected announcement flags");
	zassert_equal(net_ntohs(header.qdcount), 0U, "Announcement contains a question");
	zassert_equal(net_ntohs(header.nscount), 0U, "Announcement contains authority records");
	zassert_equal(net_ntohs(header.arcount), 0U,
		      "Announcement placed records in the Additional section");

	answer_count = net_ntohs(header.ancount);
	zassert_true(answer_count >= 4U, "Incomplete DNS-SD announcement");

	for (uint16_t i = 0U; i < answer_count; ++i) {
		uint32_t expected_ttl;
		uint32_t ttl;
		uint16_t class_;
		uint16_t expected_class;
		uint16_t rdlength;
		uint16_t type;

		zassert_true(skip_dns_name(pkt), "Invalid answer owner name");
		zassert_ok(net_pkt_read_be16(pkt, &type), "Missing answer type");
		zassert_ok(net_pkt_read_be16(pkt, &class_), "Missing answer class");
		zassert_ok(net_pkt_read_be32(pkt, &ttl), "Missing answer TTL");
		zassert_ok(net_pkt_read_be16(pkt, &rdlength), "Missing RDATA length");

		switch (type) {
		case DNS_RR_TYPE_PTR:
			expected_class = DNS_CLASS_IN;
			expected_ttl = DNS_SD_PTR_TTL;
			found_ptr = true;
			break;
		case DNS_RR_TYPE_TXT:
			expected_class = DNS_CLASS_IN | DNS_CLASS_FLUSH;
			expected_ttl = DNS_SD_TXT_TTL;
			found_txt = true;
			break;
		case DNS_RR_TYPE_SRV:
			expected_class = DNS_CLASS_IN | DNS_CLASS_FLUSH;
			expected_ttl = DNS_SD_SRV_TTL;
			found_srv = true;
			break;
		case DNS_RR_TYPE_A:
			expected_class = DNS_CLASS_IN | DNS_CLASS_FLUSH;
			expected_ttl = DNS_SD_A_TTL;
			found_address = true;
			break;
		case DNS_RR_TYPE_AAAA:
			expected_class = DNS_CLASS_IN | DNS_CLASS_FLUSH;
			expected_ttl = DNS_SD_AAAA_TTL;
			found_address = true;
			break;
		default:
			zassert_unreachable("Unexpected DNS-SD announcement record type");
			return;
		}

		zassert_equal(class_, expected_class, "Unexpected class for type %u", type);
		zassert_equal(ttl, expected_ttl, "Unexpected TTL for type %u", type);
		zassert_ok(net_pkt_skip(pkt, rdlength), "Truncated answer RDATA");
	}

	zassert_true(found_ptr, "Announcement is missing its PTR record");
	zassert_true(found_srv, "Announcement is missing its SRV record");
	zassert_true(found_txt, "Announcement is missing its TXT record");
	zassert_true(found_address, "Announcement is missing its address record");
}

static bool packet_has_aaaa_answer(struct net_pkt *pkt, const struct net_in6_addr *addr,
				   uint32_t ttl)
{
	struct dns_header header;
	uint16_t answer_count;

	net_pkt_cursor_init(pkt);
	net_pkt_set_overwrite(pkt, true);

	if (net_pkt_skip(pkt, NET_IPV6UDPH_LEN) != 0 ||
	    net_pkt_read(pkt, &header, sizeof(header)) != 0 ||
	    (net_ntohs(header.flags) & BIT(15)) == 0 || net_ntohs(header.qdcount) != 0U) {
		return false;
	}

	answer_count = net_ntohs(header.ancount);
	for (uint16_t i = 0U; i < answer_count; ++i) {
		struct net_in6_addr answer_addr;
		uint32_t answer_ttl;
		uint16_t class_;
		uint16_t type;
		uint16_t rdlength;

		if (!skip_dns_name(pkt) || net_pkt_read_be16(pkt, &type) != 0 ||
		    net_pkt_read_be16(pkt, &class_) != 0 ||
		    net_pkt_read_be32(pkt, &answer_ttl) != 0 ||
		    net_pkt_read_be16(pkt, &rdlength) != 0) {
			return false;
		}

		if (type == DNS_RR_TYPE_AAAA && class_ == (DNS_CLASS_IN | DNS_CLASS_FLUSH) &&
		    rdlength == sizeof(answer_addr)) {
			if (net_pkt_read(pkt, &answer_addr, sizeof(answer_addr)) != 0) {
				return false;
			}

			if (answer_ttl == ttl && net_ipv6_addr_cmp(&answer_addr, addr)) {
				return true;
			}
		} else if (net_pkt_skip(pkt, rdlength) != 0) {
			return false;
		}
	}

	return false;
}

static void clear_responses(void)
{
	for (size_t i = 0; i < responses_count; ++i) {
		if (response_pkts[i] != NULL) {
			net_pkt_unref(response_pkts[i]);
			response_pkts[i] = NULL;
		}
	}

	responses_count = 0U;
	while (k_sem_take(&wait_data, K_NO_WAIT) == 0) {
	}
}

static bool wait_for_aaaa_answers(const struct net_in6_addr *addr, uint32_t ttl,
				  size_t expected_count)
{
	size_t answer_count = 0U;
	size_t checked_count = 0U;

	for (int poll = 0; poll < PROBE_MAX_POLLS; ++poll) {
		for (size_t i = checked_count; i < responses_count; ++i) {
			if (packet_has_aaaa_answer(response_pkts[i], addr, ttl)) {
				answer_count++;
			}
		}
		if (answer_count >= expected_count) {
			return true;
		}

		checked_count = responses_count;
		(void)k_sem_take(&wait_data, RESPONSE_TIMEOUT);
	}

	return false;
}

ZTEST(test_mdns_responder_probe, test_address_change_announce_and_goodbye)
{
	static const struct net_in6_addr changed_addr = {
		{{0xfe, 0x80, 0x43, 0xb8, 0, 0, 0, 0, 0x9f, 0x74, 0x88, 0x9c, 0x1b, 0x44, 0x72,
		  0x40}}};
	struct net_if_addr *ifaddr;

	zassert_true(wait_for_responder(), "Responder did not finish startup");
	k_sleep(RESPONSE_TIMEOUT);
	clear_responses();

	ifaddr = net_if_ipv6_addr_add(iface1, &changed_addr, NET_ADDR_MANUAL, 0);
	zassert_not_null(ifaddr, "Failed to add changed address");
	ifaddr->addr_state = NET_ADDR_PREFERRED;
	zassert_true(wait_for_aaaa_answers(&changed_addr, CONFIG_MDNS_RESPONDER_TTL, 2U),
		     "Added address was not announced twice");

	clear_responses();
	zassert_true(net_if_ipv6_addr_rm(iface1, &changed_addr),
		     "Failed to remove changed address");
	zassert_true(wait_for_aaaa_answers(&changed_addr, 0U, 1U),
		     "Removed address was not sent with TTL zero");

	clear_responses();
	net_mgmt_event_notify_with_info(NET_EVENT_IPV6_ADDR_DEL, iface1, &changed_addr,
					sizeof(changed_addr));
	zassert_equal(k_sem_take(&wait_data, RESPONSE_TIMEOUT), -EAGAIN,
		      "Retired address was announced again");
}

ZTEST(test_mdns_responder_probe, test_tentative_ipv6_waits_for_dad)
{
	struct net_if_addr *ifaddr;

	zassert_true(wait_for_responder(), "Responder did not finish startup");
	k_sleep(RESPONSE_TIMEOUT);
	clear_responses();

	ifaddr = net_if_ipv6_addr_lookup_by_iface(iface1, &ll_addr);
	zassert_not_null(ifaddr, "Test address is missing");
	net_if_flag_clear(iface1, NET_IF_IPV6_NO_ND);
	ifaddr->addr_state = NET_ADDR_TENTATIVE;
	net_mgmt_event_notify_with_info(NET_EVENT_IPV6_ADDR_ADD, iface1, &ll_addr, sizeof(ll_addr));
	k_sleep(RESPONSE_TIMEOUT);
	for (size_t i = 0; i < responses_count; ++i) {
		zassert_false(packet_has_aaaa_answer(response_pkts[i], &ll_addr,
						     CONFIG_MDNS_RESPONDER_TTL),
			      "Tentative IPv6 address was announced before DAD completed");
	}
	clear_responses();
	net_mgmt_event_notify_with_info(NET_EVENT_IPV6_ADDR_DEL, iface1, &ll_addr, sizeof(ll_addr));
	k_sleep(RESPONSE_TIMEOUT);
	for (size_t i = 0; i < responses_count; ++i) {
		zassert_false(packet_has_aaaa_answer(response_pkts[i], &ll_addr, 0U),
			      "Failed DAD produced a goodbye for an unannounced address");
	}
	clear_responses();

	net_mgmt_event_notify_with_info(NET_EVENT_IPV6_ADDR_ADD, iface1, &ll_addr, sizeof(ll_addr));
	ifaddr->addr_state = NET_ADDR_PREFERRED;
	net_mgmt_event_notify_with_info(NET_EVENT_IPV6_DAD_SUCCEED, iface1, &ll_addr,
					sizeof(ll_addr));
	zassert_true(wait_for_aaaa_answers(&ll_addr, CONFIG_MDNS_RESPONDER_TTL, 2U),
		     "Validated IPv6 address was not announced twice after DAD");
	net_if_flag_set(iface1, NET_IF_IPV6_NO_ND);
}

/* Regression test for the RFC 6762 8.3 announce-completeness feature:
 * send_announce() used to only ever announce address (A/AAAA) records on
 * startup/re-announce, never a registered DNS-SD service's PTR/SRV/TXT
 * records, even though RFC 6762 8.3 requires announcing *all* of a
 * responder's newly registered records. This registers one external DNS-SD
 * record, drives the responder through a normal (non-racing) bring-up, and
 * checks that at least one of the announce packets sent to the mDNS group
 * is a PTR answer for that service -- not just the address-only announce.
 */
ZTEST(test_mdns_responder_probe, test_announce_includes_dns_sd_records)
{
	static const uint16_t svc_port = sys_cpu_to_be16(5353);
	static const char instance[] = "zephyr";
	static const char service[] = "_zephyr";
	static const char proto[] = "_tcp";
	static const char domain[] = "local";
	static const struct dns_sd_rec record = {
		.instance = instance,
		.service = service,
		.proto = proto,
		.domain = domain,
		.text = DNS_SD_EMPTY_TXT,
		.text_size = sizeof(dns_sd_empty_txt),
		.port = &svc_port,
	};
	size_t dns_sd_announce_count = 0U;
	size_t checked_count = 0U;

	zassert_true(wait_for_responder(), "Responder did not finish startup");
	clear_responses();

	zassert_ok(mdns_responder_set_ext_records(&record, 1),
		   "Failed to register the external DNS-SD record");
	net_mgmt_event_notify(NET_EVENT_IF_UP, iface1);

	/* Poll for both required announcement rounds instead of sleeping the
	 * full worst-case probe window.
	 */
	for (int poll = 0; poll < PROBE_MAX_POLLS && dns_sd_announce_count < 2U; poll++) {
		k_sleep(RESPONSE_TIMEOUT);

		for (size_t i = checked_count; i < responses_count; i++) {
			if (packet_has_ptr_answer(response_pkts[i], service, proto, domain)) {
				check_complete_dns_sd_announce(response_pkts[i]);
				dns_sd_announce_count++;
			}
		}

		checked_count = responses_count;
	}

	zassert_true(responses_count > 0, "No announce packets were sent at all");

	zassert_equal(dns_sd_announce_count, 2U, "Expected two DNS-SD announcements, received %zu",
		      dns_sd_announce_count);
}

/* Regression test for the "DHCP-bound announce races ahead of the RFC 6762
 * probe" bug: start_announce() (fired on NET_EVENT_IPV4_DHCP_BOUND) used to
 * set do_announce = true unconditionally and immediately, before the probe
 * (a 0-250ms scheduling delay, then the ~1.75-3s RFC 6762 window) had even
 * started. do_init_listener() branches on that same do_announce flag to
 * decide whether to call init_listener() at all -- so on real hardware,
 * DHCP binding always won that race (same thread, no delay, immediately
 * after the address is added), meaning init_listener() -- and thus the
 * mDNS listener socket -- never started on any boot with DHCPv4 + probing
 * both enabled.
 *
 * This reproduces the exact race: net_if_ipv4_addr_add() (which schedules
 * the probe) immediately followed by a NET_EVENT_IPV4_DHCP_BOUND
 * notification, back to back with no delay in between, exactly mirroring
 * dhcpv4_handle_msg_ack()'s real sequence in dhcpv4.c. It then waits out
 * the full probe window and checks that the responder is reachable --
 * proving init_listener() actually ran despite the race.
 */
ZTEST(test_mdns_responder_probe, test_dhcp_bound_race_does_not_block_listener)
{
	static struct net_in_addr ipv4_addr = {{{192, 0, 2, 1}}};
	struct net_if_addr *ifaddr;

	ifaddr = net_if_ipv4_addr_add(iface1, &ipv4_addr, NET_ADDR_DHCP, 0);
	zassert_not_null(ifaddr, "Failed to add IPv4 address");

	/* No delay here on purpose -- this is the race itself. */
	net_mgmt_event_notify(NET_EVENT_IPV4_DHCP_BOUND, iface1);

	/* Poll for reachability instead of sleeping the full worst-case probe
	 * window: this returns as soon as the responder answers (~probe
	 * completion), while still bounding the wait at PROBE_SEQUENCE_TIMEOUT.
	 */
	zassert_true(wait_for_responder(),
		     "Responder never became reachable -- DHCP-bound announce blocked "
		     "init_listener() from ever running");
}

/* Same scenario as tests/net/lib/mdns_responder/'s
 * test_multi_question_compressed_dns_sd_query, but through a listener
 * brought up via the real probe sequence instead of that suite's
 * CONFIG_MDNS_RESPONDER_PROBE=n path. PTR wire-format correctness is
 * already covered there, so this only checks both questions get answered.
 */
ZTEST(test_mdns_responder_probe, test_multi_question_compressed_query_after_probe)
{
	static const uint8_t multi_question_query[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
		0x5f, 0x66, 0x6f, 0x6f, 0x04, 0x5f, 0x75, 0x64, 0x70, 0x05, 0x6c, 0x6f, 0x63,
		0x61, 0x6c, 0x00, 0x00, 0x0c, 0x00, 0x01, 0x07, 0x5f, 0x7a, 0x65, 0x70, 0x68,
		0x79, 0x72, 0x04, 0x5f, 0x74, 0x63, 0x70, 0xc0, 0x16, 0x00, 0x0c, 0x00, 0x01};
	static const uint16_t svc_port = sys_cpu_to_be16(5353);
	static const char instance[] = "zephyr";
	static const char foo_service[] = "_foo";
	static const char foo_proto[] = "_udp";
	static const char zephyr_service[] = "_zephyr";
	static const char zephyr_proto[] = "_tcp";
	static const char domain[] = "local";
	static const struct dns_sd_rec records[2] = {
		{
			.instance = instance,
			.service = foo_service,
			.proto = foo_proto,
			.domain = domain,
			.text = DNS_SD_EMPTY_TXT,
			.text_size = sizeof(dns_sd_empty_txt),
			.port = &svc_port,
		},
		{
			.instance = instance,
			.service = zephyr_service,
			.proto = zephyr_proto,
			.domain = domain,
			.text = DNS_SD_EMPTY_TXT,
			.text_size = sizeof(dns_sd_empty_txt),
			.port = &svc_port,
		},
	};
	int res;

	zassert_true(wait_for_responder(),
		     "Responder never became reachable via the probe sequence");

	zassert_ok(mdns_responder_set_ext_records(records, ARRAY_SIZE(records)),
		   "Failed to register the external DNS-SD records");

	/* responder_answers_query() above leaves its own AAAA reply captured. */
	responses_count = 0;
	while (k_sem_take(&wait_data, K_NO_WAIT) == 0) {
		/* NOP */
	}

	send_msg(multi_question_query, sizeof(multi_question_query));

	res = k_sem_take(&wait_data, RESPONSE_TIMEOUT);
	zassert_ok(res, "Did not receive a response to Question 1");
	res = k_sem_take(&wait_data, RESPONSE_TIMEOUT);
	zassert_ok(res, "Did not receive a response to Question 2 (the compressed one)");

	zassert_equal(responses_count, 2, "Expected exactly 2 responses, got %zu", responses_count);

	zassert_true(packet_has_ptr_answer(response_pkts[0], "_foo", "_udp", "local"),
		     "Response to Question 1 was not the expected PTR answer");
	zassert_true(packet_has_ptr_answer(response_pkts[1], "_zephyr", "_tcp", "local"),
		     "Response to Question 2 was not the expected PTR answer");
}

ZTEST_SUITE(test_mdns_responder_probe, NULL, test_setup, before, cleanup, NULL);
