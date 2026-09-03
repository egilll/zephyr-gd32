..
   Copyright (c) 2026, Ylhyra ehf.
   SPDX-License-Identifier: Apache-2.0

.. _mdns_responder_interface:

Multicast DNS Responder
#######################

.. contents::
   :local:
   :depth: 2

Overview
********

The Multicast DNS (mDNS) responder advertises the configured network hostname
in the link-local ``.local`` domain, as described by :rfc:`6762`. Enable it with
:kconfig:option:`CONFIG_MDNS_RESPONDER` and configure a meaningful hostname
with :kconfig:option:`CONFIG_NET_HOSTNAME`.

The responder listens on the IPv4 and IPv6 mDNS multicast groups for every
enabled interface by default. An allowlist or denylist can select interfaces
at build time. Applications can also enable or disable individual interfaces
at runtime when :kconfig:option:`CONFIG_MDNS_RESPONDER_RUNTIME_IFACE_CONTROL`
is enabled.

Supported responses
*******************

The responder answers the following questions for names it owns:

* ``A``, ``AAAA``, and ``ANY`` questions for ``<hostname>.local``;
* IPv4 ``in-addr.arpa`` and IPv6 ``ip6.arpa`` reverse ``PTR`` questions; and
* negative questions for unique host and reverse names using restricted
  ``NSEC`` records.

Multicast-response questions, questions requesting a unicast response, and
legacy unicast questions are supported. Legacy responses repeat the question
and transaction identifier, use a short TTL, and do not set the cache-flush
bit. Responses are scoped to the interface on which the question arrived.

DNS Service Discovery
*********************

Enable :kconfig:option:`CONFIG_DNS_SD` and
:kconfig:option:`CONFIG_MDNS_RESPONDER_DNS_SD` to advertise services using
DNS-Based Service Discovery (DNS-SD), as described by :rfc:`6763`. Register
static TCP or UDP services with :c:macro:`DNS_SD_REGISTER_TCP_SERVICE` or
:c:macro:`DNS_SD_REGISTER_UDP_SERVICE`.

The responder supports service browsing with ``PTR`` questions, direct
service-instance ``SRV``, ``TXT``, and ``ANY`` questions, and service-type
enumeration. Browse replies include the applicable ``SRV``, ``TXT``, ``A``,
and ``AAAA`` additional records. Known answers in multicast questions suppress
matching host, browse, and service-instance answers.

Applications can replace the externally supplied service-record array with
:c:func:`mdns_responder_set_ext_records`. The responder stores the supplied
pointers rather than copying their contents, so the array, its strings, TXT
data, and port values must remain valid until the records are removed. A
non-empty update made after startup schedules the standard two announcement
rounds when probing and DNS-SD announcements are enabled.

Probing and announcements
*************************

:kconfig:option:`CONFIG_MDNS_RESPONDER_PROBE` enables experimental hostname
probing. The responder waits for IPv4 address conflict detection and IPv6
duplicate address detection before advertising an address. It sends repeated
startup and address-change announcements, and sends goodbye records when an
advertised address is removed.

DNS-SD records are included in announcements by default when
:kconfig:option:`CONFIG_MDNS_RESPONDER_ANNOUNCE_DNS_SD` is enabled. Service
instance probing, conflict defense, and a runtime service update/removal
lifecycle are not currently provided.

Client support and limitations
******************************

:kconfig:option:`CONFIG_MDNS_RESOLVER` routes ordinary resolver queries for
``.local`` names over mDNS. It is a one-shot resolver; Zephyr does not
currently provide a persistent mDNS cache or a continuous DNS-SD browse and
resolve subscription API.

The responder does not currently aggregate and randomly delay shared-record
responses, suppress duplicate questions, or combine truncated multipacket
known-answer queries. DNS-SD subtypes and domains other than ``local`` are not
supported.

See :zephyr:code-sample:`mdns-responder` for a complete responder and DNS-SD
advertisement example.

API Reference
*************

.. doxygengroup:: mdns_responder

.. doxygengroup:: dns_sd
