/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32 ENET uses:
 * - private DMA-owned RX buffers, copied into Zephyr packets before frames
 *   are handed to the stack, and
 * - zero-copy TX, where DMA reads directly from `net_pkt` fragments.
 *
 * Notes about the vendor HAL (`gd32f4xx_enet.c`):
 * - We do not call `enet_init()` because it mixes PHY management and long
 *   polling loops; Zephyr PHY drivers handle link negotiation separately.
 * - We do not use the HAL's legacy `ENET_RXBUF_NUM/ENET_RXBUF_SIZE`-backed
 *   rxdesc_tab/txdesc_tab/rx_buff/tx_buff APIs (descriptor init helpers,
 *   enet_frame_receive/transmit wrappers). This driver owns the DMA rings and
 *   frame buffers.
 */

#define DT_DRV_COMPAT gd_gd32_ethernet

// TODO: doesn't build with CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST=n
// TODO: doesn't work with dynamic net bufs

#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/dt-bindings/clock/gd32f4xx-clocks.h>
#include <zephyr/devicetree/gpio.h>
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net_buf.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/lldp.h>
#include <zephyr/net/phy.h>
#include <zephyr/nvmem.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>
#include <ethernet/eth_stats.h>

#include "eth.h"

#include <gd32f4xx_enet.h>
#include <gd32f4xx_syscfg.h>
#include <gd32f4xx_rcu.h>
#include <gd32f4xx_gpio.h>

LOG_MODULE_REGISTER(eth_gd32, CONFIG_ETHERNET_LOG_LEVEL);

#define GD32_ETH_TX_TIMEOUT_MS  100U
#define GD32_ETH_FRAME_SIZE_MAX (NET_ETH_MTU + 18)

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1,
	     "GD32 ENET HAL supports only one instance");

#define GD32_OUI_B0 0xE0
#define GD32_OUI_B1 0x05
#define GD32_OUI_B2 0x1C

#define GD32_ETH_RX_DESC_COUNT CONFIG_ETH_GD32_RX_DESC_COUNT
#define GD32_ETH_TX_DESC_COUNT CONFIG_ETH_GD32_TX_DESC_COUNT

#define GD32_ETH_RX_BUF_SIZE CONFIG_NET_BUF_DATA_SIZE
#define GD32_DMA_POLL_DEMAND 1U

BUILD_ASSERT((GD32_ETH_RX_BUF_SIZE % 4U) == 0U, "RX buffer size must be 32-bit aligned");
BUILD_ASSERT(GD32_ETH_RX_BUF_SIZE <= 0x1FFF, "RX buffer size must fit RB1S field");
BUILD_ASSERT(GD32_ETH_TX_DESC_COUNT >= 4, "Need at least 4 TX descriptors");
BUILD_ASSERT(GD32_ETH_RX_DESC_COUNT >= 8, "Need at least 8 RX descriptors");
BUILD_ASSERT((GD32_ETH_RX_DESC_COUNT * CONFIG_NET_BUF_DATA_SIZE) >= GD32_ETH_FRAME_SIZE_MAX,
	     "RX ring too small for a full frame");

#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
BUILD_ASSERT(sizeof(enet_descriptors_struct) == 32, "Enhanced mode expects 32-byte descriptors");
#else
BUILD_ASSERT(sizeof(enet_descriptors_struct) == 16, "Normal mode expects 16-byte descriptors");
#endif

#if DT_INST_NODE_HAS_PROP(0, zephyr_mac_address_prefix)
BUILD_ASSERT(DT_INST_PROP_LEN(0, zephyr_mac_address_prefix) == 3,
	     "zephyr,mac-address-prefix must be 3 bytes (OUI)");
#endif

#if DT_HAS_CHOSEN(zephyr_dtcm)
#define GD32_DTCM_BASE DT_REG_ADDR(DT_CHOSEN(zephyr_dtcm))
#define GD32_DTCM_END  (DT_REG_ADDR(DT_CHOSEN(zephyr_dtcm)) + DT_REG_SIZE(DT_CHOSEN(zephyr_dtcm)))
#endif

struct eth_gd32_dma_rings {
	enet_descriptors_struct rx_desc[GD32_ETH_RX_DESC_COUNT];
	enet_descriptors_struct tx_desc[GD32_ETH_TX_DESC_COUNT];
};

static struct eth_gd32_dma_rings eth0_dma __nocache __aligned(4);
static uint8_t eth0_rx_dma_buf[GD32_ETH_RX_DESC_COUNT][GD32_ETH_RX_BUF_SIZE]
	__nocache __aligned(4);

void eth_gd32_delay(uint32_t ticks)
{
	if (k_is_pre_kernel() || k_is_in_isr()) {
		k_busy_wait(1);
		return;
	}

	if (ticks >= 1000000U) {
		k_sleep(K_MSEC(2));
	} else if (ticks >= 100000U) {
		k_sleep(K_MSEC(1));
	} else {
		k_yield();
	}
}

struct eth_gd32_config {
	uint16_t clk_mac;
	uint16_t clk_tx;
	uint16_t clk_rx;
	bool has_ptp;
	uint16_t clk_ptp;

	const struct pinctrl_dev_config *pcfg;
	uint8_t phy_mode;
	void (*config_func)(void);

#if DT_INST_PROP_LEN(0, resets) > 0
	struct reset_dt_spec rst0;
#endif
#if DT_INST_PROP_LEN(0, resets) > 1
	struct reset_dt_spec rst1;
#endif
#if DT_INST_PROP_LEN(0, resets) > 2
	struct reset_dt_spec rst2;
#endif
#if DT_INST_PROP_LEN(0, resets) > 3
	struct reset_dt_spec rst3;
#endif
};

struct eth_gd32_data {
	const struct device *dev;
	struct net_if *iface;
	uint8_t mac_addr[6];
	struct k_mutex tx_mutex;
	struct k_sem tx_desc_sem;
	struct k_spinlock tx_ring_lock;
	struct net_pkt *tx_pkt_refs[GD32_ETH_TX_DESC_COUNT];
	struct k_work rx_work;
	atomic_t rx_work_flags;
	struct k_work recover_work;
	atomic_t recovering;
	atomic_t link_up;
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
#if defined(CONFIG_ETH_GD32_DEBUG_COUNTERS)
	atomic_t dbg_isr;
	atomic_t dbg_rs;
	atomic_t dbg_rbu;
	atomic_t dbg_ro;
	atomic_t dbg_rxwork;
	atomic_t dbg_delivered;
	atomic_t dbg_alloc_fail;
#endif
	uint8_t mcast_hash_refcnt[64];
	uint16_t rx_tail;
	uint16_t tx_next_to_use;
	uint16_t tx_next_to_clean;
	uint16_t tx_descs_in_use;
	bool hw_ready;
};

struct eth_gd32_tx_reclaim {
	struct net_pkt *pkts[GD32_ETH_TX_DESC_COUNT];
	uint16_t pkt_count;
	uint16_t desc_count;
};

static void eth_gd32_mac_dma_default_config(void);
static void eth_gd32_rings_configure(struct eth_gd32_data *data);
static void eth_gd32_dma_irqs_enable(void);
static void eth_gd32_dma_irqs_clear_all(void);
static int eth_gd32_hw_reset_and_configure(struct eth_gd32_data *data);
static void eth_gd32_reclaim_tx(const struct device *dev);

static inline bool gd32_dma_addr_ok(uintptr_t addr, size_t len)
{
#if DT_HAS_CHOSEN(zephyr_dtcm)
	uintptr_t end = addr + len;
	if (addr < end && addr < GD32_DTCM_END && end > GD32_DTCM_BASE) {
		return false;
	}
#else
	ARG_UNUSED(addr);
	ARG_UNUSED(len);
#endif
	return true;
}

static inline void gd32_cache_invalidate(const void *addr, size_t len)
{
	(void)sys_cache_data_invd_range((void *)addr, len);
}

static inline void gd32_cache_clean(const void *addr, size_t len)
{
	(void)sys_cache_data_flush_range((void *)addr, len);
}

static inline bool rx_desc_dma_owned(const enet_descriptors_struct *desc)
{
	return (desc->status & ENET_RDES0_DAV) != 0U;
}

static inline bool tx_desc_dma_owned(const enet_descriptors_struct *desc)
{
	return (desc->status & ENET_TDES0_DAV) != 0U;
}

static inline uint16_t modulo_inc(uint16_t idx, uint16_t max)
{
	idx++;
	return (idx < max) ? idx : 0U;
}

static inline void eth_gd32_rx_buf_prepare_for_dma(void *buf, size_t len)
{
	/*
	 * DMA writes RX data into the buffer; discard any cached lines so that:
	 * - we don't write back stale data over the DMA writes later, and
	 * - subsequent CPU reads see the freshly received bytes.
	 */
	gd32_cache_invalidate(buf, len);
}

static inline void eth_gd32_rx_desc_give_to_dma(enet_descriptors_struct *desc, void *buf)
{
	desc->buffer1_addr = (uint32_t)(uintptr_t)buf;
	barrier_dmem_fence_full();
	desc->status = ENET_RDES0_DAV;
}

static inline void eth_gd32_dma_tx_poll_demand(void)
{
	/*
	 * ENET_DMA_TPEN/ENET_DMA_RPEN are "write any value to resume/poll"
	 * registers. Use a fixed non-zero value for readability.
	 */
	ENET_DMA_TPEN = GD32_DMA_POLL_DEMAND;
}

static inline void eth_gd32_dma_rx_poll_demand(void)
{
	ENET_DMA_RPEN = GD32_DMA_POLL_DEMAND;
}

static inline void eth_gd32_tx_desc_reset(enet_descriptors_struct *desc)
{
	desc->status = ENET_TDES0_TCHM;
	desc->control_buffer_size = 0U;
	desc->buffer1_addr = 0U;
}

static void eth_gd32_tx_reclaim_flush(struct eth_gd32_data *data,
				      const struct eth_gd32_tx_reclaim *reclaim)
{
	for (uint16_t i = 0U; i < reclaim->pkt_count; i++) {
		net_pkt_unref(reclaim->pkts[i]);
	}

	for (uint16_t i = 0U; i < reclaim->desc_count; i++) {
		k_sem_give(&data->tx_desc_sem);
	}
}

static void eth_gd32_tx_reclaim_completed(struct eth_gd32_data *data,
					  struct eth_gd32_tx_reclaim *reclaim)
{
	k_spinlock_key_t key = k_spin_lock(&data->tx_ring_lock);

	__ASSERT_NO_MSG(data->tx_descs_in_use <= GD32_ETH_TX_DESC_COUNT);

	while (data->tx_descs_in_use > 0U) {
		uint16_t idx = data->tx_next_to_clean;
		enet_descriptors_struct *desc = &eth0_dma.tx_desc[idx];

		if (tx_desc_dma_owned(desc)) {
			break;
		}

		if (data->tx_pkt_refs[idx] != NULL) {
			__ASSERT_NO_MSG(reclaim->pkt_count < ARRAY_SIZE(reclaim->pkts));
			reclaim->pkts[reclaim->pkt_count++] = data->tx_pkt_refs[idx];
			data->tx_pkt_refs[idx] = NULL;
		}

		eth_gd32_tx_desc_reset(desc);
		data->tx_next_to_clean = modulo_inc(idx, GD32_ETH_TX_DESC_COUNT);
		data->tx_descs_in_use--;
		reclaim->desc_count++;
	}

	k_spin_unlock(&data->tx_ring_lock, key);
}

static void eth_gd32_tx_desc_unreserve(struct eth_gd32_data *data, uint16_t desc_count)
{
	for (uint16_t i = 0U; i < desc_count; i++) {
		k_sem_give(&data->tx_desc_sem);
	}
}

static int eth_gd32_tx_desc_reserve(const struct device *dev, uint16_t desc_count)
{
	struct eth_gd32_data *data = dev->data;
	uint16_t reserved = 0U;

	while (reserved < desc_count) {
		if (k_sem_take(&data->tx_desc_sem, K_MSEC(GD32_ETH_TX_TIMEOUT_MS)) == 0) {
			reserved++;
			continue;
		}

		eth_gd32_reclaim_tx(dev);
		if (k_sem_take(&data->tx_desc_sem, K_NO_WAIT) == 0) {
			reserved++;
			continue;
		}

		eth_gd32_tx_desc_unreserve(data, reserved);
		return -ENOBUFS;
	}

	return 0;
}

static void eth_gd32_tx_reset(struct eth_gd32_data *data)
{
	struct eth_gd32_tx_reclaim reclaim = {0};
	k_spinlock_key_t key = k_spin_lock(&data->tx_ring_lock);

	for (uint16_t i = 0U; i < GD32_ETH_TX_DESC_COUNT; i++) {
		if (data->tx_pkt_refs[i] != NULL) {
			__ASSERT_NO_MSG(reclaim.pkt_count < ARRAY_SIZE(reclaim.pkts));
			reclaim.pkts[reclaim.pkt_count++] = data->tx_pkt_refs[i];
			data->tx_pkt_refs[i] = NULL;
		}
	}

	data->tx_next_to_use = 0U;
	data->tx_next_to_clean = 0U;
	data->tx_descs_in_use = 0U;

	k_spin_unlock(&data->tx_ring_lock, key);

	k_sem_reset(&data->tx_desc_sem);
	for (uint16_t i = 0U; i < GD32_ETH_TX_DESC_COUNT; i++) {
		k_sem_give(&data->tx_desc_sem);
	}

	eth_gd32_tx_reclaim_flush(data, &reclaim);
}

enum {
	GD32_RX_WORK_FLAG_RBU = BIT(0),
	GD32_RX_WORK_FLAG_KICK = BIT(1),
};

static void eth_gd32_dma_irqs_enable(void)
{
	enet_interrupt_enable(ENET_DMA_INT_NIE);
	enet_interrupt_enable(ENET_DMA_INT_AIE);
	enet_interrupt_enable(ENET_DMA_INT_RIE);
	enet_interrupt_enable(ENET_DMA_INT_RBUIE);
	enet_interrupt_enable(ENET_DMA_INT_RPSIE);
	enet_interrupt_enable(ENET_DMA_INT_ROIE);
	enet_interrupt_enable(ENET_DMA_INT_TIE);
	enet_interrupt_enable(ENET_DMA_INT_TBUIE);
	enet_interrupt_enable(ENET_DMA_INT_TPSIE);
	enet_interrupt_enable(ENET_DMA_INT_TUIE);
	enet_interrupt_enable(ENET_DMA_INT_FBEIE);
}

static void eth_gd32_dma_irqs_clear_all(void)
{
	ENET_DMA_STAT = ENET_DMA_STAT_TS | ENET_DMA_STAT_TBU | ENET_DMA_STAT_TJT |
			ENET_DMA_STAT_TU | ENET_DMA_STAT_RS | ENET_DMA_STAT_RBU | ENET_DMA_STAT_RO |
			ENET_DMA_STAT_RPS | ENET_DMA_STAT_TPS | ENET_DMA_STAT_RWT |
			ENET_DMA_STAT_ET | ENET_DMA_STAT_FBE | ENET_DMA_STAT_ER | ENET_DMA_STAT_AI |
			ENET_DMA_STAT_NI;
}

static void eth_gd32_rings_configure(struct eth_gd32_data *data)
{
	for (uint16_t i = 0U; i < GD32_ETH_RX_DESC_COUNT; i++) {
		enet_descriptors_struct *desc = &eth0_dma.rx_desc[i];

		desc->control_buffer_size =
			ENET_RDES1_RCHM | ((uint32_t)GD32_ETH_RX_BUF_SIZE & ENET_RDES1_RB1S);
		enet_rx_desc_immediate_receive_complete_interrupt(desc);
		desc->buffer2_next_desc_addr =
			(uint32_t)(uintptr_t)&eth0_dma
				.rx_desc[modulo_inc(i, GD32_ETH_RX_DESC_COUNT)];

		eth_gd32_rx_buf_prepare_for_dma(eth0_rx_dma_buf[i], GD32_ETH_RX_BUF_SIZE);
		eth_gd32_rx_desc_give_to_dma(desc, eth0_rx_dma_buf[i]);
	}

	for (uint16_t i = 0U; i < GD32_ETH_TX_DESC_COUNT; i++) {
		enet_descriptors_struct *desc = &eth0_dma.tx_desc[i];

		eth_gd32_tx_desc_reset(desc);
		desc->buffer2_next_desc_addr =
			(uint32_t)(uintptr_t)&eth0_dma
				.tx_desc[modulo_inc(i, GD32_ETH_TX_DESC_COUNT)];
		data->tx_pkt_refs[i] = NULL;
	}

	data->rx_tail = 0U;
	data->tx_next_to_use = 0U;
	data->tx_next_to_clean = 0U;
	data->tx_descs_in_use = 0U;

	ENET_DMA_RDTADDR = (uint32_t)(uintptr_t)eth0_dma.rx_desc;
	ENET_DMA_TDTADDR = (uint32_t)(uintptr_t)eth0_dma.tx_desc;
	barrier_dmem_fence_full();
}

static int eth_gd32_hw_reset_and_configure(struct eth_gd32_data *data)
{
	enet_disable();
	enet_deinit();

	if (enet_software_reset() != SUCCESS) {
		return -EIO;
	}

	eth_gd32_mac_dma_default_config();
	enet_mac_address_set(ENET_MAC_ADDRESS0, data->mac_addr);
#if !defined(CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST)
	eth_gd32_mcast_hash_sync(data);
#endif

	eth_gd32_rings_configure(data);
	eth_gd32_dma_irqs_enable();
	eth_gd32_dma_irqs_clear_all();

	return 0;
}

static void eth_gd32_reclaim_tx(const struct device *dev)
{
	struct eth_gd32_data *data = dev->data;
	struct eth_gd32_tx_reclaim reclaim = {0};

	eth_gd32_tx_reclaim_completed(data, &reclaim);
	eth_gd32_tx_reclaim_flush(data, &reclaim);
}

static void eth_gd32_rx(const struct device *dev, bool rbu_seen)
{
	struct eth_gd32_data *data = dev->data;

	while (data->iface != NULL) {
		enet_descriptors_struct *start_desc = &eth0_dma.rx_desc[data->rx_tail];

		if (rx_desc_dma_owned(start_desc)) {
			break;
		}

		if ((start_desc->status & ENET_RDES0_FDES) == 0U) {
			eth_gd32_rx_buf_prepare_for_dma(eth0_rx_dma_buf[data->rx_tail],
							GD32_ETH_RX_BUF_SIZE);
			eth_gd32_rx_desc_give_to_dma(start_desc, eth0_rx_dma_buf[data->rx_tail]);

			data->rx_tail = modulo_inc(data->rx_tail, GD32_ETH_RX_DESC_COUNT);
			continue;
		}

		uint16_t idx = data->rx_tail;
		uint16_t cnt = 0U;
		bool complete = false;
		bool waiting_for_dma = false;

		while (cnt < GD32_ETH_RX_DESC_COUNT) {
			enet_descriptors_struct *desc = &eth0_dma.rx_desc[idx];

			if (rx_desc_dma_owned(desc)) {
				waiting_for_dma = true;
				break;
			}

			cnt++;
			if (desc->status & ENET_RDES0_LDES) {
				complete = true;
				break;
			}

			idx = modulo_inc(idx, GD32_ETH_RX_DESC_COUNT);
		}

		if (!complete) {
			if (waiting_for_dma) {
				break;
			}

			/* Corrupt/oversized frame: drop and rearm what we've scanned. */
			eth_stats_update_errors_rx(data->iface);

			idx = data->rx_tail;
			for (uint16_t n = 0U; n < cnt; n++) {
				enet_descriptors_struct *desc = &eth0_dma.rx_desc[idx];

				eth_gd32_rx_buf_prepare_for_dma(eth0_rx_dma_buf[idx],
								GD32_ETH_RX_BUF_SIZE);
				eth_gd32_rx_desc_give_to_dma(desc, eth0_rx_dma_buf[idx]);
				idx = modulo_inc(idx, GD32_ETH_RX_DESC_COUNT);
			}

			data->rx_tail = idx;
			continue;
		}

		enet_descriptors_struct *end_desc = &eth0_dma.rx_desc[idx];
		uint32_t frame_len = GET_RDES0_FRML(end_desc->status);

		bool rx_ok = (frame_len > 0U) && (frame_len <= GD32_ETH_FRAME_SIZE_MAX);
		rx_ok = rx_ok && ((end_desc->status & ENET_RDES0_ERRS) == 0U);

		struct net_pkt *pkt = NULL;

		if (rx_ok) {
			pkt = net_pkt_rx_alloc_with_buffer(data->iface, frame_len, NET_AF_UNSPEC, 0,
							   K_NO_WAIT);
			if (!pkt) {
				rx_ok = false;
#if defined(CONFIG_ETH_GD32_DEBUG_COUNTERS)
				atomic_inc(&data->dbg_alloc_fail);
#endif
			}
		}

		if (!rx_ok) {
			if (pkt) {
				net_pkt_unref(pkt);
			}

			eth_stats_update_errors_rx(data->iface);

			idx = data->rx_tail;
			for (uint16_t n = 0U; n < cnt; n++) {
				enet_descriptors_struct *desc = &eth0_dma.rx_desc[idx];

				eth_gd32_rx_buf_prepare_for_dma(eth0_rx_dma_buf[idx],
								GD32_ETH_RX_BUF_SIZE);
				eth_gd32_rx_desc_give_to_dma(desc, eth0_rx_dma_buf[idx]);
				idx = modulo_inc(idx, GD32_ETH_RX_DESC_COUNT);
			}

			data->rx_tail = idx;
			continue;
		}

		uint32_t copied = 0U;
		idx = data->rx_tail;

		for (uint16_t n = 0U; n < cnt; n++) {
			enet_descriptors_struct *desc = &eth0_dma.rx_desc[idx];
			uint32_t frag_len = MIN((uint32_t)GD32_ETH_RX_BUF_SIZE, frame_len - copied);
			void *rx_buf = eth0_rx_dma_buf[idx];

			gd32_cache_invalidate(rx_buf, frag_len);
			if (net_pkt_write(pkt, rx_buf, frag_len) != 0) {
				rx_ok = false;
			}

			copied += frag_len;
			eth_gd32_rx_buf_prepare_for_dma(rx_buf, GD32_ETH_RX_BUF_SIZE);
			eth_gd32_rx_desc_give_to_dma(desc, rx_buf);
			idx = modulo_inc(idx, GD32_ETH_RX_DESC_COUNT);
		}

		data->rx_tail = idx;

		if (!rx_ok || copied != frame_len || net_recv_data(data->iface, pkt) < 0) {
			eth_stats_update_errors_rx(data->iface);
			net_pkt_unref(pkt);
		}
#if defined(CONFIG_ETH_GD32_DEBUG_COUNTERS)
		else {
			atomic_inc(&data->dbg_delivered);
		}
#endif
	}

	if (rbu_seen) {
		eth_gd32_dma_rx_poll_demand();
	}
}

static void eth_gd32_rx_work_handler(struct k_work *work)
{
	struct eth_gd32_data *data = CONTAINER_OF(work, struct eth_gd32_data, rx_work);

#if defined(CONFIG_ETH_GD32_DEBUG_COUNTERS)
	atomic_inc(&data->dbg_rxwork);
#endif

	for (;;) {
		atomic_val_t flags = atomic_clear(&data->rx_work_flags);

		eth_gd32_rx(data->dev, (flags & GD32_RX_WORK_FLAG_RBU) != 0U);

		if (atomic_get(&data->rx_work_flags) == 0) {
			break;
		}
	}
}

static void eth_gd32_recover_work_handler(struct k_work *work)
{
	struct eth_gd32_data *data = CONTAINER_OF(work, struct eth_gd32_data, recover_work);

	LOG_ERR("Recovering ENET after fatal bus error");

	ENET_DMA_INTEN = 0U;
	enet_disable();

	k_mutex_lock(&data->tx_mutex, K_FOREVER);
	eth_gd32_tx_reset(data);
	data->rx_tail = 0U;

	if (eth_gd32_hw_reset_and_configure(data) != 0) {
		LOG_ERR("ENET re-init failed");
		goto out;
	}

	if (atomic_get(&data->link_up) != 0) {
		enet_enable();
		eth_gd32_dma_rx_poll_demand();
	} else {
		enet_disable();
	}

out:
	k_mutex_unlock(&data->tx_mutex);
	atomic_clear(&data->recovering);
}

static void eth_gd32_isr(const struct device *dev)
{
	struct eth_gd32_data *data = dev->data;
	uint32_t stat = ENET_DMA_STAT;
	uint32_t clear = 0U;

#if defined(CONFIG_ETH_GD32_DEBUG_COUNTERS)
	atomic_inc(&data->dbg_isr);
	if (stat & ENET_DMA_STAT_RS) {
		atomic_inc(&data->dbg_rs);
	}
	if (stat & ENET_DMA_STAT_RBU) {
		atomic_inc(&data->dbg_rbu);
	}
	if (stat & ENET_DMA_STAT_RO) {
		atomic_inc(&data->dbg_ro);
	}
#endif

	clear |= stat &
		 (ENET_DMA_STAT_TS | ENET_DMA_STAT_TBU | ENET_DMA_STAT_TJT | ENET_DMA_STAT_TU);
	clear |= stat &
		 (ENET_DMA_STAT_RS | ENET_DMA_STAT_RBU | ENET_DMA_STAT_RO | ENET_DMA_STAT_RPS);
	clear |= stat &
		 (ENET_DMA_STAT_TPS | ENET_DMA_STAT_RWT | ENET_DMA_STAT_ET | ENET_DMA_STAT_FBE);
	clear |= stat & (ENET_DMA_STAT_ER | ENET_DMA_STAT_AI | ENET_DMA_STAT_NI);

	if (clear) {
		ENET_DMA_STAT = clear;
	}

	if (stat & (ENET_DMA_STAT_RS | ENET_DMA_STAT_RBU)) {
		atomic_or(&data->rx_work_flags, GD32_RX_WORK_FLAG_KICK);

		if (stat & ENET_DMA_STAT_RBU) {
			atomic_or(&data->rx_work_flags, GD32_RX_WORK_FLAG_RBU);
		}

		(void)k_work_submit(&data->rx_work);
	}

	if (stat & ENET_DMA_STAT_TS) {
		eth_gd32_reclaim_tx(dev);
	}

	if (stat & (ENET_DMA_STAT_TBU | ENET_DMA_STAT_TU)) {
		eth_gd32_dma_tx_poll_demand();
	}

	if (stat & ENET_DMA_STAT_RO) {
		LOG_WRN("RX overflow");
	}

	if (stat & ENET_DMA_STAT_FBE) {
		LOG_ERR("Fatal bus error (DMA_STAT=0x%08x)", stat);
		if (atomic_cas(&data->recovering, 0, 1)) {
			k_work_submit(&data->recover_work);
		}
	}
}

static int eth_gd32_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct eth_gd32_data *data = dev->data;
	uint16_t frag_cnt = 0U;
	struct net_pkt *tx_pkt;
	k_spinlock_key_t key;
	uint16_t idx;
	uint16_t first_idx;
	uint16_t last_idx = 0U;
	uint16_t remaining;

	if (!data->hw_ready || (data->iface == NULL) || (atomic_get(&data->link_up) == 0)) {
		return -ENETDOWN;
	}

	__ASSERT_NO_MSG(pkt);
	__ASSERT_NO_MSG(pkt->frags);

	for (struct net_buf *frag = pkt->frags; frag; frag = frag->frags) {
		if (frag->len == 0U) {
			continue;
		}

		frag_cnt++;

		if (((uintptr_t)frag->data & 0x3U) != 0U ||
		    !gd32_dma_addr_ok((uintptr_t)frag->data, frag->len)) {
			LOG_ERR("TX buffer not DMA accessible");
			return -EFAULT;
		}

		gd32_cache_clean(frag->data, frag->len);
	}

	if (frag_cnt == 0U || frag_cnt > GD32_ETH_TX_DESC_COUNT) {
		if (frag_cnt > GD32_ETH_TX_DESC_COUNT) {
			LOG_ERR("Attempted to send more fragments than available descriptors. You have to increase GD32_ETH_TX_DESC_COUNT");
		}
		return -EINVAL;
	}

	k_mutex_lock(&data->tx_mutex, K_FOREVER);

	if (eth_gd32_tx_desc_reserve(dev, frag_cnt) != 0) {
		k_mutex_unlock(&data->tx_mutex);
		return -ENOBUFS;
	}

	tx_pkt = net_pkt_ref(pkt);

	key = k_spin_lock(&data->tx_ring_lock);
	first_idx = data->tx_next_to_use;
	idx = first_idx;
	remaining = frag_cnt;

	/*
	 * Publish the whole frame to the ring while TX completion is blocked by
	 * tx_ring_lock. The first descriptor is handed to DMA last.
	 */
	for (struct net_buf *frag = pkt->frags; frag; frag = frag->frags) {
		uint32_t status;
		enet_descriptors_struct *desc;

		if (frag->len == 0U) {
			continue;
		}

		desc = &eth0_dma.tx_desc[idx];
		status = ENET_TDES0_TCHM | ENET_CHECKSUM_DISABLE;

		if (idx == first_idx) {
			status |= ENET_TDES0_FSG;
		}
		if (remaining == 1U) {
			status |= ENET_TDES0_LSG | ENET_TDES0_INTC;
			last_idx = idx;
		}

		desc->control_buffer_size = (uint32_t)frag->len & ENET_TDES1_TB1S;
		desc->buffer1_addr = (uint32_t)(uintptr_t)frag->data;
		desc->status = status;
		data->tx_pkt_refs[idx] = NULL;
		idx = modulo_inc(idx, GD32_ETH_TX_DESC_COUNT);
		remaining--;
	}

	data->tx_pkt_refs[last_idx] = tx_pkt;
	data->tx_next_to_use = idx;
	data->tx_descs_in_use += frag_cnt;

	__ASSERT_NO_MSG(data->tx_descs_in_use <= GD32_ETH_TX_DESC_COUNT);

	barrier_dmem_fence_full();

	idx = modulo_inc(first_idx, GD32_ETH_TX_DESC_COUNT);
	for (uint16_t i = 1U; i < frag_cnt; i++) {
		eth0_dma.tx_desc[idx].status |= ENET_TDES0_DAV;
		idx = modulo_inc(idx, GD32_ETH_TX_DESC_COUNT);
	}

	barrier_dmem_fence_full();
	eth0_dma.tx_desc[first_idx].status |= ENET_TDES0_DAV;
	k_spin_unlock(&data->tx_ring_lock, key);

	if (ENET_DMA_STAT & (ENET_DMA_STAT_TBU | ENET_DMA_STAT_TU)) {
		ENET_DMA_STAT = ENET_DMA_STAT_TBU | ENET_DMA_STAT_TU;
	}
	eth_gd32_dma_tx_poll_demand();

	k_mutex_unlock(&data->tx_mutex);
	return 0;
}

static enum ethernet_hw_caps eth_gd32_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE
#if !defined(CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST)
	       | ETHERNET_HW_FILTERING
#endif
#if defined(CONFIG_NET_LLDP)
	       | ETHERNET_LLDP
#endif
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	       | ETHERNET_PROMISC_MODE
#endif
		;
}

static void eth_gd32_set_mac_config(const struct device *dev, struct phy_link_state *state)
{
	ARG_UNUSED(dev);

	uint32_t reg = ENET_MAC_CFG;
	reg &= ~(ENET_MAC_CFG_SPD | ENET_MAC_CFG_DPM);
	if (PHY_LINK_IS_FULL_DUPLEX(state->speed)) {
		reg |= ENET_MAC_CFG_DPM;
	}
	if (PHY_LINK_IS_SPEED_100M(state->speed)) {
		reg |= ENET_MAC_CFG_SPD;
	}
	ENET_MAC_CFG = reg;
}

static void phy_link_state_changed(const struct device *phy_dev, struct phy_link_state *state,
				   void *user_data)
{
	const struct device *dev = user_data;
	struct eth_gd32_data *data = dev->data;
	ARG_UNUSED(phy_dev);

	atomic_set(&data->link_up, state->is_up ? 1 : 0);

	if (!data->hw_ready) {
		return;
	}

	enet_disable();
	if (state->is_up) {
		eth_gd32_set_mac_config(dev, state);
		enet_enable();
		eth_gd32_dma_rx_poll_demand();
		if (data->iface) {
			net_eth_carrier_on(data->iface);
		}
	} else {
		if (data->iface) {
			net_eth_carrier_off(data->iface);
		}
	}

	if (state->is_up) {
		const char *speed = PHY_LINK_IS_SPEED_100M(state->speed) ? "100M" : "10M";
		const char *duplex = PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half";

		LOG_INF("Link up (%s %s-duplex)", speed, duplex);
	} else {
		LOG_INF("Link down");
	}
}

#if !defined(CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST)
static uint32_t eth_gd32_mcast_hash_index_get(const struct net_eth_addr *addr)
{
	uint32_t crc = __RBIT(crc32_ieee(addr->addr, sizeof(addr->addr)));

	return (crc >> 26) & 0x3FU;
}

static void eth_gd32_mcast_hash_sync(const struct eth_gd32_data *data)
{
	uint32_t hash_table[2] = {0U};

	for (uint32_t i = 0U; i < ARRAY_SIZE(data->mcast_hash_refcnt); i++) {
		if (data->mcast_hash_refcnt[i] == 0U) {
			continue;
		}

		hash_table[i / 32U] |= BIT(i % 32U);
	}

	ENET_MAC_HLL = hash_table[0];
	ENET_MAC_HLH = hash_table[1];
}
#endif /* !CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST */

static int eth_gd32_set_config(const struct device *dev, enum ethernet_config_type type,
			       const struct ethernet_config *config)
{
	struct eth_gd32_data *data = dev->data;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(data->mac_addr, config->mac_address.addr, sizeof(data->mac_addr));
		enet_mac_address_set(ENET_MAC_ADDRESS0, data->mac_addr);
		if (data->iface) {
			net_if_set_link_addr(data->iface, data->mac_addr, sizeof(data->mac_addr),
					     NET_LINK_ETHERNET);
		}
		return 0;
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE: {
		uint32_t frmf = ENET_MAC_FRMF;
		if (config->promisc_mode) {
			frmf |= ENET_MAC_FRMF_PM;
		} else {
			frmf &= ~ENET_MAC_FRMF_PM;
		}
		ENET_MAC_FRMF = frmf;
		return 0;
	}
#endif
	case ETHERNET_CONFIG_TYPE_FILTER: {
		const struct ethernet_filter *filter = &config->filter;
		struct net_eth_addr addr = filter->mac_address;

		if (filter->type != ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS) {
			return -ENOTSUP;
		}

		if (!net_eth_is_addr_multicast(&addr) || net_eth_is_addr_broadcast(&addr)) {
			return -EINVAL;
		}

#if defined(CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST)
		return -ENOTSUP;
#else
		uint32_t hash_index = eth_gd32_mcast_hash_index_get(&addr);
		uint8_t *refcnt = &data->mcast_hash_refcnt[hash_index];

		if (filter->set) {
			if (*refcnt != UINT8_MAX) {
				(*refcnt)++;
			}
		} else {
			if (*refcnt == 0U) {
				return -ENOENT;
			}

			(*refcnt)--;
		}

		eth_gd32_mcast_hash_sync(data);
		return 0;
#endif /* CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST */
	}
	default:
		break;
	}

	return -ENOTSUP;
}

static const struct device *eth_gd32_get_phy(const struct device *dev)
{
	ARG_UNUSED(dev);
#if DT_INST_NODE_HAS_PROP(0, phy_handle)
	return DEVICE_DT_GET(DT_INST_PHANDLE(0, phy_handle));
#else
	return NULL;
#endif
}

static bool eth_gd32_mac_is_valid(const uint8_t mac[NET_ETH_ADDR_LEN])
{
	struct net_eth_addr addr;

	memcpy(addr.addr, mac, sizeof(addr.addr));
	return net_eth_is_addr_valid(&addr);
}

static bool eth_gd32_dt_mac_prefix_get(uint8_t oui[3])
{
#if DT_INST_NODE_HAS_PROP(0, zephyr_mac_address_prefix)
	const uint8_t prefix[3] = DT_INST_PROP(0, zephyr_mac_address_prefix);

	if ((prefix[0] & 0x01) != 0U) {
		return false;
	}

	memcpy(oui, prefix, 3);
	return true;
#else
	ARG_UNUSED(oui);
	return false;
#endif
}

static void eth_gd32_mac_random(uint8_t mac[NET_ETH_ADDR_LEN], const uint8_t *prefix,
			       size_t prefix_len)
{
	__ASSERT_NO_MSG(prefix_len <= NET_ETH_ADDR_LEN);

	if (prefix_len > 0U) {
		memcpy(mac, prefix, prefix_len);
	}
	sys_rand_get(&mac[prefix_len], NET_ETH_ADDR_LEN - prefix_len);

	/* Locally administered, unicast */
	mac[0] |= 0x02;
	mac[0] &= ~0x01;
}

static int eth_gd32_mac_from_dt(uint8_t mac[NET_ETH_ADDR_LEN])
{
#if NODE_HAS_VALID_MAC_ADDR(DT_DRV_INST(0))
	const uint8_t addr[NET_ETH_ADDR_LEN] = DT_INST_PROP(0, local_mac_address);
	memcpy(mac, addr, sizeof(addr));
	return eth_gd32_mac_is_valid(mac) ? 0 : -EINVAL;
#else
	ARG_UNUSED(mac);
	return -ENODATA;
#endif
}

static int eth_gd32_mac_from_nvmem(uint8_t mac[NET_ETH_ADDR_LEN])
{
#if DT_INST_NVMEM_CELLS_HAS_NAME(0, mac_address)
	const struct nvmem_cell cell = NVMEM_CELL_INST_GET_BY_NAME(0, mac_address);
	uint8_t oui[3] = { GD32_OUI_B0, GD32_OUI_B1, GD32_OUI_B2 };
	int ret;

	if (!device_is_ready(cell.dev)) {
		return -ENODEV;
	}

	if (cell.size == NET_ETH_ADDR_LEN) {
		ret = flash_read(cell.dev, cell.offset, mac, cell.size);
		if (ret < 0) {
			return ret;
		}
		return eth_gd32_mac_is_valid(mac) ? 0 : -EINVAL;
	}

	if (cell.size == 3U && eth_gd32_dt_mac_prefix_get(oui)) {
		mac[0] = oui[0];
		mac[1] = oui[1];
		mac[2] = oui[2];

		ret = flash_read(cell.dev, cell.offset, &mac[3], cell.size);
		if (ret < 0) {
			return ret;
		}

		return eth_gd32_mac_is_valid(mac) ? 0 : -EINVAL;
	}

	return -EINVAL;
#else
	ARG_UNUSED(mac);
	return -ENODATA;
#endif
}

static int eth_gd32_mac_from_hwinfo(const uint8_t oui[3], uint8_t mac[NET_ETH_ADDR_LEN])
{
	uint8_t unique_device_id[16];
	ssize_t len;
	uint32_t hash;

	len = hwinfo_get_device_id(unique_device_id, sizeof(unique_device_id));
	if (len < 0) {
		return (int)len;
	}
	if (len == 0) {
		return -ENODATA;
	}

	hash = crc32_ieee(unique_device_id, (size_t)len);

	mac[0] = oui[0];
	mac[1] = oui[1];
	mac[2] = oui[2];
	memcpy(&mac[3], &hash, 3);

	return eth_gd32_mac_is_valid(mac) ? 0 : -EINVAL;
}

static void eth_gd32_mac_init(const struct device *dev)
{
	struct eth_gd32_data *data = dev->data;
	uint8_t user_oui[3];
	uint8_t oui[3] = { GD32_OUI_B0, GD32_OUI_B1, GD32_OUI_B2 };
	const char *src = "unknown";
	int ret;

	if (DT_INST_PROP(0, zephyr_random_mac_address)) {
		if (eth_gd32_dt_mac_prefix_get(user_oui)) {
			eth_gd32_mac_random(data->mac_addr, user_oui, sizeof(user_oui));
		} else {
			eth_gd32_mac_random(data->mac_addr, NULL, 0U);
		}

		src = "random";
		goto out;
	}

	ret = eth_gd32_mac_from_dt(data->mac_addr);
	if (ret == 0) {
		src = "dt";
		goto out;
	}

	ret = eth_gd32_mac_from_nvmem(data->mac_addr);
	if (ret == 0) {
		src = "nvmem";
		goto out;
	}

	if (eth_gd32_dt_mac_prefix_get(user_oui)) {
		memcpy(oui, user_oui, sizeof(oui));
		ret = eth_gd32_mac_from_hwinfo(oui, data->mac_addr);
		if (ret == 0) {
			src = "hwinfo+dt-oui";
			goto out;
		}
	}

	ret = eth_gd32_mac_from_hwinfo(oui, data->mac_addr);
	if (ret == 0) {
		src = "hwinfo+gd-oui";
		goto out;
	}

	eth_gd32_mac_random(data->mac_addr, NULL, 0U);
	src = "random-fallback";

out:
	LOG_INF("MAC %02x:%02x:%02x:%02x:%02x:%02x (%s)",
		data->mac_addr[0], data->mac_addr[1], data->mac_addr[2],
		data->mac_addr[3], data->mac_addr[4], data->mac_addr[5], src);
}

static void eth_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_gd32_data *data = dev->data;
	const struct device *phy = eth_gd32_get_phy(dev);

	if (data->iface == NULL) {
		data->iface = iface;
	}

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr), NET_LINK_ETHERNET);
	ethernet_init(iface);
	net_eth_carrier_off(iface);
	net_lldp_set_lldpdu(iface);

	if (phy == NULL) {
		LOG_WRN("No PHY device, assuming link up");
		atomic_set(&data->link_up, 1);
		enet_enable();
		eth_gd32_dma_rx_poll_demand();
		net_eth_carrier_on(iface);
		return;
	}

	if (!device_is_ready(phy)) {
		LOG_WRN("PHY device not ready");
		return;
	}

	phy_link_callback_set(phy, phy_link_state_changed, (void *)dev);

	struct phy_link_state st = {0};
	if (phy_get_link_state(phy, &st) == 0) {
		phy_link_state_changed(phy, &st, (void *)dev);
	}
}

static const struct ethernet_api eth_api = {
	.iface_api.init = eth_iface_init,
	.get_capabilities = eth_gd32_get_capabilities,
	.set_config = eth_gd32_set_config,
	.get_phy = eth_gd32_get_phy,
	.send = eth_gd32_tx,
};

static void eth_gd32_mac_dma_default_config(void)
{
	/*
	 * This driver does a "clean init": it programs MAC/DMA registers
	 * explicitly and lets the Zephyr PHY layer negotiate the link.
	 */

	/* -------------------------- MAC configuration -------------------------- */
	uint32_t mac_cfg = ENET_MAC_CFG;

	/*
	 * Start from a known state: clear bits for features we leave disabled
	 * and set bits we want enabled. (The vendor's *_ENABLE macros often
	 * evaluate to 0, which hides what the code actually does.)
	 */
	mac_cfg &= ~(ENET_MAC_CFG_TEN | ENET_MAC_CFG_REN | ENET_MAC_CFG_LBM | ENET_MAC_CFG_IPFCO |
		     ENET_MAC_CFG_APCD | ENET_MAC_CFG_TFCD | ENET_MAC_CFG_DFC | ENET_MAC_CFG_ROD |
		     ENET_MAC_CFG_RTD | ENET_MAC_CFG_CSD | ENET_MAC_CFG_WDD | ENET_MAC_CFG_JBD |
		     ENET_MAC_CFG_IGBS | ENET_MAC_CFG_BOL | ENET_MAC_CFG_SPD | ENET_MAC_CFG_DPM);

	/* Default to 100M full-duplex; the PHY callback updates speed/duplex. */
	mac_cfg |= ENET_SPEEDMODE_100M | ENET_MODE_FULLDUPLEX;

	/* Standard Ethernet IFG, and 16 retries (BOL=10 => min(n,10)). */
	mac_cfg |= ENET_INTERFRAMEGAP_96BIT | ENET_BACKOFFLIMIT_10;

	/*
	 * Keep checksum offload disabled: enabling IP checksum checking in the
	 * receiver can drop valid frames on this SoC (see errata in project docs).
	 */
	/*
	 * Always strip padding/FCS in hardware so the network stack receives
	 * Ethernet frames without the trailing 4-byte FCS (as other Zephyr
	 * Ethernet drivers do).
	 */
	mac_cfg |= ENET_MAC_CFG_APCD | ENET_MAC_CFG_TFCD;
	ENET_MAC_CFG = mac_cfg;

	/*
	 * Packet filtering:
	 * - Always accept broadcast.
	 * - Either accept all multicast frames (debug/compat) or use multicast hash filtering.
	 */
	uint32_t frmf = ENET_MAC_FRMF;
	frmf &= ~(ENET_MAC_FRMF_FAR | ENET_MAC_FRMF_PM | ENET_MAC_FRMF_BFRMD | ENET_MAC_FRMF_PCFRM |
		  ENET_MAC_FRMF_HMF | ENET_MAC_FRMF_HPFLT | ENET_MAC_FRMF_HUF | ENET_MAC_FRMF_MFD);
	frmf |= ENET_PCFRM_PREVENT_ALL;
	if (IS_ENABLED(CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST)) {
		frmf |= ENET_MULTICAST_FILTER_PASS;
	} else {
		frmf |= ENET_MULTICAST_FILTER_HASH_OR_PERFECT;
	}
	ENET_MAC_FRMF = frmf;

	/* Multicast hash table (64 bits). */
	ENET_MAC_HLH = 0U;
	ENET_MAC_HLL = 0U;

	/* Flow control disabled. */
	uint32_t fctl = ENET_MAC_FCTL;
	fctl &= ~(ENET_MAC_FCTL_TFCEN | ENET_MAC_FCTL_RFCEN | ENET_MAC_FCTL_PTM | ENET_MAC_FCTL_PLTS |
		  ENET_MAC_FCTL_DZQP);
	ENET_MAC_FCTL = fctl;

	/* VLAN tag filtering disabled. */
	ENET_MAC_VLT = 0U;

	/* -------------------------- DMA configuration -------------------------- */
	uint32_t dma_ctl = ENET_DMA_CTL;

	/*
	 * Store-and-forward generally maximizes correctness. Cut-through can
	 * reduce latency, but can forward a frame to memory before all errors are
	 * known (see the reference manual).
	 */
	dma_ctl &= ~(ENET_DMA_CTL_RSFD | ENET_DMA_CTL_TSFD | ENET_DMA_CTL_OSF | ENET_DMA_CTL_DAFRF |
		     ENET_DMA_CTL_DTCERFD | ENET_DMA_CTL_RTHC | ENET_DMA_CTL_TTHC);

	if (IS_ENABLED(CONFIG_ETH_GD32_RX_STORE_FORWARD)) {
		dma_ctl |= ENET_DMA_CTL_RSFD;
	} else if (IS_ENABLED(CONFIG_ETH_GD32_RX_THRESHOLD_32)) {
		dma_ctl |= ENET_RX_THRESHOLD_32BYTES;
	} else if (IS_ENABLED(CONFIG_ETH_GD32_RX_THRESHOLD_96)) {
		dma_ctl |= ENET_RX_THRESHOLD_96BYTES;
	} else if (IS_ENABLED(CONFIG_ETH_GD32_RX_THRESHOLD_128)) {
		dma_ctl |= ENET_RX_THRESHOLD_128BYTES;
	} else {
		dma_ctl |= ENET_RX_THRESHOLD_64BYTES;
	}

	if (IS_ENABLED(CONFIG_ETH_GD32_TX_STORE_FORWARD)) {
		dma_ctl |= ENET_DMA_CTL_TSFD;
	} else if (IS_ENABLED(CONFIG_ETH_GD32_TX_THRESHOLD_128)) {
		dma_ctl |= ENET_TX_THRESHOLD_128BYTES;
	} else if (IS_ENABLED(CONFIG_ETH_GD32_TX_THRESHOLD_192)) {
		dma_ctl |= ENET_TX_THRESHOLD_192BYTES;
	} else if (IS_ENABLED(CONFIG_ETH_GD32_TX_THRESHOLD_256)) {
		dma_ctl |= ENET_TX_THRESHOLD_256BYTES;
	} else if (IS_ENABLED(CONFIG_ETH_GD32_TX_THRESHOLD_40)) {
		dma_ctl |= ENET_TX_THRESHOLD_40BYTES;
	} else if (IS_ENABLED(CONFIG_ETH_GD32_TX_THRESHOLD_32)) {
		dma_ctl |= ENET_TX_THRESHOLD_32BYTES;
	} else if (IS_ENABLED(CONFIG_ETH_GD32_TX_THRESHOLD_24)) {
		dma_ctl |= ENET_TX_THRESHOLD_24BYTES;
	} else if (IS_ENABLED(CONFIG_ETH_GD32_TX_THRESHOLD_16)) {
		dma_ctl |= ENET_TX_THRESHOLD_16BYTES;
	} else {
		dma_ctl |= ENET_TX_THRESHOLD_64BYTES;
	}

	if (IS_ENABLED(CONFIG_ETH_GD32_TX_OSF)) {
		dma_ctl |= ENET_DMA_CTL_OSF;
	}
	ENET_DMA_CTL = dma_ctl;

	uint32_t dma_bctl = 0U;
	dma_bctl |= ENET_ADDRESS_ALIGN_ENABLE;
	dma_bctl |= ENET_ARBITRATION_RXTX_2_1;
	dma_bctl |= ENET_RXDP_32BEAT | ENET_PGBL_32BEAT | ENET_RXTX_DIFFERENT_PGBL;
	dma_bctl |= ENET_FIXED_BURST_ENABLE | ENET_MIXED_BURST_DISABLE;
	dma_bctl |= DMA_BCTL_DPSL(0U);

#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
	dma_bctl |= ENET_ENHANCED_DESCRIPTOR;
#else
	dma_bctl |= ENET_NORMAL_DESCRIPTOR;
#endif

	ENET_DMA_BCTL = dma_bctl;
}

static int eth_gd32_hw_init(const struct device *dev)
{
	struct eth_gd32_data *data = dev->data;
	const struct eth_gd32_config *cfg = dev->config;
	int ret;

	data->dev = dev;
	k_mutex_init(&data->tx_mutex);
	k_sem_init(&data->tx_desc_sem, GD32_ETH_TX_DESC_COUNT, GD32_ETH_TX_DESC_COUNT);
	k_work_init(&data->rx_work, eth_gd32_rx_work_handler);
	(void)atomic_clear(&data->rx_work_flags);
	k_work_init(&data->recover_work, eth_gd32_recover_work_handler);
	(void)atomic_clear(&data->recovering);
	(void)atomic_clear(&data->link_up);

	/*
	 * Order matters: the RMII/MII interface selection and the RMII reference
	 * clock must both be in place BEFORE the ENET MAC clocks are enabled. The
	 * MAC latches the SYSCFG PHY-interface selection as it comes out of reset;
	 * selecting RMII after the MAC clock is already running leaves the datapath
	 * in MII mode even though SYSCFG_CFG1 reads back RMII, which corrupts every
	 * received frame (100% CRC/alignment errors) while TX still appears to work.
	 */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	uint16_t clk_syscfg = GD32_CLOCK_SYSCFG;
	(void)clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&clk_syscfg);

	if (cfg->phy_mode == 0) {
		syscfg_enet_phy_interface_config(SYSCFG_ENET_PHY_RMII);
	} else {
		syscfg_enet_phy_interface_config(SYSCFG_ENET_PHY_MII);
	}

#if DT_INST_PROP(0, gd_phy_clk_out)
	{
		uint16_t clk_gpioa = GD32_CLOCK_GPIOA;
		(void)clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&clk_gpioa);
		gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
		gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_8);
		gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_8);
		rcu_ckout0_config(RCU_CKOUT0SRC_PLLP, RCU_CKOUT0_DIV4);
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, ethernet_reset_gpios)
	{
		/*
		 * Give the PHY a clean hardware reset with the RMII reference clock
		 * already running, so it latches its strap configuration correctly. The
		 * gdzone wires PHY nRST to GPIOG; the pin/polarity come from devicetree.
		 */
		uint16_t clk_gpiog = GD32_CLOCK_GPIOG;
		uint32_t rst_pin = BIT(DT_INST_GPIO_PIN(0, ethernet_reset_gpios));
		bool active_low =
			(DT_INST_GPIO_FLAGS(0, ethernet_reset_gpios) & GPIO_ACTIVE_LOW) != 0U;

		(void)clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&clk_gpiog);
		gpio_mode_set(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, rst_pin);
		gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, rst_pin);

		gpio_bit_write(GPIOG, rst_pin, active_low ? RESET : SET);
		k_msleep(10);
		gpio_bit_write(GPIOG, rst_pin, active_low ? SET : RESET);
		k_msleep(50);
	}
#endif

	/* Bring up the MAC/TX/RX clocks now; this latches the RMII selection. */
	ret = clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clk_mac);
	ret |= clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clk_tx);
	ret |= clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clk_rx);
	if (cfg->has_ptp) {
		ret |= clock_control_on(GD32_CLOCK_CONTROLLER,
					(clock_control_subsys_t)&cfg->clk_ptp);
	}
	if (ret) {
		return -EIO;
	}

	/* Toggle optional hardware reset lines */
#if DT_INST_PROP_LEN(0, resets) > 0
	if (cfg->rst0.dev) {
		(void)reset_line_toggle_dt(&cfg->rst0);
	}
#endif
#if DT_INST_PROP_LEN(0, resets) > 1
	if (cfg->rst1.dev) {
		(void)reset_line_toggle_dt(&cfg->rst1);
	}
#endif
#if DT_INST_PROP_LEN(0, resets) > 2
	if (cfg->rst2.dev) {
		(void)reset_line_toggle_dt(&cfg->rst2);
	}
#endif
#if DT_INST_PROP_LEN(0, resets) > 3
	if (cfg->rst3.dev) {
		(void)reset_line_toggle_dt(&cfg->rst3);
	}
#endif

	if (cfg->config_func) {
		cfg->config_func();
	}

	for (uint16_t i = 0U; i < GD32_ETH_RX_DESC_COUNT; i++) {
		if (((uintptr_t)eth0_rx_dma_buf[i] & 0x3U) != 0U ||
		    !gd32_dma_addr_ok((uintptr_t)eth0_rx_dma_buf[i], GD32_ETH_RX_BUF_SIZE)) {
			return -EFAULT;
		}
	}

	if (eth_gd32_hw_reset_and_configure(data) != 0) {
		return -EIO;
	}

	/* Start disabled; link-up callback (PHY) enables TX/RX. */
	enet_disable();
	eth_gd32_dma_rx_poll_demand();

	data->hw_ready = true;
	LOG_INF("ENET initialized (rx-desc=%u, tx-desc=%u)", GD32_ETH_RX_DESC_COUNT,
		GD32_ETH_TX_DESC_COUNT);

	return 0;
}

static int eth_gd32_init(const struct device *dev)
{
	eth_gd32_mac_init(dev);
	return eth_gd32_hw_init(dev);
}

static void eth0_irq_config(void)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), eth_gd32_isr, DEVICE_DT_INST_GET(0),
		    0);
	irq_enable(DT_INST_IRQN(0));
}

PINCTRL_DT_INST_DEFINE(0);

static const struct eth_gd32_config eth0_config = {
	.clk_mac = DT_INST_CLOCKS_CELL_BY_NAME(0, mac, id),
	.clk_tx = DT_INST_CLOCKS_CELL_BY_NAME(0, tx, id),
	.clk_rx = DT_INST_CLOCKS_CELL_BY_NAME(0, rx, id),
#if DT_INST_CLOCKS_HAS_NAME(0, ptp)
	.has_ptp = true,
	.clk_ptp = DT_INST_CLOCKS_CELL_BY_NAME(0, ptp, id),
#else
	.has_ptp = false,
#endif
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.phy_mode = DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, mii) ? 1 : 0,
	.config_func = eth0_irq_config,
#if DT_INST_PROP_LEN(0, resets) > 0
	.rst0 = RESET_DT_SPEC_INST_GET_BY_IDX(0, 0),
#endif
#if DT_INST_PROP_LEN(0, resets) > 1
	.rst1 = RESET_DT_SPEC_INST_GET_BY_IDX(0, 1),
#endif
#if DT_INST_PROP_LEN(0, resets) > 2
	.rst2 = RESET_DT_SPEC_INST_GET_BY_IDX(0, 2),
#endif
#if DT_INST_PROP_LEN(0, resets) > 3
	.rst3 = RESET_DT_SPEC_INST_GET_BY_IDX(0, 3),
#endif
};

static struct eth_gd32_data eth0_data;

#if defined(CONFIG_ETH_GD32_DEBUG_COUNTERS)
static void eth_gd32_dbg_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	struct eth_gd32_data *data = &eth0_data;

	for (;;) {
		k_sleep(K_MSEC(1000));

		uint16_t dma_owned = 0U;

		for (uint16_t i = 0U; i < GD32_ETH_RX_DESC_COUNT; i++) {
			if (rx_desc_dma_owned(&eth0_dma.rx_desc[i])) {
				dma_owned++;
			}
		}

		uint32_t stat = ENET_DMA_STAT;
		uint32_t rps = (stat >> 17) & 0x7U;
		uint32_t base = (uint32_t)(uintptr_t)eth0_dma.rx_desc;
		int hw_idx = (int)(((uint32_t)ENET_DMA_CRDADDR - base) /
				   sizeof(enet_descriptors_struct));

		LOG_INF("rx: isr=%ld rs=%ld rbu=%ld ro=%ld work=%ld deliv=%ld allocfail=%ld | "
			"dma_owned=%u/%u rx_tail=%u hw_idx=%d rps=%u stat=0x%08x link=%ld recov=%ld",
			atomic_get(&data->dbg_isr), atomic_get(&data->dbg_rs),
			atomic_get(&data->dbg_rbu), atomic_get(&data->dbg_ro),
			atomic_get(&data->dbg_rxwork), atomic_get(&data->dbg_delivered),
			atomic_get(&data->dbg_alloc_fail), dma_owned,
			(unsigned int)GD32_ETH_RX_DESC_COUNT, data->rx_tail, hw_idx, rps, stat,
			atomic_get(&data->link_up), atomic_get(&data->recovering));
	}
}

K_THREAD_DEFINE(eth_gd32_dbg, 1024, eth_gd32_dbg_thread, NULL, NULL, NULL, 7, 0, 0);
#endif /* CONFIG_ETH_GD32_DEBUG_COUNTERS */

ETH_NET_DEVICE_DT_INST_DEFINE(0, eth_gd32_init, NULL, &eth0_data, &eth0_config,
			      CONFIG_ETH_INIT_PRIORITY, &eth_api, NET_ETH_MTU);
