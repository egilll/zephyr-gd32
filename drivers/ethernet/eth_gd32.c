/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32 ENET uses:
 * - private DMA-owned RX buffers, copied into Zephyr packets before frames
 *   are handed to the stack, and
 * - zero-copy TX when the packet fragments are DMA-safe, with a per-instance
 *   DMA bounce fallback for fragmented or non-DMA-safe packets.
 */

#include <errno.h>

#define DT_DRV_COMPAT gd_gd32_ethernet

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/mem_mgmt/mem_attr.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/lldp.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/phy.h>
#include <zephyr/nvmem.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <ethernet/eth_stats.h>

#include "eth.h"
#include "gd32_enet_platform.h"

#if defined(CONFIG_SOC_SERIES_GD32F4XX)
#include <gd32f4xx_enet.h>
#elif defined(CONFIG_SOC_SERIES_GD32E50X)
#include <gd32e50x_enet.h>
#else
#error "Unsupported GD32 SoC series for ENET"
#endif

LOG_MODULE_REGISTER(eth_gd32, CONFIG_ETHERNET_LOG_LEVEL);

#define GD32_ETH_TX_TIMEOUT_MS       100U
#define GD32_ETH_FRAME_SIZE_MAX      (NET_ETH_MTU + 18U)
#define GD32_ETH_DMA_POLL_DEMAND     1U
#define GD32_ETH_RESET_TIMEOUT_US    100000U
#define GD32_ETH_TX_FLUSH_TIMEOUT_US 100000U
#define GD32_ETH_STOP_TIMEOUT_US     100000U

#define GD32_OUI_B0 0xE0
#define GD32_OUI_B1 0x05
#define GD32_OUI_B2 0x1C

#ifdef CONFIG_DCACHE_LINE_SIZE
#define GD32_ETH_DMA_ALIGN MAX(CONFIG_DCACHE_LINE_SIZE, 4)
#else
#define GD32_ETH_DMA_ALIGN 4
#endif

#define GD32_ETH_RX_DESC_COUNT CONFIG_ETH_GD32_RX_DESC_COUNT
#define GD32_ETH_TX_DESC_COUNT CONFIG_ETH_GD32_TX_DESC_COUNT
#define GD32_ETH_RX_DMA_BUFS_PER_DESC 1U
#define GD32_ETH_TX_DMA_BUFS_PER_DESC 2U
#if defined(CONFIG_NET_BUF_FIXED_DATA_SIZE)
#define GD32_ETH_DMA_BUF_SIZE ROUND_UP(ROUND_DOWN(CONFIG_NET_BUF_DATA_SIZE, 4), GD32_ETH_DMA_ALIGN)
#else
#define GD32_ETH_DMA_BUF_SIZE ROUND_UP(128U, GD32_ETH_DMA_ALIGN)
#endif
#define GD32_ETH_RX_DESC_BUF_LEN (GD32_ETH_RX_DMA_BUFS_PER_DESC * GD32_ETH_DMA_BUF_SIZE)
#define GD32_ETH_TX_DESC_BUF_LEN (GD32_ETH_TX_DMA_BUFS_PER_DESC * GD32_ETH_DMA_BUF_SIZE)
#define GD32_ETH_DMA_DESC_SIZE ROUND_UP(sizeof(enet_descriptors_struct), GD32_ETH_DMA_ALIGN)

#define GD32_ENET_MAC_CFG_OFFSET      0x0000U
#define GD32_ENET_MAC_FRMF_OFFSET     0x0004U
#define GD32_ENET_MAC_HLH_OFFSET      0x0008U
#define GD32_ENET_MAC_HLL_OFFSET      0x000CU
#define GD32_ENET_MAC_PHY_CTL_OFFSET  0x0010U
#define GD32_ENET_MAC_PHY_DATA_OFFSET 0x0014U
#define GD32_ENET_MAC_FCTL_OFFSET     0x0018U
#define GD32_ENET_MAC_VLT_OFFSET      0x001CU
#define GD32_ENET_MAC_ADDR0H_OFFSET   0x0040U
#define GD32_ENET_MAC_ADDR0L_OFFSET   0x0044U
#define GD32_ENET_DMA_BCTL_OFFSET     0x1000U
#define GD32_ENET_DMA_TPEN_OFFSET     0x1004U
#define GD32_ENET_DMA_RPEN_OFFSET     0x1008U
#define GD32_ENET_DMA_RDTADDR_OFFSET  0x100CU
#define GD32_ENET_DMA_TDTADDR_OFFSET  0x1010U
#define GD32_ENET_DMA_STAT_OFFSET     0x1014U
#define GD32_ENET_DMA_CTL_OFFSET      0x1018U
#define GD32_ENET_DMA_INTEN_OFFSET    0x101CU
#define GD32_ENET_DMA_CRDADDR_OFFSET  0x104CU
#define GD32_ENET_DMA_CRBADDR_OFFSET  0x1054U

BUILD_ASSERT(GD32_ETH_DMA_BUF_SIZE >= 4U, "DMA buffer size must be at least 4");
BUILD_ASSERT((GD32_ETH_DMA_BUF_SIZE % 4U) == 0U, "DMA buffer size must be 32-bit aligned");
BUILD_ASSERT((GD32_ETH_DMA_BUF_SIZE % GD32_ETH_DMA_ALIGN) == 0U,
	     "DMA buffer size must be cache-line aligned");
BUILD_ASSERT(GD32_ETH_DMA_BUF_SIZE <= 0x1FFFU, "DMA buffer size must fit RB1S field");
BUILD_ASSERT(GD32_ETH_TX_DESC_COUNT >= 4, "Need at least 4 TX descriptors");
BUILD_ASSERT(GD32_ETH_RX_DESC_COUNT >= 8, "Need at least 8 RX descriptors");
BUILD_ASSERT((GD32_ETH_RX_DESC_COUNT * GD32_ETH_RX_DESC_BUF_LEN) >= GD32_ETH_FRAME_SIZE_MAX,
	     "RX ring too small for a full frame");
BUILD_ASSERT((GD32_ETH_TX_DESC_COUNT * GD32_ETH_TX_DESC_BUF_LEN) >= GD32_ETH_FRAME_SIZE_MAX,
	     "TX ring too small for a full frame");

#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
BUILD_ASSERT(sizeof(enet_descriptors_struct) == 32, "Enhanced mode expects 32-byte descriptors");
#else
BUILD_ASSERT(sizeof(enet_descriptors_struct) == 16, "Normal mode expects 16-byte descriptors");
#endif
BUILD_ASSERT((GD32_ETH_DMA_DESC_SIZE % GD32_ETH_DMA_ALIGN) == 0U,
	     "Descriptor slot size must be cache-line aligned");

union eth_gd32_dma_desc {
	enet_descriptors_struct desc;
	uint8_t raw[GD32_ETH_DMA_DESC_SIZE];
};

struct eth_gd32_dma_rings {
	union eth_gd32_dma_desc rx_desc[GD32_ETH_RX_DESC_COUNT];
	union eth_gd32_dma_desc tx_desc[GD32_ETH_TX_DESC_COUNT];
};

struct eth_gd32_config {
	uintptr_t base;
	const struct pinctrl_dev_config *pcfg;
	struct gd32_enet_platform_config platform;
	const struct device *phy_dev;
	const struct nvmem_cell mac_nvmem;
	struct eth_gd32_dma_rings *dma;
	uint8_t (*rx_dma_buf)[GD32_ETH_RX_DMA_BUFS_PER_DESC][GD32_ETH_DMA_BUF_SIZE];
	uint8_t (*tx_dma_buf)[GD32_ETH_TX_DMA_BUFS_PER_DESC][GD32_ETH_DMA_BUF_SIZE];
	uint8_t local_mac[NET_ETH_ADDR_LEN];
	uint8_t mac_prefix[3];
	bool random_mac;
	bool has_local_mac;
	bool has_mac_prefix;
	bool has_mac_nvmem;
	void (*irq_config)(void);
};

struct eth_gd32_data {
	const struct device *dev;
	struct net_if *iface;
	uint8_t mac_addr[NET_ETH_ADDR_LEN];
	struct k_mutex tx_mutex;
	struct k_sem tx_desc_sem;
	struct k_spinlock tx_ring_lock;
	struct net_pkt *tx_pkt_refs[GD32_ETH_TX_DESC_COUNT];
	struct k_work rx_work;
	struct k_work recover_work;
	atomic_t rx_work_pending;
	atomic_t rx_resume_pending;
	atomic_t recovering;
	atomic_t link_up;
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
#if !defined(CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST)
	uint8_t mcast_hash_refcnt[64];
#endif
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
	uint16_t err_count;
};

static inline uint16_t modulo_inc(uint16_t idx, uint16_t max)
{
	idx++;
	return (idx < max) ? idx : 0U;
}

static inline uint32_t eth_gd32_reg_read(const struct eth_gd32_config *cfg, uint32_t offset)
{
	return sys_read32(cfg->base + offset);
}

static inline void eth_gd32_reg_write(const struct eth_gd32_config *cfg, uint32_t offset,
				      uint32_t value)
{
	sys_write32(value, cfg->base + offset);
}

static inline void eth_gd32_reg_set_bits(const struct eth_gd32_config *cfg, uint32_t offset,
					 uint32_t mask)
{
	eth_gd32_reg_write(cfg, offset, eth_gd32_reg_read(cfg, offset) | mask);
}

static inline void eth_gd32_reg_clear_bits(const struct eth_gd32_config *cfg, uint32_t offset,
					   uint32_t mask)
{
	eth_gd32_reg_write(cfg, offset, eth_gd32_reg_read(cfg, offset) & ~mask);
}

static inline struct eth_gd32_dma_rings *eth_gd32_dma(const struct device *dev)
{
	const struct eth_gd32_config *cfg = dev->config;

	return cfg->dma;
}

static inline uint8_t *eth_gd32_rx_dma_buf(const struct device *dev, uint16_t desc_idx)
{
	const struct eth_gd32_config *cfg = dev->config;

	return cfg->rx_dma_buf[desc_idx][0];
}

static inline uint8_t *eth_gd32_tx_dma_buf(const struct device *dev, uint16_t desc_idx,
					   uint8_t buf_idx)
{
	const struct eth_gd32_config *cfg = dev->config;

	return cfg->tx_dma_buf[desc_idx][buf_idx];
}

static inline enet_descriptors_struct *eth_gd32_rx_desc(struct eth_gd32_dma_rings *dma,
							uint16_t idx)
{
	return &dma->rx_desc[idx].desc;
}

static inline enet_descriptors_struct *eth_gd32_tx_desc(struct eth_gd32_dma_rings *dma,
							uint16_t idx)
{
	return &dma->tx_desc[idx].desc;
}

static inline bool gd32_dma_addr_ok(uintptr_t addr, size_t len)
{
#if IS_ENABLED(CONFIG_MEM_ATTR)
	int ret;

	if (IS_ENABLED(CONFIG_MMU) || (len == 0U)) {
		return true;
	}

	if ((addr + len) < addr) {
		return false;
	}

	ret = mem_attr_check_buf((void *)addr, len, DT_MEM_DMA);
	if (ret == 0 || ret == -ENOSYS) {
		return true;
	}

	/*
	 * Many GD32 boards do not yet describe general SRAM with memory
	 * attributes. Keep the old "assume DMA-capable unless explicitly
	 * rejected" behavior for unannotated regions, while still rejecting
	 * buffers in regions tagged as non-DMA.
	 */
	return ret == -ENOBUFS;
#else
	ARG_UNUSED(addr);
	ARG_UNUSED(len);
#endif

	return true;
}

static inline void gd32_cache_invalidate(const void *addr, size_t len)
{
	uintptr_t start;
	uintptr_t end;

	if (len == 0U) {
		return;
	}

#if defined(CONFIG_DCACHE_LINE_SIZE)
	start = ROUND_DOWN((uintptr_t)addr, GD32_ETH_DMA_ALIGN);
	end = ROUND_UP((uintptr_t)addr + len, GD32_ETH_DMA_ALIGN);
#else
	start = (uintptr_t)addr;
	end = (uintptr_t)addr + len;
#endif

	(void)sys_cache_data_invd_range((void *)start, end - start);
}

static inline void gd32_cache_clean(const void *addr, size_t len)
{
	uintptr_t start;
	uintptr_t end;

	if (len == 0U) {
		return;
	}

#if defined(CONFIG_DCACHE_LINE_SIZE)
	start = ROUND_DOWN((uintptr_t)addr, GD32_ETH_DMA_ALIGN);
	end = ROUND_UP((uintptr_t)addr + len, GD32_ETH_DMA_ALIGN);
#else
	start = (uintptr_t)addr;
	end = (uintptr_t)addr + len;
#endif

	(void)sys_cache_data_flush_range((void *)start, end - start);
}

static inline void eth_gd32_desc_refresh(const enet_descriptors_struct *desc)
{
	gd32_cache_invalidate(desc, GD32_ETH_DMA_DESC_SIZE);
}

static inline void eth_gd32_desc_publish(const enet_descriptors_struct *desc)
{
	gd32_cache_clean(desc, GD32_ETH_DMA_DESC_SIZE);
}

static inline bool eth_gd32_rx_desc_dma_owned(const enet_descriptors_struct *desc)
{
	eth_gd32_desc_refresh(desc);
	return (desc->status & ENET_RDES0_DAV) != 0U;
}

static inline bool eth_gd32_tx_desc_dma_owned(const enet_descriptors_struct *desc)
{
	eth_gd32_desc_refresh(desc);
	return (desc->status & ENET_TDES0_DAV) != 0U;
}

static void eth_gd32_trace_state(const struct device *dev, const char *tag, uint32_t stat,
				 bool warn)
{
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_data *data = dev->data;
	struct eth_gd32_dma_rings *dma = eth_gd32_dma(dev);
	uint32_t crdaddr = eth_gd32_reg_read(cfg, GD32_ENET_DMA_CRDADDR_OFFSET);
	uint32_t crbaddr = eth_gd32_reg_read(cfg, GD32_ENET_DMA_CRBADDR_OFFSET);
	uint16_t rx_dma_owned = 0U;
	uint16_t tx_dma_owned = 0U;
	uint16_t tx_pkt_refs = 0U;
	int16_t rx_curr_idx = -1;
	uint32_t rx_curr_status = 0U;
	uint32_t rx_curr_ctl = 0U;

	for (uint16_t i = 0U; i < GD32_ETH_RX_DESC_COUNT; i++) {
		enet_descriptors_struct *desc = eth_gd32_rx_desc(dma, i);

		if ((uint32_t)(uintptr_t)desc == crdaddr) {
			rx_curr_idx = (int16_t)i;
			eth_gd32_desc_refresh(desc);
			rx_curr_status = desc->status;
			rx_curr_ctl = desc->control_buffer_size;
		}

		if (eth_gd32_rx_desc_dma_owned(desc)) {
			rx_dma_owned++;
		}
	}

	for (uint16_t i = 0U; i < GD32_ETH_TX_DESC_COUNT; i++) {
		if (eth_gd32_tx_desc_dma_owned(eth_gd32_tx_desc(dma, i))) {
			tx_dma_owned++;
		}

		if (data->tx_pkt_refs[i] != NULL) {
			tx_pkt_refs++;
		}
	}

	if (warn) {
		LOG_WRN("%s: IRQ_STAT=0x%08x DMA_STAT=0x%08x DMA_CTL=0x%08x MAC_CFG=0x%08x "
			"rx_tail=%u rx_curr=%d crd=0x%08x crb=0x%08x rdes0=0x%08x rdes1=0x%08x "
			"tx_use=%u tx_clean=%u tx_in_use=%u tx_sem=%u "
			"rx_owned=%u/%u tx_owned=%u/%u tx_pkt_refs=%u",
			tag, stat, eth_gd32_reg_read(cfg, GD32_ENET_DMA_STAT_OFFSET),
			eth_gd32_reg_read(cfg, GD32_ENET_DMA_CTL_OFFSET),
			eth_gd32_reg_read(cfg, GD32_ENET_MAC_CFG_OFFSET), data->rx_tail, rx_curr_idx,
			crdaddr, crbaddr, rx_curr_status, rx_curr_ctl, data->tx_next_to_use,
			data->tx_next_to_clean, data->tx_descs_in_use,
			data->tx_desc_sem.count, rx_dma_owned, GD32_ETH_RX_DESC_COUNT, tx_dma_owned,
			GD32_ETH_TX_DESC_COUNT, tx_pkt_refs);
	} else {
		LOG_DBG("%s: IRQ_STAT=0x%08x DMA_STAT=0x%08x DMA_CTL=0x%08x MAC_CFG=0x%08x "
			"rx_tail=%u rx_curr=%d crd=0x%08x crb=0x%08x rdes0=0x%08x rdes1=0x%08x "
			"tx_use=%u tx_clean=%u tx_in_use=%u tx_sem=%u "
			"rx_owned=%u/%u tx_owned=%u/%u tx_pkt_refs=%u",
			tag, stat, eth_gd32_reg_read(cfg, GD32_ENET_DMA_STAT_OFFSET),
			eth_gd32_reg_read(cfg, GD32_ENET_DMA_CTL_OFFSET),
			eth_gd32_reg_read(cfg, GD32_ENET_MAC_CFG_OFFSET), data->rx_tail, rx_curr_idx,
			crdaddr, crbaddr, rx_curr_status, rx_curr_ctl, data->tx_next_to_use,
			data->tx_next_to_clean, data->tx_descs_in_use,
			data->tx_desc_sem.count, rx_dma_owned, GD32_ETH_RX_DESC_COUNT, tx_dma_owned,
			GD32_ETH_TX_DESC_COUNT, tx_pkt_refs);
	}
}

static inline void eth_gd32_rx_buf_prepare_for_dma(void *buf, size_t len)
{
	gd32_cache_invalidate(buf, len);
}

static inline bool eth_gd32_desc_is_last(uint16_t idx, uint16_t count)
{
	return idx == (count - 1U);
}

static inline uint32_t eth_gd32_tx_desc_static_status(bool last)
{
	return last ? ENET_TDES0_TERM : 0U;
}

static inline uint32_t eth_gd32_tx_desc_frame_status(uint16_t idx, uint16_t first_idx,
						     uint16_t remaining)
{
	uint32_t status = ENET_CHECKSUM_DISABLE;

	if (idx == first_idx) {
		status |= ENET_TDES0_FSG;
	}

	if (remaining == 1U) {
		status |= ENET_TDES0_LSG | ENET_TDES0_INTC;
	}

	return status;
}

static inline void eth_gd32_rx_desc_give_to_dma(const struct device *dev,
						enet_descriptors_struct *desc, uint16_t idx);

static void eth_gd32_rx_desc_rearm(const struct device *dev, enet_descriptors_struct *desc, uint16_t idx)
{
	eth_gd32_rx_buf_prepare_for_dma(eth_gd32_rx_dma_buf(dev, idx), GD32_ETH_DMA_BUF_SIZE);

	eth_gd32_rx_desc_give_to_dma(dev, desc, idx);
}

static bool eth_gd32_rx_desc_copy(struct net_pkt *pkt, const struct device *dev, uint16_t idx,
				  uint32_t *copied, uint32_t frame_len)
{
	uint32_t frag_len = MIN(frame_len - *copied, (uint32_t)GD32_ETH_DMA_BUF_SIZE);
	void *rx_buf;

	if (frag_len == 0U) {
		return true;
	}

	rx_buf = eth_gd32_rx_dma_buf(dev, idx);
	gd32_cache_invalidate(rx_buf, frag_len);
	if (net_pkt_write(pkt, rx_buf, frag_len) != 0) {
		return false;
	}

	*copied += frag_len;
	return true;
}

static inline void eth_gd32_rx_desc_give_to_dma(const struct device *dev,
						enet_descriptors_struct *desc, uint16_t idx)
{
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_dma_rings *dma = cfg->dma;

	desc->buffer1_addr = (uint32_t)(uintptr_t)eth_gd32_rx_dma_buf(dev, idx);
	desc->buffer2_next_desc_addr =
		(uint32_t)(uintptr_t)eth_gd32_rx_desc(dma, modulo_inc(idx, GD32_ETH_RX_DESC_COUNT));
	barrier_dmem_fence_full();
	desc->status = ENET_RDES0_DAV;
	eth_gd32_desc_publish(desc);
}

static inline void eth_gd32_tx_desc_reset(enet_descriptors_struct *desc, bool last)
{
	desc->status = eth_gd32_tx_desc_static_status(last);
	desc->control_buffer_size = 0U;
	desc->buffer1_addr = 0U;
	desc->buffer2_next_desc_addr = 0U;
	eth_gd32_desc_publish(desc);
}

static inline void eth_gd32_tx_desc_setup(enet_descriptors_struct *desc, uint32_t status,
					  const void *buf1, size_t len1, const void *buf2, size_t len2)
{
	/*
	 * Ring-mode metadata lives in TDES0 and must survive every reuse of the
	 * descriptor. In particular, the last descriptor must keep TERM set or
	 * TxDMA will stop wrapping back to the start of the ring.
	 */
	status |= desc->status & ENET_TDES0_TERM;
	desc->control_buffer_size = FIELD_PREP(ENET_TDES1_TB1S, len1) |
				    FIELD_PREP(ENET_TDES1_TB2S, len2);
	desc->buffer1_addr = (uint32_t)(uintptr_t)buf1;
	desc->buffer2_next_desc_addr = (uint32_t)(uintptr_t)buf2;
	desc->status = status;
}

static inline void eth_gd32_tx_desc_give_to_dma(enet_descriptors_struct *desc)
{
	desc->status |= ENET_TDES0_DAV;
	eth_gd32_desc_publish(desc);
}

static inline void eth_gd32_tx_poll_demand(const struct eth_gd32_config *cfg)
{
	eth_gd32_reg_write(cfg, GD32_ENET_DMA_TPEN_OFFSET, GD32_ETH_DMA_POLL_DEMAND);
}

static bool eth_gd32_tx_has_pending_descs(const struct device *dev)
{
	struct eth_gd32_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->tx_ring_lock);
	bool pending = data->tx_descs_in_use != 0U;

	k_spin_unlock(&data->tx_ring_lock, key);

	return pending;
}

/*
 * TBU is the normal "ring empty" suspend condition. Only the path that has
 * just published new DAV=1 descriptors should wake TxDMA; waking it from the
 * idle IRQ path makes the DMA re-read the same empty descriptor and raise TBU
 * again immediately.
 */
static void eth_gd32_tx_resume_after_publish(const struct eth_gd32_config *cfg)
{
	uint32_t clear = eth_gd32_reg_read(cfg, GD32_ENET_DMA_STAT_OFFSET) &
			 (ENET_DMA_STAT_TBU | ENET_DMA_STAT_TU);

	if (clear != 0U) {
		eth_gd32_reg_write(cfg, GD32_ENET_DMA_STAT_OFFSET, clear);
	}

	eth_gd32_tx_poll_demand(cfg);
}

static inline void eth_gd32_rx_poll_demand(const struct eth_gd32_config *cfg)
{
	eth_gd32_reg_write(cfg, GD32_ENET_DMA_RPEN_OFFSET, GD32_ETH_DMA_POLL_DEMAND);
}

static bool eth_gd32_rx_dma_suspended(uint32_t stat)
{
	return ((stat & ENET_DMA_STAT_RBU) != 0U) || (FIELD_GET(ENET_DMA_STAT_RP, stat) == 4U);
}

/*
 * Resume RxDMA as soon as software has returned descriptors. Waiting until the
 * whole batch is handed to the stack leaves the MAC suspended on bursty
 * traffic even though fresh DAV=1 descriptors are already available.
 */
static void eth_gd32_rx_resume_if_suspended(const struct eth_gd32_config *cfg)
{
	uint32_t stat = eth_gd32_reg_read(cfg, GD32_ENET_DMA_STAT_OFFSET);
	uint32_t clear;

	if (!eth_gd32_rx_dma_suspended(stat)) {
		return;
	}

	clear = stat & (ENET_DMA_STAT_RBU | ENET_DMA_STAT_RPS);
	if (clear != 0U) {
		eth_gd32_reg_write(cfg, GD32_ENET_DMA_STAT_OFFSET, clear);
	}

	eth_gd32_rx_poll_demand(cfg);
}

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

static int eth_gd32_wait_reg_clear(const struct eth_gd32_config *cfg, uint32_t offset,
				   uint32_t mask, uint32_t timeout_us)
{
	if (!WAIT_FOR((eth_gd32_reg_read(cfg, offset) & mask) == 0U, timeout_us, k_usleep(10))) {
		return -ETIMEDOUT;
	}

	return 0;
}

static int eth_gd32_txfifo_flush(const struct eth_gd32_config *cfg)
{
	eth_gd32_reg_set_bits(cfg, GD32_ENET_DMA_CTL_OFFSET, ENET_DMA_CTL_FTF);
	return eth_gd32_wait_reg_clear(cfg, GD32_ENET_DMA_CTL_OFFSET, ENET_DMA_CTL_FTF,
				       GD32_ETH_TX_FLUSH_TIMEOUT_US);
}

static int eth_gd32_wait_for_stop_state(const struct eth_gd32_config *cfg, uint32_t mask,
					uint32_t want, uint32_t timeout_us)
{
	if (!WAIT_FOR((eth_gd32_reg_read(cfg, GD32_ENET_DMA_STAT_OFFSET) & mask) == want,
		      timeout_us, k_usleep(10))) {
		return -ETIMEDOUT;
	}

	return 0;
}

static void eth_gd32_enable(const struct eth_gd32_config *cfg)
{
	eth_gd32_reg_set_bits(cfg, GD32_ENET_MAC_CFG_OFFSET, ENET_MAC_CFG_TEN | ENET_MAC_CFG_REN);
	(void)eth_gd32_txfifo_flush(cfg);
	eth_gd32_reg_set_bits(cfg, GD32_ENET_DMA_CTL_OFFSET, ENET_DMA_CTL_STE | ENET_DMA_CTL_SRE);
}

static void eth_gd32_disable(const struct eth_gd32_config *cfg)
{
	eth_gd32_reg_clear_bits(cfg, GD32_ENET_DMA_CTL_OFFSET, ENET_DMA_CTL_STE);
	(void)eth_gd32_wait_for_stop_state(cfg, ENET_DMA_STAT_TP, FIELD_PREP(ENET_DMA_STAT_TP, 0U),
					   GD32_ETH_STOP_TIMEOUT_US);
	eth_gd32_reg_clear_bits(cfg, GD32_ENET_MAC_CFG_OFFSET, ENET_MAC_CFG_TEN | ENET_MAC_CFG_REN);
	eth_gd32_reg_clear_bits(cfg, GD32_ENET_DMA_CTL_OFFSET, ENET_DMA_CTL_SRE);
	(void)eth_gd32_wait_for_stop_state(cfg, ENET_DMA_STAT_RP, FIELD_PREP(ENET_DMA_STAT_RP, 0U),
					   GD32_ETH_STOP_TIMEOUT_US);
	(void)eth_gd32_txfifo_flush(cfg);
}

static void eth_gd32_mac_address_set(const struct eth_gd32_config *cfg,
				     enet_macaddress_enum mac_addr,
				     const uint8_t mac[NET_ETH_ADDR_LEN])
{
	eth_gd32_reg_write(cfg, GD32_ENET_MAC_ADDR0H_OFFSET + (uint32_t)mac_addr,
			   ENET_SET_MACADDRH(mac));
	eth_gd32_reg_write(cfg, GD32_ENET_MAC_ADDR0L_OFFSET + (uint32_t)mac_addr,
			   ENET_SET_MACADDRL(mac));
}

static int eth_gd32_software_reset(const struct eth_gd32_config *cfg)
{
	eth_gd32_reg_set_bits(cfg, GD32_ENET_DMA_BCTL_OFFSET, ENET_DMA_BCTL_SWR);
	return eth_gd32_wait_reg_clear(cfg, GD32_ENET_DMA_BCTL_OFFSET, ENET_DMA_BCTL_SWR,
				       GD32_ETH_RESET_TIMEOUT_US);
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

	for (uint16_t i = 0U; i < reclaim->err_count; i++) {
		if (data->iface != NULL) {
			eth_stats_update_errors_tx(data->iface);
		}
	}
}

static void eth_gd32_tx_reclaim_completed(const struct device *dev,
					  struct eth_gd32_tx_reclaim *reclaim)
{
	struct eth_gd32_data *data = dev->data;
	struct eth_gd32_dma_rings *dma = eth_gd32_dma(dev);
	k_spinlock_key_t key = k_spin_lock(&data->tx_ring_lock);

	while (data->tx_descs_in_use > 0U) {
		uint16_t idx = data->tx_next_to_clean;
		enet_descriptors_struct *desc = eth_gd32_tx_desc(dma, idx);

		if (eth_gd32_tx_desc_dma_owned(desc)) {
			break;
		}

		if (((desc->status & ENET_TDES0_LSG) != 0U) && ((desc->status & ENET_TDES0_ES) != 0U)) {
			reclaim->err_count++;
		}

		if (data->tx_pkt_refs[idx] != NULL) {
			reclaim->pkts[reclaim->pkt_count++] = data->tx_pkt_refs[idx];
			data->tx_pkt_refs[idx] = NULL;
		}

			eth_gd32_tx_desc_reset(desc, eth_gd32_desc_is_last(idx, GD32_ETH_TX_DESC_COUNT));
		data->tx_next_to_clean = modulo_inc(idx, GD32_ETH_TX_DESC_COUNT);
		data->tx_descs_in_use--;
		reclaim->desc_count++;
	}

	k_spin_unlock(&data->tx_ring_lock, key);
}

static void eth_gd32_reclaim_tx(const struct device *dev)
{
	struct eth_gd32_tx_reclaim reclaim = {0};

	eth_gd32_tx_reclaim_completed(dev, &reclaim);
	eth_gd32_tx_reclaim_flush(dev->data, &reclaim);
}

static void eth_gd32_tx_reset(const struct device *dev)
{
	struct eth_gd32_data *data = dev->data;
	struct eth_gd32_dma_rings *dma = eth_gd32_dma(dev);
	struct eth_gd32_tx_reclaim reclaim = {0};
	k_spinlock_key_t key = k_spin_lock(&data->tx_ring_lock);

	for (uint16_t i = 0U; i < GD32_ETH_TX_DESC_COUNT; i++) {
		if (data->tx_pkt_refs[i] != NULL) {
			reclaim.pkts[reclaim.pkt_count++] = data->tx_pkt_refs[i];
			data->tx_pkt_refs[i] = NULL;
		}

		eth_gd32_tx_desc_reset(eth_gd32_tx_desc(dma, i),
				       eth_gd32_desc_is_last(i, GD32_ETH_TX_DESC_COUNT));
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
		eth_gd32_trace_state(dev, "TX descriptor reserve timeout",
				     eth_gd32_reg_read(dev->config, GD32_ENET_DMA_STAT_OFFSET),
				     true);
		return -ENOBUFS;
	}

	return 0;
}

static void eth_gd32_dma_irqs_enable(const struct eth_gd32_config *cfg)
{
	/*
	 * ENET_DMA_INT_* are HAL helper selectors that encode register offset
	 * and bit position. Direct MMIO writes must use the raw ENET_DMA_INTEN_*
	 * register bits instead.
	 */
	const uint32_t mask = ENET_DMA_INTEN_NIE | ENET_DMA_INTEN_AIE | ENET_DMA_INTEN_RIE |
			      ENET_DMA_INTEN_RBUIE | ENET_DMA_INTEN_RPSIE |
			      ENET_DMA_INTEN_ROIE |
			      ENET_DMA_INTEN_TIE | ENET_DMA_INTEN_TPSIE | ENET_DMA_INTEN_TUIE |
			      ENET_DMA_INTEN_FBEIE;

	eth_gd32_reg_set_bits(cfg, GD32_ENET_DMA_INTEN_OFFSET, mask);
}

static void eth_gd32_dma_irqs_clear_all(const struct eth_gd32_config *cfg)
{
	const uint32_t clear = ENET_DMA_STAT_TS | ENET_DMA_STAT_TBU | ENET_DMA_STAT_TJT |
			       ENET_DMA_STAT_TU | ENET_DMA_STAT_RS | ENET_DMA_STAT_RBU |
			       ENET_DMA_STAT_RO | ENET_DMA_STAT_RPS | ENET_DMA_STAT_TPS |
			       ENET_DMA_STAT_RWT | ENET_DMA_STAT_ET | ENET_DMA_STAT_FBE |
			       ENET_DMA_STAT_ER | ENET_DMA_STAT_AI | ENET_DMA_STAT_NI;

	eth_gd32_reg_write(cfg, GD32_ENET_DMA_STAT_OFFSET, clear);
}

#if !defined(CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST)
static uint32_t eth_gd32_mcast_hash_index_get(const struct net_eth_addr *addr)
{
	uint32_t crc = __RBIT(crc32_ieee(addr->addr, sizeof(addr->addr)));

	return (crc >> 26) & 0x3FU;
}

static void eth_gd32_mcast_hash_sync(const struct device *dev)
{
	const struct eth_gd32_config *cfg = dev->config;
	const struct eth_gd32_data *data = dev->data;
	uint32_t hash_table[2] = {0U};

	for (uint32_t i = 0U; i < ARRAY_SIZE(data->mcast_hash_refcnt); i++) {
		if (data->mcast_hash_refcnt[i] != 0U) {
			hash_table[i / 32U] |= BIT(i % 32U);
		}
	}

	eth_gd32_reg_write(cfg, GD32_ENET_MAC_HLL_OFFSET, hash_table[0]);
	eth_gd32_reg_write(cfg, GD32_ENET_MAC_HLH_OFFSET, hash_table[1]);
}
#endif

static void eth_gd32_rings_configure(const struct device *dev)
{
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_data *data = dev->data;
	struct eth_gd32_dma_rings *dma = cfg->dma;

	for (uint16_t i = 0U; i < GD32_ETH_RX_DESC_COUNT; i++) {
		enet_descriptors_struct *desc = eth_gd32_rx_desc(dma, i);

		desc->control_buffer_size = FIELD_PREP(ENET_RDES1_RB1S, GD32_ETH_DMA_BUF_SIZE) |
					    ENET_RDES1_RCHM;
		desc->control_buffer_size &= ~ENET_RDES1_DINTC;
		if (eth_gd32_desc_is_last(i, GD32_ETH_RX_DESC_COUNT)) {
			desc->control_buffer_size |= ENET_RDES1_RERM;
		}

		eth_gd32_rx_buf_prepare_for_dma(eth_gd32_rx_dma_buf(dev, i), GD32_ETH_DMA_BUF_SIZE);
		eth_gd32_rx_desc_give_to_dma(dev, desc, i);
	}

	for (uint16_t i = 0U; i < GD32_ETH_TX_DESC_COUNT; i++) {
		enet_descriptors_struct *desc = eth_gd32_tx_desc(dma, i);

		eth_gd32_tx_desc_reset(desc, eth_gd32_desc_is_last(i, GD32_ETH_TX_DESC_COUNT));
		data->tx_pkt_refs[i] = NULL;
	}

	data->rx_tail = 0U;
	data->tx_next_to_use = 0U;
	data->tx_next_to_clean = 0U;
	data->tx_descs_in_use = 0U;

	gd32_cache_clean(dma, sizeof(*dma));
	eth_gd32_reg_write(cfg, GD32_ENET_DMA_RDTADDR_OFFSET,
			   (uint32_t)(uintptr_t)eth_gd32_rx_desc(dma, 0U));
	eth_gd32_reg_write(cfg, GD32_ENET_DMA_TDTADDR_OFFSET,
			   (uint32_t)(uintptr_t)eth_gd32_tx_desc(dma, 0U));
	barrier_dmem_fence_full();
}

static void eth_gd32_mac_dma_default_config(const struct device *dev)
{
	const struct eth_gd32_config *cfg = dev->config;
	uint32_t mac_cfg = eth_gd32_reg_read(cfg, GD32_ENET_MAC_CFG_OFFSET);
	uint32_t frmf = eth_gd32_reg_read(cfg, GD32_ENET_MAC_FRMF_OFFSET);
	uint32_t dma_ctl = eth_gd32_reg_read(cfg, GD32_ENET_DMA_CTL_OFFSET);
	uint32_t dma_bctl = 0U;

	mac_cfg &= ~(ENET_MAC_CFG_TEN | ENET_MAC_CFG_REN | ENET_MAC_CFG_LBM | ENET_MAC_CFG_IPFCO |
		     ENET_MAC_CFG_APCD | ENET_MAC_CFG_TFCD | ENET_MAC_CFG_DFC | ENET_MAC_CFG_ROD |
		     ENET_MAC_CFG_RTD | ENET_MAC_CFG_CSD | ENET_MAC_CFG_WDD | ENET_MAC_CFG_JBD |
		     ENET_MAC_CFG_IGBS | ENET_MAC_CFG_BOL | ENET_MAC_CFG_SPD | ENET_MAC_CFG_DPM);
	mac_cfg |= ENET_SPEEDMODE_100M | ENET_MODE_FULLDUPLEX | ENET_INTERFRAMEGAP_96BIT |
		   ENET_BACKOFFLIMIT_10 | ENET_MAC_CFG_APCD | ENET_MAC_CFG_TFCD;
	eth_gd32_reg_write(cfg, GD32_ENET_MAC_CFG_OFFSET, mac_cfg);

	frmf &= ~(ENET_MAC_FRMF_FAR | ENET_MAC_FRMF_PM | ENET_MAC_FRMF_BFRMD | ENET_MAC_FRMF_PCFRM |
		  ENET_MAC_FRMF_HMF | ENET_MAC_FRMF_HPFLT | ENET_MAC_FRMF_HUF | ENET_MAC_FRMF_MFD);
	frmf |= ENET_PCFRM_PREVENT_ALL;
	if (IS_ENABLED(CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST)) {
		frmf |= ENET_MULTICAST_FILTER_PASS;
	} else {
		frmf |= ENET_MULTICAST_FILTER_HASH_OR_PERFECT;
	}
	eth_gd32_reg_write(cfg, GD32_ENET_MAC_FRMF_OFFSET, frmf);
	eth_gd32_reg_write(cfg, GD32_ENET_MAC_HLH_OFFSET, 0U);
	eth_gd32_reg_write(cfg, GD32_ENET_MAC_HLL_OFFSET, 0U);
	eth_gd32_reg_write(cfg, GD32_ENET_MAC_FCTL_OFFSET, 0U);
	eth_gd32_reg_write(cfg, GD32_ENET_MAC_VLT_OFFSET, 0U);

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

	eth_gd32_reg_write(cfg, GD32_ENET_DMA_CTL_OFFSET, dma_ctl);

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

	eth_gd32_reg_write(cfg, GD32_ENET_DMA_BCTL_OFFSET, dma_bctl);
}

static int eth_gd32_hw_reset_and_configure(const struct device *dev)
{
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_data *data = dev->data;

	eth_gd32_disable(cfg);

	if (eth_gd32_software_reset(cfg) != 0) {
		return -EIO;
	}

	eth_gd32_mac_dma_default_config(dev);
	eth_gd32_mac_address_set(cfg, ENET_MAC_ADDRESS0, data->mac_addr);
#if !defined(CONFIG_ETH_GD32_ACCEPT_ALL_MULTICAST)
	eth_gd32_mcast_hash_sync(dev);
#endif
	eth_gd32_rings_configure(dev);
	eth_gd32_dma_irqs_enable(cfg);
	eth_gd32_dma_irqs_clear_all(cfg);

	return 0;
}

static void eth_gd32_rx(const struct device *dev, bool resume_pending)
{
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_data *data = dev->data;
	struct eth_gd32_dma_rings *dma = cfg->dma;

	while (data->iface != NULL) {
		enet_descriptors_struct *start_desc = eth_gd32_rx_desc(dma, data->rx_tail);

		if (eth_gd32_rx_desc_dma_owned(start_desc)) {
			break;
		}

		if ((start_desc->status & ENET_RDES0_FDES) == 0U) {
			eth_gd32_rx_desc_rearm(dev, start_desc, data->rx_tail);
			data->rx_tail = modulo_inc(data->rx_tail, GD32_ETH_RX_DESC_COUNT);
			eth_gd32_rx_resume_if_suspended(cfg);
			continue;
		}

		uint16_t idx = data->rx_tail;
		uint16_t cnt = 0U;
		bool complete = false;
		bool waiting_for_dma = false;

		while (cnt < GD32_ETH_RX_DESC_COUNT) {
			enet_descriptors_struct *desc = eth_gd32_rx_desc(dma, idx);

			if (eth_gd32_rx_desc_dma_owned(desc)) {
				waiting_for_dma = true;
				break;
			}

			cnt++;
			if ((desc->status & ENET_RDES0_ERRS) != 0U) {
				complete = true;
				break;
			}
			if ((desc->status & ENET_RDES0_LDES) != 0U) {
				complete = true;
				break;
			}

			idx = modulo_inc(idx, GD32_ETH_RX_DESC_COUNT);
		}

		if (!complete) {
			if (waiting_for_dma) {
				break;
			}

			eth_stats_update_errors_rx(data->iface);

				idx = data->rx_tail;
				for (uint16_t n = 0U; n < cnt; n++) {
					enet_descriptors_struct *desc = eth_gd32_rx_desc(dma, idx);

					eth_gd32_rx_desc_rearm(dev, desc, idx);
					idx = modulo_inc(idx, GD32_ETH_RX_DESC_COUNT);
				}

			data->rx_tail = idx;
			eth_gd32_rx_resume_if_suspended(cfg);
			continue;
		}

		enet_descriptors_struct *end_desc = eth_gd32_rx_desc(dma, idx);
		uint32_t frame_len = GET_RDES0_FRML(end_desc->status);
		bool rx_ok = (frame_len > 0U) && (frame_len <= GD32_ETH_FRAME_SIZE_MAX) &&
			     ((end_desc->status & ENET_RDES0_ERRS) == 0U);
		struct net_pkt *pkt = NULL;

		if (rx_ok) {
			pkt = net_pkt_rx_alloc_with_buffer(data->iface, frame_len, NET_AF_UNSPEC, 0,
							   K_NO_WAIT);
			if (pkt == NULL) {
				rx_ok = false;
			}
		}

		if (!rx_ok) {
			if (pkt != NULL) {
				net_pkt_unref(pkt);
			}

			eth_stats_update_errors_rx(data->iface);

				idx = data->rx_tail;
				for (uint16_t n = 0U; n < cnt; n++) {
					enet_descriptors_struct *desc = eth_gd32_rx_desc(dma, idx);

					eth_gd32_rx_desc_rearm(dev, desc, idx);
					idx = modulo_inc(idx, GD32_ETH_RX_DESC_COUNT);
				}

			data->rx_tail = idx;
			eth_gd32_rx_resume_if_suspended(cfg);
			continue;
		}

		uint32_t copied = 0U;

		idx = data->rx_tail;

			for (uint16_t n = 0U; n < cnt; n++) {
				enet_descriptors_struct *desc = eth_gd32_rx_desc(dma, idx);

				if (!eth_gd32_rx_desc_copy(pkt, dev, idx, &copied, frame_len)) {
					rx_ok = false;
				}

				eth_gd32_rx_desc_rearm(dev, desc, idx);
				idx = modulo_inc(idx, GD32_ETH_RX_DESC_COUNT);
			}

		data->rx_tail = idx;
		eth_gd32_rx_resume_if_suspended(cfg);

		if (!rx_ok || copied != frame_len || net_recv_data(data->iface, pkt) < 0) {
			eth_stats_update_errors_rx(data->iface);
			net_pkt_unref(pkt);
		}
	}

	if (resume_pending) {
		eth_gd32_rx_resume_if_suspended(cfg);
	}
}

static bool eth_gd32_rx_stuck(const struct device *dev)
{
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_dma_rings *dma = eth_gd32_dma(dev);
	uint32_t stat = eth_gd32_reg_read(cfg, GD32_ENET_DMA_STAT_OFFSET);
	uint32_t crdaddr;

	if (!eth_gd32_rx_dma_suspended(stat)) {
		return false;
	}

	crdaddr = eth_gd32_reg_read(cfg, GD32_ENET_DMA_CRDADDR_OFFSET);
	for (uint16_t i = 0U; i < GD32_ETH_RX_DESC_COUNT; i++) {
		enet_descriptors_struct *desc = eth_gd32_rx_desc(dma, i);

		if ((uint32_t)(uintptr_t)desc == crdaddr) {
			return eth_gd32_rx_desc_dma_owned(desc);
		}
	}

	return true;
}

static void eth_gd32_rx_work_handler(struct k_work *work)
{
	struct eth_gd32_data *data = CONTAINER_OF(work, struct eth_gd32_data, rx_work);

	for (;;) {
		bool resume_pending = atomic_cas(&data->rx_resume_pending, 1, 0);

		eth_gd32_rx(data->dev, resume_pending);

		if (atomic_get(&data->link_up) != 0 && eth_gd32_rx_stuck(data->dev)) {
			eth_gd32_trace_state(
				data->dev, "RX suspended with descriptors returned",
				eth_gd32_reg_read(data->dev->config, GD32_ENET_DMA_STAT_OFFSET),
				true);
			if (atomic_cas(&data->recovering, 0, 1)) {
				(void)k_work_submit(&data->recover_work);
			}
			break;
		}

		if (!atomic_cas(&data->rx_work_pending, 1, 0)) {
			break;
		}
	}
}

static void eth_gd32_set_mac_config(const struct device *dev, struct phy_link_state *state)
{
	const struct eth_gd32_config *cfg = dev->config;
	uint32_t reg = eth_gd32_reg_read(cfg, GD32_ENET_MAC_CFG_OFFSET);

	reg &= ~(ENET_MAC_CFG_SPD | ENET_MAC_CFG_DPM);
	if (PHY_LINK_IS_FULL_DUPLEX(state->speed)) {
		reg |= ENET_MAC_CFG_DPM;
	}
	if (PHY_LINK_IS_SPEED_100M(state->speed)) {
		reg |= ENET_MAC_CFG_SPD;
	}

	eth_gd32_reg_write(cfg, GD32_ENET_MAC_CFG_OFFSET, reg);
}

static void eth_gd32_recover_work_handler(struct k_work *work)
{
	struct eth_gd32_data *data = CONTAINER_OF(work, struct eth_gd32_data, recover_work);
	const struct device *dev = data->dev;
	const struct eth_gd32_config *cfg = dev->config;

	LOG_ERR("Recovering ENET after DMA fault");

	eth_gd32_reg_write(cfg, GD32_ENET_DMA_INTEN_OFFSET, 0U);
	eth_gd32_disable(cfg);

	k_mutex_lock(&data->tx_mutex, K_FOREVER);
	atomic_clear(&data->rx_work_pending);
	atomic_clear(&data->rx_resume_pending);
	eth_gd32_tx_reset(dev);
	data->rx_tail = 0U;

	if (eth_gd32_hw_reset_and_configure(dev) != 0) {
		LOG_ERR("ENET re-init failed");
	} else if (atomic_get(&data->link_up) != 0) {
		eth_gd32_enable(cfg);
		eth_gd32_rx_poll_demand(cfg);
	}

	k_mutex_unlock(&data->tx_mutex);
	atomic_clear(&data->recovering);
}

static void eth_gd32_service(const struct device *dev)
{
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_data *data = dev->data;
	uint32_t stat = eth_gd32_reg_read(cfg, GD32_ENET_DMA_STAT_OFFSET);
	uint32_t clear = 0U;
	bool rx_done = (stat & ENET_DMA_STAT_RS) != 0U;

	clear |= stat &
		 (ENET_DMA_STAT_TS | ENET_DMA_STAT_TBU | ENET_DMA_STAT_TJT | ENET_DMA_STAT_TU);
	clear |= stat &
		 (ENET_DMA_STAT_RS | ENET_DMA_STAT_RBU | ENET_DMA_STAT_RO | ENET_DMA_STAT_RPS);
	clear |= stat &
		 (ENET_DMA_STAT_TPS | ENET_DMA_STAT_RWT | ENET_DMA_STAT_ET | ENET_DMA_STAT_FBE);
	clear |= stat & (ENET_DMA_STAT_ER | ENET_DMA_STAT_AI | ENET_DMA_STAT_NI);

	if (clear != 0U) {
		eth_gd32_reg_write(cfg, GD32_ENET_DMA_STAT_OFFSET, clear);
	}

	if ((stat & (ENET_DMA_STAT_RS | ENET_DMA_STAT_RBU | ENET_DMA_STAT_RPS)) != 0U) {
		if ((stat & ENET_DMA_STAT_RBU) != 0U) {
			/*
			 * RBU is expected when RxDMA finishes a frame and runs
			 * ahead of the worker before descriptors are returned.
			 * Treat it as noteworthy only if reception did not also
			 * complete in this interrupt.
			 */
			if (!rx_done) {
				LOG_DBG("RX buffer unavailable");
				eth_gd32_trace_state(dev, "RX buffer unavailable", stat, false);
			}
			atomic_set(&data->rx_resume_pending, 1);
		}

		if ((stat & ENET_DMA_STAT_RPS) != 0U) {
			LOG_WRN("RX process stopped");
			eth_gd32_trace_state(dev, "RX process stopped", stat, true);
			atomic_set(&data->rx_resume_pending, 1);
		}

		atomic_set(&data->rx_work_pending, 1);
		(void)k_work_submit(&data->rx_work);
	}

	if ((stat & ENET_DMA_STAT_TS) != 0U) {
		eth_gd32_reclaim_tx(dev);
	}

	if ((stat & ENET_DMA_STAT_TU) != 0U) {
		LOG_WRN("TX underflow");
		eth_gd32_trace_state(dev, "TX underflow", stat, true);
		if (eth_gd32_tx_has_pending_descs(dev)) {
			/*
			 * TU is a real transmit fault: the FIFO ran dry in the
			 * middle of a frame and the manual requires clearing TU
			 * before writing TPEN to resume TxDMA.
			 */
			eth_gd32_tx_poll_demand(cfg);
		}
	}

	if ((stat & ENET_DMA_STAT_RO) != 0U) {
		LOG_WRN("RX overflow");
		eth_gd32_trace_state(dev, "RX overflow", stat, true);
	}

	if ((stat & ENET_DMA_STAT_FBE) != 0U) {
		LOG_ERR("Fatal bus error (DMA_STAT=0x%08x)", stat);
		eth_gd32_trace_state(dev, "Fatal bus error", stat, true);
		if (atomic_cas(&data->recovering, 0, 1)) {
			(void)k_work_submit(&data->recover_work);
		}
	}
}

static void eth_gd32_isr(const struct device *dev)
{
	eth_gd32_service(dev);
}

static bool eth_gd32_tx_zerocopy_possible(struct net_pkt *pkt, uint16_t *frag_count)
{
	uint16_t count = 0U;

	for (struct net_buf *frag = pkt->frags; frag != NULL; frag = frag->frags) {
		if (frag->len == 0U) {
			continue;
		}

		if (((uintptr_t)frag->data & 0x3U) != 0U ||
		    !gd32_dma_addr_ok((uintptr_t)frag->data, frag->len)) {
			return false;
		}

		count++;
		if (count > (GD32_ETH_TX_DESC_COUNT * GD32_ETH_TX_DMA_BUFS_PER_DESC)) {
			return false;
		}
	}

	*frag_count = DIV_ROUND_UP(count, GD32_ETH_TX_DMA_BUFS_PER_DESC);
	return count != 0U;
}

static size_t eth_gd32_pkt_len(struct net_pkt *pkt)
{
	size_t len = 0U;

	for (struct net_buf *frag = pkt->frags; frag != NULL; frag = frag->frags) {
		len += frag->len;
	}

	return len;
}

static int eth_gd32_tx_bounce_fill(const struct device *dev, struct net_pkt *pkt,
				   uint16_t first_idx, uint16_t desc_count)
{
	struct net_buf *frag = pkt->frags;
	size_t frag_off = 0U;
	uint16_t idx = first_idx;

	for (uint16_t n = 0U; n < desc_count; n++) {
		for (uint8_t buf_idx = 0U; buf_idx < GD32_ETH_TX_DMA_BUFS_PER_DESC; buf_idx++) {
			size_t copied = 0U;
			uint8_t *dst = eth_gd32_tx_dma_buf(dev, idx, buf_idx);

			while (copied < GD32_ETH_DMA_BUF_SIZE && frag != NULL) {
				size_t frag_len;
				size_t chunk;

				if (frag->len == 0U) {
					frag = frag->frags;
					frag_off = 0U;
					continue;
				}

				frag_len = frag->len - frag_off;
				chunk = MIN((size_t)GD32_ETH_DMA_BUF_SIZE - copied, frag_len);
				memcpy(&dst[copied], &frag->data[frag_off], chunk);
				copied += chunk;
				frag_off += chunk;

				if (frag_off == frag->len) {
					frag = frag->frags;
					frag_off = 0U;
				}
			}

			gd32_cache_clean(dst, copied);
		}

		idx = modulo_inc(idx, GD32_ETH_TX_DESC_COUNT);
	}

	return 0;
}

static int eth_gd32_tx(const struct device *dev, struct net_pkt *pkt)
{
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_data *data = dev->data;
	struct eth_gd32_dma_rings *dma = cfg->dma;
	uint16_t desc_count = 0U;
	uint16_t first_idx;
	uint16_t idx;
	uint16_t remaining;
	uint16_t last_idx = 0U;
	bool zero_copy;
	struct net_pkt *tx_pkt_ref = NULL;
	k_spinlock_key_t key;
	size_t total_len;

	if (!data->hw_ready || data->iface == NULL || atomic_get(&data->link_up) == 0) {
		return -ENETDOWN;
	}

	total_len = eth_gd32_pkt_len(pkt);
	if (total_len == 0U || total_len > GD32_ETH_FRAME_SIZE_MAX) {
		return -EINVAL;
	}

	zero_copy = eth_gd32_tx_zerocopy_possible(pkt, &desc_count);
	if (!zero_copy) {
		desc_count = DIV_ROUND_UP(total_len, GD32_ETH_TX_DESC_BUF_LEN);
		if (desc_count > GD32_ETH_TX_DESC_COUNT) {
			return -ENOBUFS;
		}
	}

	k_mutex_lock(&data->tx_mutex, K_FOREVER);

	if (eth_gd32_tx_desc_reserve(dev, desc_count) != 0) {
		k_mutex_unlock(&data->tx_mutex);
		return -ENOBUFS;
	}

	first_idx = data->tx_next_to_use;

	if (!zero_copy) {
		(void)eth_gd32_tx_bounce_fill(dev, pkt, first_idx, desc_count);
	} else {
		/*
		 * Ethernet L2 drops its own packet reference after send() returns
		 * success. Zero-copy TX therefore has to hold a private reference
		 * until the last DMA descriptor completes, while bounce TX must not
		 * keep or drop any extra packet reference at all.
		 */
		tx_pkt_ref = net_pkt_ref(pkt);
		for (struct net_buf *frag = pkt->frags; frag != NULL; frag = frag->frags) {
			if (frag->len != 0U) {
				gd32_cache_clean(frag->data, frag->len);
			}
		}
	}

	key = k_spin_lock(&data->tx_ring_lock);
	idx = first_idx;
	remaining = desc_count;

	if (zero_copy) {
		struct net_buf *frag = pkt->frags;

		while (frag != NULL) {
			struct net_buf *frag2 = NULL;
			enet_descriptors_struct *desc;
			uint32_t status;

			if (frag->len == 0U) {
				frag = frag->frags;
				continue;
			}

			desc = eth_gd32_tx_desc(dma, idx);
			status = eth_gd32_tx_desc_frame_status(idx, first_idx, remaining);
			if (remaining == 1U) {
				last_idx = idx;
			}

			for (frag2 = frag->frags; frag2 != NULL && frag2->len == 0U; frag2 = frag2->frags) {
			}

			eth_gd32_tx_desc_setup(desc, status, frag->data, frag->len,
					       (frag2 != NULL) ? frag2->data : NULL,
					       (frag2 != NULL) ? frag2->len : 0U);
			data->tx_pkt_refs[idx] = NULL;
			idx = modulo_inc(idx, GD32_ETH_TX_DESC_COUNT);
			remaining--;
			frag = (frag2 != NULL) ? frag2->frags : frag->frags;
		}
	} else {
		size_t left = total_len;

		while (left > 0U) {
			enet_descriptors_struct *desc = eth_gd32_tx_desc(dma, idx);
			size_t chunk1 = MIN(left, (size_t)GD32_ETH_DMA_BUF_SIZE);
			size_t chunk2;
			uint32_t status = eth_gd32_tx_desc_frame_status(idx, first_idx, remaining);

			if (remaining == 1U) {
				last_idx = idx;
			}

			left -= chunk1;
			chunk2 = MIN(left, (size_t)GD32_ETH_DMA_BUF_SIZE);
			eth_gd32_tx_desc_setup(desc, status, eth_gd32_tx_dma_buf(dev, idx, 0U), chunk1,
					       (chunk2 != 0U) ? eth_gd32_tx_dma_buf(dev, idx, 1U) : NULL,
					       chunk2);
			data->tx_pkt_refs[idx] = NULL;
			left -= chunk2;
			idx = modulo_inc(idx, GD32_ETH_TX_DESC_COUNT);
			remaining--;
		}
	}

	data->tx_pkt_refs[last_idx] = tx_pkt_ref;
	data->tx_next_to_use = idx;
	data->tx_descs_in_use += desc_count;

	barrier_dmem_fence_full();

	idx = modulo_inc(first_idx, GD32_ETH_TX_DESC_COUNT);
	for (uint16_t i = 1U; i < desc_count; i++) {
		eth_gd32_tx_desc_give_to_dma(eth_gd32_tx_desc(dma, idx));
		idx = modulo_inc(idx, GD32_ETH_TX_DESC_COUNT);
	}

	barrier_dmem_fence_full();
	eth_gd32_tx_desc_give_to_dma(eth_gd32_tx_desc(dma, first_idx));
	k_spin_unlock(&data->tx_ring_lock, key);

	eth_gd32_tx_resume_after_publish(cfg);

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

static int eth_gd32_set_config(const struct device *dev, enum ethernet_config_type type,
			       const struct ethernet_config *config)
{
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_data *data = dev->data;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(data->mac_addr, config->mac_address.addr, sizeof(data->mac_addr));
		eth_gd32_mac_address_set(cfg, ENET_MAC_ADDRESS0, data->mac_addr);
		if (data->iface != NULL) {
			net_if_set_link_addr(data->iface, data->mac_addr, sizeof(data->mac_addr),
					     NET_LINK_ETHERNET);
		}
		return 0;
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE: {
		uint32_t frmf = eth_gd32_reg_read(cfg, GD32_ENET_MAC_FRMF_OFFSET);

		if (config->promisc_mode) {
			frmf |= ENET_MAC_FRMF_PM;
		} else {
			frmf &= ~ENET_MAC_FRMF_PM;
		}

		eth_gd32_reg_write(cfg, GD32_ENET_MAC_FRMF_OFFSET, frmf);
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

		eth_gd32_mcast_hash_sync(dev);
		return 0;
#endif
	}
	default:
		return -ENOTSUP;
	}
}

static const struct device *eth_gd32_get_phy(const struct device *dev)
{
	const struct eth_gd32_config *cfg = dev->config;

	return cfg->phy_dev;
}

static bool eth_gd32_mac_is_valid(const uint8_t mac[NET_ETH_ADDR_LEN])
{
	struct net_eth_addr addr;

	memcpy(addr.addr, mac, sizeof(addr.addr));
	return net_eth_is_addr_valid(&addr);
}

static bool eth_gd32_dt_mac_prefix_get(const struct eth_gd32_config *cfg, uint8_t oui[3])
{
	if (!cfg->has_mac_prefix) {
		return false;
	}

	if ((cfg->mac_prefix[0] & 0x01U) != 0U) {
		return false;
	}

	memcpy(oui, cfg->mac_prefix, 3);
	return true;
}

static void eth_gd32_mac_random(uint8_t mac[NET_ETH_ADDR_LEN], const uint8_t *prefix,
				size_t prefix_len)
{
	if (prefix_len > 0U) {
		memcpy(mac, prefix, prefix_len);
	}

	sys_rand_get(&mac[prefix_len], NET_ETH_ADDR_LEN - prefix_len);
	mac[0] |= 0x02U;
	mac[0] &= ~0x01U;
}

static int eth_gd32_mac_from_dt(const struct eth_gd32_config *cfg, uint8_t mac[NET_ETH_ADDR_LEN])
{
	if (!cfg->has_local_mac) {
		return -ENODATA;
	}

	memcpy(mac, cfg->local_mac, NET_ETH_ADDR_LEN);
	return eth_gd32_mac_is_valid(mac) ? 0 : -EINVAL;
}

static int eth_gd32_mac_from_nvmem(const struct eth_gd32_config *cfg, uint8_t mac[NET_ETH_ADDR_LEN])
{
#if defined(CONFIG_NVMEM)
	uint8_t oui[3] = {GD32_OUI_B0, GD32_OUI_B1, GD32_OUI_B2};
	int ret;

	if (!cfg->has_mac_nvmem) {
		return -ENODATA;
	}

	if (!nvmem_cell_is_ready(&cfg->mac_nvmem)) {
		return -ENODEV;
	}

	if (cfg->mac_nvmem.size == NET_ETH_ADDR_LEN) {
		ret = nvmem_cell_read(&cfg->mac_nvmem, mac, 0, NET_ETH_ADDR_LEN);
		if (ret < 0) {
			return ret;
		}

		return eth_gd32_mac_is_valid(mac) ? 0 : -EINVAL;
	}

	if (cfg->mac_nvmem.size == 3U && eth_gd32_dt_mac_prefix_get(cfg, oui)) {
		memcpy(mac, oui, sizeof(oui));
		ret = nvmem_cell_read(&cfg->mac_nvmem, &mac[3], 0, 3U);
		if (ret < 0) {
			return ret;
		}

		return eth_gd32_mac_is_valid(mac) ? 0 : -EINVAL;
	}

	return -EINVAL;
#else
	ARG_UNUSED(cfg);
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
	if (len <= 0) {
		return (len < 0) ? (int)len : -ENODATA;
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
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_data *data = dev->data;
	uint8_t oui[3] = {GD32_OUI_B0, GD32_OUI_B1, GD32_OUI_B2};
	const char *src = "unknown";
	int ret;

	if (cfg->random_mac) {
		if (eth_gd32_dt_mac_prefix_get(cfg, oui)) {
			eth_gd32_mac_random(data->mac_addr, oui, sizeof(oui));
		} else {
			eth_gd32_mac_random(data->mac_addr, NULL, 0U);
		}

		src = "random";
		goto out;
	}

	ret = eth_gd32_mac_from_dt(cfg, data->mac_addr);
	if (ret == 0) {
		src = "dt";
		goto out;
	}

	ret = eth_gd32_mac_from_nvmem(cfg, data->mac_addr);
	if (ret == 0) {
		src = "nvmem";
		goto out;
	}

	ret = eth_gd32_mac_from_hwinfo(oui, data->mac_addr);
	if (ret == 0) {
		src = cfg->has_mac_prefix ? "hwinfo+dt-oui" : "hwinfo+gd-oui";
		goto out;
	}

	eth_gd32_mac_random(data->mac_addr, NULL, 0U);
	src = "random-fallback";

out:
	LOG_INF("MAC %02x:%02x:%02x:%02x:%02x:%02x (%s)", data->mac_addr[0], data->mac_addr[1],
		data->mac_addr[2], data->mac_addr[3], data->mac_addr[4], data->mac_addr[5], src);
}

static void phy_link_state_changed(const struct device *phy_dev, struct phy_link_state *state,
				   void *user_data)
{
	const struct device *dev = user_data;
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_data *data = dev->data;

	ARG_UNUSED(phy_dev);

	atomic_set(&data->link_up, state->is_up ? 1 : 0);

	if (!data->hw_ready) {
		return;
	}

	k_mutex_lock(&data->tx_mutex, K_FOREVER);
	eth_gd32_disable(cfg);
	if (state->is_up) {
		eth_gd32_set_mac_config(dev, state);
		eth_gd32_enable(cfg);
		eth_gd32_rx_poll_demand(cfg);
		if (data->iface != NULL) {
			net_eth_carrier_on(data->iface);
		}
	} else {
		if (data->iface != NULL) {
			net_eth_carrier_off(data->iface);
		}
	}
	k_mutex_unlock(&data->tx_mutex);
}

static void eth_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_data *data = dev->data;

	if (data->iface == NULL) {
		data->iface = iface;
	}

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr), NET_LINK_ETHERNET);
	ethernet_init(iface);
	net_eth_carrier_off(iface);
	net_lldp_set_lldpdu(iface);

	if (cfg->phy_dev == NULL) {
		LOG_WRN("No PHY device, assuming link up");
		atomic_set(&data->link_up, 1);
		eth_gd32_enable(cfg);
		eth_gd32_rx_poll_demand(cfg);
		net_eth_carrier_on(iface);
		return;
	}

	if (!device_is_ready(cfg->phy_dev)) {
		LOG_WRN("PHY device not ready");
		return;
	}

	phy_link_callback_set(cfg->phy_dev, phy_link_state_changed, (void *)dev);

	struct phy_link_state st = {0};

	if (phy_get_link_state(cfg->phy_dev, &st) == 0) {
		phy_link_state_changed(cfg->phy_dev, &st, (void *)dev);
	}
}

static const struct ethernet_api eth_api = {
	.iface_api.init = eth_iface_init,
	.get_capabilities = eth_gd32_get_capabilities,
	.set_config = eth_gd32_set_config,
	.get_phy = eth_gd32_get_phy,
	.send = eth_gd32_tx,
};

static int eth_gd32_hw_init(const struct device *dev)
{
	const struct eth_gd32_config *cfg = dev->config;
	struct eth_gd32_data *data = dev->data;
	int ret;

	data->dev = dev;
	k_mutex_init(&data->tx_mutex);
	k_sem_init(&data->tx_desc_sem, GD32_ETH_TX_DESC_COUNT, GD32_ETH_TX_DESC_COUNT);
	k_work_init(&data->rx_work, eth_gd32_rx_work_handler);
	k_work_init(&data->recover_work, eth_gd32_recover_work_handler);
	atomic_clear(&data->rx_work_pending);
	atomic_clear(&data->rx_resume_pending);
	atomic_clear(&data->recovering);
	atomic_clear(&data->link_up);

	ret = gd32_enet_platform_enable_clocks(&cfg->platform);
	if (ret != 0) {
		return -EIO;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ret = gd32_enet_platform_configure_phy(&cfg->platform);
	if (ret != 0) {
		return ret;
	}

	ret = gd32_enet_platform_reset_mac(&cfg->platform);
	if (ret != 0) {
		return ret;
	}

	if (cfg->irq_config != NULL) {
		cfg->irq_config();
	}

	for (uint16_t i = 0U; i < GD32_ETH_RX_DESC_COUNT; i++) {
		if (((uintptr_t)cfg->rx_dma_buf[i][0] & 0x3U) != 0U ||
		    !gd32_dma_addr_ok((uintptr_t)cfg->rx_dma_buf[i][0], GD32_ETH_DMA_BUF_SIZE)) {
			return -EFAULT;
		}
	}

	for (uint16_t i = 0U; i < GD32_ETH_TX_DESC_COUNT; i++) {
		for (uint8_t buf_idx = 0U; buf_idx < GD32_ETH_TX_DMA_BUFS_PER_DESC; buf_idx++) {
			if (((uintptr_t)cfg->tx_dma_buf[i][buf_idx] & 0x3U) != 0U ||
			    !gd32_dma_addr_ok((uintptr_t)cfg->tx_dma_buf[i][buf_idx],
					      GD32_ETH_DMA_BUF_SIZE)) {
				return -EFAULT;
			}
		}
	}

	if (eth_gd32_hw_reset_and_configure(dev) != 0) {
		return -EIO;
	}

	eth_gd32_disable(cfg);
	eth_gd32_rx_poll_demand(cfg);

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

#define ETH_GD32_RESET_SPEC_BY_NAME(inst, name)                                                    \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_INST_RESET_CTLR_BY_NAME(inst, name)),                      \
		.id = DT_INST_RESET_CELL_BY_NAME(inst, name, id),                                  \
	}

#define ETH_GD32_MAC_PREFIX_INIT(inst)                                                             \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, zephyr_mac_address_prefix),                   \
		    (.has_mac_prefix = true,                                                  \
		     .mac_prefix = DT_INST_PROP(inst, zephyr_mac_address_prefix),),           \
		    (.has_mac_prefix = false,))

#define ETH_GD32_LOCAL_MAC_INIT(inst)                                                              \
	COND_CODE_1(NODE_HAS_VALID_MAC_ADDR(DT_DRV_INST(inst)),                                 \
		    (.has_local_mac = true,                                                   \
		     .local_mac = DT_INST_PROP(inst, local_mac_address),),                   \
		    (.has_local_mac = false,))

#define ETH_GD32_PHY_DEV(inst)                                                                     \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, phy_handle),                                     \
		    (DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(inst, phy_handle))),               \
		    (NULL))

#define ETH_GD32_IRQ_CONFIG(inst)                                                                  \
	static void eth_gd32_irq_config_##inst(void)                                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), eth_gd32_isr,         \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

#define ETH_GD32_INIT(inst)                                                                        \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static struct eth_gd32_dma_rings eth_gd32_dma_##inst __nocache                             \
		__aligned(GD32_ETH_DMA_ALIGN);                                                     \
	static uint8_t eth_gd32_rx_dma_buf_##inst[GD32_ETH_RX_DESC_COUNT]                          \
							 [GD32_ETH_RX_DMA_BUFS_PER_DESC]                   \
							 [GD32_ETH_DMA_BUF_SIZE] __nocache                 \
		__aligned(GD32_ETH_DMA_ALIGN);                                                     \
	static uint8_t eth_gd32_tx_dma_buf_##inst[GD32_ETH_TX_DESC_COUNT]                          \
							 [GD32_ETH_TX_DMA_BUFS_PER_DESC]                   \
							 [GD32_ETH_DMA_BUF_SIZE] __nocache                 \
		__aligned(GD32_ETH_DMA_ALIGN);                                                     \
	ETH_GD32_IRQ_CONFIG(inst)                                                                  \
	static const struct eth_gd32_config eth_gd32_config_##inst = {                             \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.platform = {                                                                      \
			.mac_clk = DT_INST_CLOCKS_CELL_BY_NAME(inst, mac, id),                     \
			.tx_clk = DT_INST_CLOCKS_CELL_BY_NAME(inst, tx, id),                       \
			.rx_clk = DT_INST_CLOCKS_CELL_BY_NAME(inst, rx, id),                       \
			.ptp_clk = COND_CODE_1(DT_INST_CLOCKS_HAS_NAME(inst, ptp),            \
					       (DT_INST_CLOCKS_CELL_BY_NAME(inst, ptp, id)), (0)),                   \
				 .mac_reset = COND_CODE_1(DT_PROP_HAS_NAME(DT_DRV_INST(inst), resets, mac), \
						 (ETH_GD32_RESET_SPEC_BY_NAME(inst, mac)), \
						 ((struct reset_dt_spec){0})),                             \
					   .has_ptp_clk = DT_INST_CLOCKS_HAS_NAME(inst, ptp),      \
					   .has_mac_reset = DT_PROP_HAS_NAME(DT_DRV_INST(inst),    \
									     resets, mac),         \
					   .phy_mode_mii = DT_INST_ENUM_HAS_VALUE(                 \
						   inst, phy_connection_type, mii),                \
					   .phy_clk_internal = DT_INST_ENUM_HAS_VALUE(             \
						   inst, phy_clock_type, internal),                \
					 },                                                        \
					 .phy_dev = ETH_GD32_PHY_DEV(inst),                        \
					 .mac_nvmem = NVMEM_CELL_INST_GET_BY_NAME_OR(              \
						 inst, mac_address, ((struct nvmem_cell){0})),     \
					 .dma = &eth_gd32_dma_##inst,                              \
					 .rx_dma_buf = eth_gd32_rx_dma_buf_##inst,                 \
					 .tx_dma_buf = eth_gd32_tx_dma_buf_##inst,                 \
					 .random_mac =                                             \
						 DT_INST_PROP(inst, zephyr_random_mac_address),    \
					 .has_mac_nvmem =                                          \
						 DT_INST_NVMEM_CELLS_HAS_NAME(inst, mac_address),  \
					 .irq_config = eth_gd32_irq_config_##inst,                 \
					 ETH_GD32_LOCAL_MAC_INIT(inst)                             \
						 ETH_GD32_MAC_PREFIX_INIT(inst)};                  \
	static struct eth_gd32_data eth_gd32_data_##inst;                                          \
	ETH_NET_DEVICE_DT_INST_DEFINE(inst, eth_gd32_init, NULL, &eth_gd32_data_##inst,            \
				      &eth_gd32_config_##inst, CONFIG_ETH_INIT_PRIORITY, &eth_api, \
				      NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_GD32_INIT)
