/*
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I2S_I2S_GD32_BANKS_H_
#define ZEPHYR_DRIVERS_I2S_I2S_GD32_BANKS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * The blocks held by the two buffers ("banks") of a switch-buffer DMA channel. The DMA moves to
 * the other bank at the end of each block; the driver points the bank it left at the next block.
 *
 * A transmit bank with no block of its own reads the other bank's block, so the DMA only ever
 * reads memory the driver owns. Reaching such a bank means the stream has no audio left.
 */
struct gd32_i2s_banks {
	void *block[2];
	/* The bank the DMA is in */
	uint8_t active;
};

static inline void gd32_i2s_banks_init(struct gd32_i2s_banks *banks, void *first, void *second)
{
	banks->block[0] = first;
	banks->block[1] = second;
	banks->active = 0U;
}

static inline uint8_t gd32_i2s_banks_next(const struct gd32_i2s_banks *banks)
{
	return banks->active ^ 1U;
}

/* The memory the DMA reads or writes in `bank`. */
static inline void *gd32_i2s_banks_memory(const struct gd32_i2s_banks *banks, uint8_t bank)
{
	return banks->block[bank] != NULL ? banks->block[bank] : banks->block[bank ^ 1U];
}

/*
 * The DMA finished the active bank and moved to the other. Returns the block it finished, which
 * that bank, now the next one, no longer holds. Its memory stays in use until the next bank
 * points elsewhere.
 */
static inline void *gd32_i2s_banks_complete(struct gd32_i2s_banks *banks)
{
	const uint8_t done = banks->active;
	void *block = banks->block[done];

	banks->block[done] = NULL;
	banks->active = done ^ 1U;
	return block;
}

/* Whether the bank the DMA is in holds a block of its own. */
static inline bool gd32_i2s_banks_active_held(const struct gd32_i2s_banks *banks)
{
	return banks->block[banks->active] != NULL;
}

static inline bool gd32_i2s_banks_next_held(const struct gd32_i2s_banks *banks)
{
	return banks->block[gd32_i2s_banks_next(banks)] != NULL;
}

/* Gives the next bank `block`, or none. Returns the memory the DMA is to use there. */
static inline void *gd32_i2s_banks_fill_next(struct gd32_i2s_banks *banks, void *block)
{
	const uint8_t next = gd32_i2s_banks_next(banks);

	banks->block[next] = block;
	return gd32_i2s_banks_memory(banks, next);
}

/* Takes every block the banks hold, for releasing once the DMA has stopped. */
static inline size_t gd32_i2s_banks_take_all(struct gd32_i2s_banks *banks, void *blocks[2])
{
	size_t count = 0U;

	for (size_t bank = 0U; bank < 2U; bank++) {
		if (banks->block[bank] != NULL) {
			blocks[count++] = banks->block[bank];
			banks->block[bank] = NULL;
		}
	}

	return count;
}

#endif /* ZEPHYR_DRIVERS_I2S_I2S_GD32_BANKS_H_ */
