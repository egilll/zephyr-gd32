/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>

#define SAMPLE_NO 64

#if defined(CONFIG_BOARD_GD32F450Z_EVAL)
#define SAMPLE_OUTPUT_FORMAT (I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED | I2S_FMT_BIT_CLK_INV)
#define SAMPLE_CONTINUOUS_OUTPUT 1
#define SAMPLE_ATTENUATION 0
#else
#define SAMPLE_OUTPUT_FORMAT I2S_FMT_DATA_FORMAT_I2S
#define SAMPLE_CONTINUOUS_OUTPUT 0
#define SAMPLE_ATTENUATION 1
#endif

/* The data represent a sine wave */
static int16_t data[SAMPLE_NO] = {
	  3211,   6392,   9511,  12539,  15446,  18204,  20787,  23169,
	 25329,  27244,  28897,  30272,  31356,  32137,  32609,  32767,
	 32609,  32137,  31356,  30272,  28897,  27244,  25329,  23169,
	 20787,  18204,  15446,  12539,   9511,   6392,   3211,      0,
	 -3212,  -6393,  -9512, -12540, -15447, -18205, -20788, -23170,
	-25330, -27245, -28898, -30273, -31357, -32138, -32610, -32767,
	-32610, -32138, -31357, -30273, -28898, -27245, -25330, -23170,
	-20788, -18205, -15447, -12540,  -9512,  -6393,  -3212,     -1,
};

/* Fill buffer with sine wave on left channel, and sine wave shifted by
 * 90 degrees on right channel. "att" represents a power of two to attenuate
 * the samples by
 */
static void fill_buf(int16_t *tx_block, int att)
{
	int r_idx;

	for (int i = 0; i < SAMPLE_NO; i++) {
		/* Left channel is sine wave */
		tx_block[2 * i] = data[i] / (1 << att);
		/* Right channel is same sine wave, shifted by 90 degrees */
		r_idx = (i + (ARRAY_SIZE(data) / 4)) % ARRAY_SIZE(data);
		tx_block[2 * i + 1] = data[r_idx] / (1 << att);
	}
}

#define NUM_BLOCKS 20
#define BLOCK_SIZE (2 * sizeof(data))
#define PRIME_BLOCKS 4
#define DEMO_DURATION_MS 3000
#define DEMO_BLOCK_COUNT \
	((DEMO_DURATION_MS * i2s_cfg.frame_clk_freq + (SAMPLE_NO * 1000U) - 1U) / \
	 (SAMPLE_NO * 1000U))

#ifdef CONFIG_NOCACHE_MEMORY
	#define MEM_SLAB_CACHE_ATTR __nocache
#else
	#define MEM_SLAB_CACHE_ATTR
#endif /* CONFIG_NOCACHE_MEMORY */

static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(32))
	_k_mem_slab_buf_tx_0_mem_slab[(NUM_BLOCKS) * WB_UP(BLOCK_SIZE)];

static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_0_mem_slab) =
	Z_MEM_SLAB_INITIALIZER(tx_0_mem_slab, _k_mem_slab_buf_tx_0_mem_slab,
				WB_UP(BLOCK_SIZE), NUM_BLOCKS);

int main(void)
{
	struct i2s_config i2s_cfg;
	int16_t tx_block[2 * SAMPLE_NO];
	int ret;
	uint32_t tx_idx;
	uint32_t total_blocks;
	const struct device *dev_i2s = DEVICE_DT_GET(DT_ALIAS(i2s_tx));

	if (!device_is_ready(dev_i2s)) {
		printf("I2S device not ready\n");
		return -ENODEV;
	}
	/* Configure I2S stream */
	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = SAMPLE_OUTPUT_FORMAT;
	i2s_cfg.frame_clk_freq = 44100;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = 2000;
	/* Configure the Transmit port as Master */
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER
			| I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &tx_0_mem_slab;
	ret = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);
	if (ret < 0) {
		printf("Failed to configure I2S stream\n");
		return ret;
	}

	fill_buf(tx_block, SAMPLE_ATTENUATION);
	total_blocks = DEMO_BLOCK_COUNT;

	for (tx_idx = 0; tx_idx < MIN(PRIME_BLOCKS, total_blocks); tx_idx++) {
		ret = i2s_buf_write(dev_i2s, tx_block, BLOCK_SIZE);
		if (ret < 0) {
			printf("Could not prime TX buffer %u\n", tx_idx);
			return ret;
		}
	}

	/* Trigger the I2S transmission */
	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printf("Could not trigger I2S tx\n");
		return ret;
	}

	if (SAMPLE_CONTINUOUS_OUTPUT) {
		printf("Continuous I2S tone running\n");

		for (;;) {
			ret = i2s_buf_write(dev_i2s, tx_block, BLOCK_SIZE);
			if (ret < 0) {
				printf("Could not write TX buffer %u\n", tx_idx);
				return ret;
			}
			tx_idx++;
		}
	}

	for (; tx_idx < total_blocks; tx_idx++) {
		ret = i2s_buf_write(dev_i2s, tx_block, BLOCK_SIZE);
		if (ret < 0) {
			printf("Could not write TX buffer %u\n", tx_idx);
			return ret;
		}
	}
	/* Drain TX queue */
	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		printf("Could not trigger I2S tx\n");
		return ret;
	}
	printf("All I2S blocks written\n");
	return 0;
}
