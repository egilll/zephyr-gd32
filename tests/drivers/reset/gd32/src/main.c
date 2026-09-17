/*
 * Copyright (c) 2026, Ylhyra ehf.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#define TEST_REGISTER_COUNT 32U
#define TEST_RACE_ROUNDS    128U

static volatile uint32_t registers[TEST_REGISTER_COUNT];
static unsigned int reads;
static unsigned int writes;
static unsigned int sequence;
static unsigned int read_sequence[2];
static unsigned int write_sequence[2];
static uint32_t write_values[2];
static bool stress_updates;

static uint32_t test_read32(uintptr_t addr)
{
	zassert_true(addr >= 0x1000 && addr < 0x1080);

	if (stress_updates) {
		for (unsigned int i = 0; i < 256U; i++) {
			arch_nop();
		}
	} else if (reads < ARRAY_SIZE(read_sequence)) {
		read_sequence[reads] = ++sequence;
		reads++;
	}

	return registers[(addr - 0x1000) / sizeof(uint32_t)];
}

static void test_write32(uint32_t value, uintptr_t addr)
{
	zassert_true(addr >= 0x1000 && addr < 0x1080);

	if (!stress_updates) {
		if (writes < ARRAY_SIZE(write_sequence)) {
			write_sequence[writes] = ++sequence;
			write_values[writes] = value;
		}
		writes++;
	}

	registers[(addr - 0x1000) / sizeof(uint32_t)] = value;
}

#define sys_read32  test_read32
#define sys_write32 test_write32
#include "reset_gd32.c"

static const struct device *const rctl = DEVICE_DT_GET(DT_NODELABEL(rctl));

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	memset((void *)registers, 0, sizeof(registers));
	reads = 0U;
	writes = 0U;
	sequence = 0U;
	memset(read_sequence, 0, sizeof(read_sequence));
	memset(write_sequence, 0, sizeof(write_sequence));
	memset(write_values, 0, sizeof(write_values));
	stress_updates = false;
}

ZTEST(reset_gd32, test_invalid_ids_do_not_access_mmio)
{
	static const uint32_t invalid_ids[] = {
		BIT(5), 2U << 6, BIT(14), UINT32_MAX, 0x84U << 6, 0xfcU << 6,
	};
	uint8_t status;

	zassert_true(device_is_ready(rctl));

	for (size_t i = 0U; i < ARRAY_SIZE(invalid_ids); i++) {
		zassert_equal(reset_line_assert(rctl, invalid_ids[i]), -EINVAL);
		zassert_equal(reset_line_deassert(rctl, invalid_ids[i]), -EINVAL);
		zassert_equal(reset_line_toggle(rctl, invalid_ids[i]), -EINVAL);
		zassert_equal(reset_status(rctl, invalid_ids[i], &status), -EINVAL);
	}
	zassert_equal(reads, 0U);
	zassert_equal(writes, 0U);
}

ZTEST(reset_gd32, test_valid_operations)
{
	const uint32_t id = (0x24U << 6) | 8U;
	uint8_t status;

	registers[0x24 / sizeof(uint32_t)] = BIT(4);
	zassert_ok(reset_line_assert(rctl, id));
	zassert_equal(registers[0x24 / sizeof(uint32_t)], BIT(4) | BIT(8));
	zassert_ok(reset_status(rctl, id, &status));
	zassert_equal(status, 1U);
	zassert_ok(reset_line_deassert(rctl, id));
	zassert_equal(registers[0x24 / sizeof(uint32_t)], BIT(4));
	zassert_ok(reset_status(rctl, id, &status));
	zassert_equal(status, 0U);
}

ZTEST(reset_gd32, test_toggle_reads_back_assertion)
{
	const uint32_t id = (0x24U << 6) | 8U;

	registers[0x24 / sizeof(uint32_t)] = BIT(4);
	zassert_ok(reset_line_toggle(rctl, id));
	zassert_equal(registers[0x24 / sizeof(uint32_t)], BIT(4));
	zassert_equal(reads, 2U);
	zassert_equal(writes, 2U);
	zassert_equal(read_sequence[0], 1U);
	zassert_equal(write_sequence[0], 2U);
	zassert_equal(read_sequence[1], 3U);
	zassert_equal(write_sequence[1], 4U);
	zassert_equal(write_values[0], BIT(4) | BIT(8));
	zassert_equal(write_values[1], BIT(4));
}

ZTEST(reset_gd32, test_null_status)
{
	zassert_equal(reset_status(rctl, 0U, NULL), -EINVAL);
}

#ifdef CONFIG_SMP
struct contender {
	struct k_sem start;
	uint32_t id;
};

static struct contender contenders[] = {
	{.id = (0x24U << 6) | 8U},
	{.id = (0x24U << 6) | 9U},
};

static K_SEM_DEFINE(done, 0, ARRAY_SIZE(contenders));
static K_THREAD_STACK_ARRAY_DEFINE(stacks, ARRAY_SIZE(contenders), 1024);
static struct k_thread threads[ARRAY_SIZE(contenders)];

static void update_contender(void *arg, void *unused1, void *unused2)
{
	struct contender *contender = arg;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	for (unsigned int round = 0; round < TEST_RACE_ROUNDS; round++) {
		zassert_ok(k_sem_take(&contender->start, K_SECONDS(1)));
		zassert_ok(reset_line_assert(rctl, contender->id));
		k_sem_give(&done);
	}
}
#endif

ZTEST(reset_gd32, test_concurrent_updates_preserve_both_bits)
{
#ifdef CONFIG_SMP
	const uint32_t expected = BIT(8) | BIT(9);

	stress_updates = true;
	for (size_t i = 0; i < ARRAY_SIZE(contenders); i++) {
		k_sem_init(&contenders[i].start, 0, 1);
		k_thread_create(&threads[i], stacks[i], K_THREAD_STACK_SIZEOF(stacks[i]),
				update_contender, &contenders[i], NULL, NULL, K_PRIO_PREEMPT(1), 0,
				K_FOREVER);
		zassert_ok(k_thread_cpu_pin(&threads[i], i));
		k_thread_start(&threads[i]);
	}

	for (unsigned int round = 0; round < TEST_RACE_ROUNDS; round++) {
		registers[0x24 / sizeof(uint32_t)] = 0U;
		for (size_t i = 0; i < ARRAY_SIZE(contenders); i++) {
			k_sem_give(&contenders[i].start);
		}
		for (size_t i = 0; i < ARRAY_SIZE(contenders); i++) {
			zassert_ok(k_sem_take(&done, K_SECONDS(1)));
		}
		zassert_equal(registers[0x24 / sizeof(uint32_t)], expected);
	}

	for (size_t i = 0; i < ARRAY_SIZE(threads); i++) {
		zassert_ok(k_thread_join(&threads[i], K_SECONDS(1)));
	}
#else
	ztest_test_skip();
#endif
}

ZTEST_SUITE(reset_gd32, NULL, NULL, before, NULL, NULL);
