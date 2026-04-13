/*
 * Copyright (c) 2025 Microchip Technology Inc.
 * Copyright (c) 2026 Ylhyra ehf.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define EEPROM_NODE DT_NODELABEL(eeprom0)
#define EEPROM_I2C_ADDR DT_REG_ADDR(EEPROM_NODE)
#define EEPROM_ADDR_WIDTH_BITS DT_PROP_OR(EEPROM_NODE, address_width, 8)
#define EEPROM_ADDR_WIDTH_BYTES (EEPROM_ADDR_WIDTH_BITS / 8)
#define EEPROM_SIZE DT_PROP(EEPROM_NODE, size)

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(i2c_async_controller))
#define I2C_NODE DT_ALIAS(i2c_async_controller)
#else
#define I2C_NODE DT_PARENT(EEPROM_NODE)
#endif

#define TEST_SMALL_DATA_LEN 8
#define TEST_LONG_DATA_LEN  32
#define TEST_REPEAT_COUNT   16
#define TEST_TIMEOUT_MS     1000

BUILD_ASSERT((EEPROM_ADDR_WIDTH_BITS == 8) || (EEPROM_ADDR_WIDTH_BITS == 16),
	     "Only 8-bit and 16-bit EEPROM address widths are supported by this test");
BUILD_ASSERT(EEPROM_SIZE >= (TEST_LONG_DATA_LEN + 1),
	     "EEPROM must be large enough for the long async transfer test");

static const struct device *const i2c_dev = DEVICE_DT_GET(I2C_NODE);
static const struct device *const eeprom_dev = DEVICE_DT_GET(EEPROM_NODE);

struct async_result {
	struct k_sem sem;
	int status;
	uint32_t callback_count;
};

static void i2c_async_callback(const struct device *dev, int status, void *user_data)
{
	struct async_result *result = user_data;

	ARG_UNUSED(dev);

	result->status = status;
	result->callback_count++;
	k_sem_give(&result->sem);
}

static void async_result_reset(struct async_result *result)
{
	k_sem_reset(&result->sem);
	result->status = INT32_MIN;
	result->callback_count = 0U;
}

static void async_wait_ok(struct async_result *result, const char *what)
{
	zassert_equal(k_sem_take(&result->sem, K_MSEC(TEST_TIMEOUT_MS)), 0,
		      "%s timed out", what);
	zassert_equal(result->status, 0, "%s completed with error %d",
		      what, result->status);
	zassert_equal(result->callback_count, 1U, "%s completed %u times",
		      what, result->callback_count);
}

static void encode_offset(uint16_t offset, uint8_t *buf)
{
	if (EEPROM_ADDR_WIDTH_BYTES == 1U) {
		buf[0] = (uint8_t)offset;
		return;
	}

	buf[0] = (uint8_t)(offset >> 8);
	buf[1] = (uint8_t)offset;
}

static void fill_pattern(uint8_t *buf, size_t len, uint8_t seed)
{
	for (size_t i = 0; i < len; i++) {
		buf[i] = (uint8_t)(seed + (i * 13U));
	}
}

static void async_eeprom_write_read(uint16_t offset, const uint8_t *write_buf, uint8_t *read_buf,
				    size_t len)
{
	uint8_t addr_buf[EEPROM_ADDR_WIDTH_BYTES];
	struct async_result result;
	struct i2c_msg msgs[2];
	int ret;

	zassert_true((offset + len) <= EEPROM_SIZE, "Write/read exceeds EEPROM size");

	k_sem_init(&result.sem, 0, 1);

	encode_offset(offset, addr_buf);
	msgs[0] = (struct i2c_msg){
		.buf = addr_buf,
		.len = sizeof(addr_buf),
		.flags = I2C_MSG_WRITE,
	};
	msgs[1] = (struct i2c_msg){
		.buf = (uint8_t *)write_buf,
		.len = len,
		.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
	};

	async_result_reset(&result);
	ret = i2c_transfer_cb(i2c_dev, msgs, ARRAY_SIZE(msgs), EEPROM_I2C_ADDR,
			      i2c_async_callback, &result);
	zassert_equal(ret, 0, "EEPROM async write failed: %d", ret);
	async_wait_ok(&result, "EEPROM async write");

	msgs[0] = (struct i2c_msg){
		.buf = addr_buf,
		.len = sizeof(addr_buf),
		.flags = I2C_MSG_WRITE,
	};
	msgs[1] = (struct i2c_msg){
		.buf = read_buf,
		.len = len,
		.flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP,
	};

	async_result_reset(&result);
	ret = i2c_transfer_cb(i2c_dev, msgs, ARRAY_SIZE(msgs), EEPROM_I2C_ADDR,
			      i2c_async_callback, &result);
	zassert_equal(ret, 0, "EEPROM async read failed: %d", ret);
	async_wait_ok(&result, "EEPROM async read");
}

static void async_eeprom_single_message_write(uint16_t offset, const uint8_t *write_buf, size_t len)
{
	uint8_t tx_buf[EEPROM_ADDR_WIDTH_BYTES + TEST_LONG_DATA_LEN];
	struct async_result result;
	struct i2c_msg msg;
	int ret;

	zassert_true(len <= TEST_LONG_DATA_LEN, "Write length %zu exceeds test buffer", len);
	zassert_true((offset + len) <= EEPROM_SIZE, "Single-message write exceeds EEPROM size");

	k_sem_init(&result.sem, 0, 1);

	encode_offset(offset, tx_buf);
	memcpy(tx_buf + EEPROM_ADDR_WIDTH_BYTES, write_buf, len);

	msg = (struct i2c_msg){
		.buf = tx_buf,
		.len = EEPROM_ADDR_WIDTH_BYTES + len,
		.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
	};

	async_result_reset(&result);
	ret = i2c_transfer_cb(i2c_dev, &msg, 1, EEPROM_I2C_ADDR,
			      i2c_async_callback, &result);
	zassert_equal(ret, 0, "EEPROM async single-message write failed: %d", ret);
	async_wait_ok(&result, "EEPROM async single-message write");
}

static void async_eeprom_single_message_read(uint16_t offset, uint8_t *read_buf, size_t len)
{
	uint8_t addr_buf[EEPROM_ADDR_WIDTH_BYTES];
	struct async_result result;
	struct i2c_msg msg;
	int ret;

	zassert_true((offset + len) <= EEPROM_SIZE, "Single-message read exceeds EEPROM size");

	k_sem_init(&result.sem, 0, 1);

	encode_offset(offset, addr_buf);
	ret = i2c_write(i2c_dev, addr_buf, sizeof(addr_buf), EEPROM_I2C_ADDR);
	zassert_equal(ret, 0, "EEPROM offset write failed: %d", ret);

	msg = (struct i2c_msg){
		.buf = read_buf,
		.len = len,
		.flags = I2C_MSG_READ | I2C_MSG_STOP,
	};

	async_result_reset(&result);
	ret = i2c_transfer_cb(i2c_dev, &msg, 1, EEPROM_I2C_ADDR,
			      i2c_async_callback, &result);
	zassert_equal(ret, 0, "EEPROM async single-message read failed: %d", ret);
	async_wait_ok(&result, "EEPROM async single-message read");
}

ZTEST(i2c_async, test_eeprom_async_basic)
{
	uint8_t write_data[TEST_SMALL_DATA_LEN];
	uint8_t read_data[TEST_SMALL_DATA_LEN] = {0};

	fill_pattern(write_data, sizeof(write_data), 0x10);
	async_eeprom_write_read(1U, write_data, read_data, sizeof(write_data));
	zassert_mem_equal(read_data, write_data, sizeof(write_data),
			  "Basic async write/read returned unexpected data");
}

ZTEST(i2c_async, test_eeprom_async_single_message)
{
	uint8_t write_data[TEST_LONG_DATA_LEN];
	uint8_t read_data[TEST_LONG_DATA_LEN] = {0};

	fill_pattern(write_data, sizeof(write_data), 0x40);
	async_eeprom_single_message_write(16U, write_data, sizeof(write_data));
	async_eeprom_single_message_read(16U, read_data, sizeof(read_data));
	zassert_mem_equal(read_data, write_data, sizeof(write_data),
			  "Single-message async access returned unexpected data");
}

ZTEST(i2c_async, test_eeprom_async_repeated)
{
	uint8_t write_data[TEST_SMALL_DATA_LEN];
	uint8_t read_data[TEST_SMALL_DATA_LEN];

	for (uint16_t round = 0U; round < TEST_REPEAT_COUNT; round++) {
		uint16_t offset = (uint16_t)((round * 7U) % (EEPROM_SIZE - TEST_SMALL_DATA_LEN));

		fill_pattern(write_data, sizeof(write_data), (uint8_t)(0x80U + round));
		memset(read_data, 0, sizeof(read_data));

		async_eeprom_write_read(offset, write_data, read_data, sizeof(write_data));
		zassert_mem_equal(read_data, write_data, sizeof(write_data),
				  "Repeated async write/read mismatch on round %u", round);
	}
}

static void *i2c_test_setup(void)
{
	zassert_true(device_is_ready(i2c_dev), "I2C controller is not ready");
	zassert_true(device_is_ready(eeprom_dev), "I2C EEPROM target is not ready");
	zassert_equal(i2c_target_driver_register(eeprom_dev), 0,
		      "Failed to register EEPROM target");

	return NULL;
}

static void i2c_test_teardown(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_equal(i2c_target_driver_unregister(eeprom_dev), 0,
		      "Failed to unregister EEPROM target");
}

ZTEST_SUITE(i2c_async, NULL, i2c_test_setup, NULL, NULL, i2c_test_teardown);
