/*
 * Copyright (c) 2017 BayLibre, SAS
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/target/eeprom.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/ztest.h>

#define NODE_EP0 DT_NODELABEL(eeprom0)
#define NODE_EP1 DT_NODELABEL(eeprom1)

#define TEST_DATA_SIZE	MIN(CONFIG_I2C_TEST_DATA_MAX_SIZE, \
			    MIN(DT_PROP(NODE_EP0, size), DT_PROP(NODE_EP1, size)))

static uint8_t eeprom_0_data[TEST_DATA_SIZE];
static uint8_t eeprom_1_data[TEST_DATA_SIZE];
static uint8_t expected_0_data[TEST_DATA_SIZE];
static uint8_t expected_1_data[TEST_DATA_SIZE];
static uint8_t i2c_buffer[TEST_DATA_SIZE];

struct test_i2c_target {
	struct i2c_target_config config;
	uint8_t data[32];
	uint8_t offset;
	uint8_t addr_bytes_seen;
	uint8_t data_bytes_seen;
	uint8_t nack_after_data;
	bool nack_on_write_received;
};

/*
 * We need 5x(buffer size) + 1 to print a comma-separated list of each
 * byte in hex, plus a null.
 */
uint8_t buffer_print_eeprom[TEST_DATA_SIZE * 5 + 1];
uint8_t buffer_print_i2c[TEST_DATA_SIZE * 5 + 1];

static const size_t transfer_boundary_lengths[] = {
	1U, 2U, 3U, 4U, 7U, 8U, 15U, 16U, 17U, 31U, 32U, 63U, 64U, 127U, 128U, 255U, 256U,
};

static void init_eeprom_test_data(void)
{
	size_t n;

	/*
	 * Initialize EEPROM data with printable ASCII value (range [32 126]).
	 * Make sure content differs between eeprom_0_data[] and eeprom_1_data[].
	 */
	for (n = 0; n < sizeof(eeprom_0_data); n++) {
		eeprom_0_data[n] = 32 + (n % (126 - 32));
	}

	for (n = 0; n < sizeof(eeprom_1_data); n++) {
		eeprom_1_data[n] = 32 + (((n + 10) * 3) % (126 - 32));
	}
}

static void to_display_format(const uint8_t *src, size_t size, char *dst)
{
	size_t i;

	for (i = 0; i < size; i++) {
		sprintf(dst + 5 * i, "0x%02x,", src[i]);
	}
}

static int compare_buffer(const uint8_t *actual, const uint8_t *expected, size_t len)
{
	if (memcmp(actual, expected, len) == 0) {
		return 0;
	}

	to_display_format(actual, len, buffer_print_i2c);
	to_display_format(expected, len, buffer_print_eeprom);
	TC_PRINT("Error: Buffer contents are different: %s\n", buffer_print_i2c);
	TC_PRINT("                         vs expected: %s\n", buffer_print_eeprom);

	return -EIO;
}

static bool should_run_length(size_t len)
{
	return (len > 0U) && (len <= TEST_DATA_SIZE);
}

static bool should_run_offset(unsigned int offset)
{
	static const unsigned int offset_boundaries[] = {
		0U, 1U, 2U, 3U, 4U, 7U, 8U, 15U, 16U, 31U, 32U, 63U, 64U, 127U, 128U, 255U,
	};

	if (offset >= TEST_DATA_SIZE) {
		return false;
	}

	if (TEST_DATA_SIZE <= 32U) {
		return true;
	}

	if (offset == (TEST_DATA_SIZE - 1U)) {
		return true;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(offset_boundaries); i++) {
		if (offset == offset_boundaries[i]) {
			return true;
		}
	}

	return false;
}

static int run_full_read(const struct device *i2c, uint8_t addr,
			 uint8_t addr_width, const uint8_t *comp_buffer)
{
	int ret;
	uint8_t start_addr[2];

	TC_PRINT("Testing full read: Master: %s, address: 0x%x\n",
		 i2c->name, addr);

	/* Read EEPROM from I2C Master requests, then compare */
	memset(start_addr, 0, sizeof(start_addr));
	ret = i2c_write_read(i2c, addr, start_addr, (addr_width >> 3), i2c_buffer, TEST_DATA_SIZE);
	zassert_equal(ret, 0, "Failed to read EEPROM");

	return compare_buffer(i2c_buffer, comp_buffer, TEST_DATA_SIZE);
}

static int run_partial_read(const struct device *i2c, uint8_t addr,
			    uint8_t addr_width, const uint8_t *comp_buffer, unsigned int offset)
{
	int ret;
	uint8_t start_addr[2];

	TC_PRINT("Testing partial read. Master: %s, address: 0x%x, off=%d\n",
		 i2c->name, addr, offset);

	switch (addr_width) {
	case 8:
		start_addr[0] = (uint8_t) (offset & 0xFF);
	break;
	case 16:
		sys_put_be16((uint16_t)(offset & 0xFFFF), start_addr);
	break;
	default:
		return -EINVAL;
	}

	ret = i2c_write_read(i2c, addr,
			     start_addr, (addr_width >> 3), i2c_buffer, TEST_DATA_SIZE-offset);
	zassert_equal(ret, 0, "Failed to read EEPROM");

	return compare_buffer(i2c_buffer, &comp_buffer[offset], TEST_DATA_SIZE - offset);
}

static int set_read_offset(const struct device *i2c, uint8_t addr,
			   uint8_t addr_width, unsigned int offset)
{
	uint8_t start_addr[2];

	switch (addr_width) {
	case 8:
		start_addr[0] = (uint8_t)(offset & 0xFF);
		break;
	case 16:
		sys_put_be16((uint16_t)(offset & 0xFFFF), start_addr);
		break;
	default:
		return -EINVAL;
	}

	return i2c_write(i2c, start_addr, (addr_width >> 3), addr);
}

static int run_direct_read(const struct device *i2c, uint8_t addr,
			   uint8_t addr_width, const uint8_t *comp_buffer,
			   unsigned int offset, size_t len)
{
	int ret;

	TC_PRINT("Testing direct read. Master: %s, address: 0x%x, off=%d, len=%zu\n",
		 i2c->name, addr, offset, len);

	ret = set_read_offset(i2c, addr, addr_width, offset);
	zassert_equal(ret, 0, "Failed to set EEPROM read offset");

	ret = i2c_read(i2c, i2c_buffer, len, addr);
	zassert_equal(ret, 0, "Failed to perform direct EEPROM read");

	return compare_buffer(i2c_buffer, &comp_buffer[offset], len);
}

static int run_split_read(const struct device *i2c, uint8_t addr,
			  uint8_t addr_width, const uint8_t *comp_buffer,
			  unsigned int offset, size_t first_len, size_t second_len)
{
	int ret;
	uint8_t first_buf[TEST_DATA_SIZE];
	uint8_t second_buf[TEST_DATA_SIZE];
	struct i2c_msg msgs[2] = {
		{
			.buf = first_buf,
			.len = first_len,
			.flags = I2C_MSG_READ,
		},
		{
			.buf = second_buf,
			.len = second_len,
			.flags = I2C_MSG_READ | I2C_MSG_STOP,
		},
	};

	TC_PRINT("Testing split direct read. Master: %s, address: 0x%x, off=%d, lens=%zu/%zu\n",
		 i2c->name, addr, offset, first_len, second_len);

	ret = set_read_offset(i2c, addr, addr_width, offset);
	zassert_equal(ret, 0, "Failed to set EEPROM read offset");

	ret = i2c_transfer(i2c, msgs, ARRAY_SIZE(msgs), addr);
	zassert_equal(ret, 0, "Failed split direct I2C read");

	ret = compare_buffer(first_buf, &comp_buffer[offset], first_len);
	if (ret != 0) {
		return ret;
	}

	return compare_buffer(second_buf, &comp_buffer[offset + first_len], second_len);
}

static int run_combined_split_read(const struct device *i2c, uint8_t addr,
				   uint8_t addr_width, const uint8_t *comp_buffer,
				   unsigned int offset, size_t first_len, size_t second_len)
{
	int ret;
	uint8_t first_buf[TEST_DATA_SIZE];
	uint8_t second_buf[TEST_DATA_SIZE];
	uint8_t start_addr[2];
	struct i2c_msg msgs[3] = {
		{
			.buf = start_addr,
			.len = (uint8_t)(addr_width >> 3),
			.flags = I2C_MSG_WRITE,
		},
		{
			.buf = first_buf,
			.len = first_len,
			.flags = I2C_MSG_READ | I2C_MSG_RESTART,
		},
		{
			.buf = second_buf,
			.len = second_len,
			.flags = I2C_MSG_READ | I2C_MSG_STOP,
		},
	};

	TC_PRINT("Testing combined split read. Master: %s, address: 0x%x, off=%d, lens=%zu/%zu\n",
		 i2c->name, addr, offset, first_len, second_len);

	switch (addr_width) {
	case 8:
		start_addr[0] = (uint8_t)(offset & 0xFF);
		break;
	case 16:
		sys_put_be16((uint16_t)(offset & 0xFFFF), start_addr);
		break;
	default:
		return -EINVAL;
	}

	ret = i2c_transfer(i2c, msgs, ARRAY_SIZE(msgs), addr);
	zassert_equal(ret, 0, "Failed combined split I2C read");

	ret = compare_buffer(first_buf, &comp_buffer[offset], first_len);
	if (ret != 0) {
		return ret;
	}

	return compare_buffer(second_buf, &comp_buffer[offset + first_len], second_len);
}

static int run_split_write_read(const struct device *i2c, uint8_t addr,
				uint8_t addr_width, uint8_t *comp_buffer,
				unsigned int offset, size_t len)
{
	int ret;
	uint8_t addr_buf[2];
	uint8_t payload[TEST_DATA_SIZE];
	struct i2c_msg msgs[2] = {
		{
			.buf = addr_buf,
			.len = (uint8_t)(addr_width >> 3),
			.flags = I2C_MSG_WRITE,
		},
		{
			.buf = payload,
			.len = len,
			.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
		},
	};

	TC_PRINT("Testing split write. Master: %s, address: 0x%x, off=%d, len=%zu\n",
		 i2c->name, addr, offset, len);

	switch (addr_width) {
	case 8:
		addr_buf[0] = (uint8_t)(offset & 0xFF);
		break;
	case 16:
		sys_put_be16((uint16_t)(offset & 0xFFFF), addr_buf);
		break;
	default:
		return -EINVAL;
	}

	for (size_t i = 0U; i < len; i++) {
		payload[i] = (uint8_t)(((offset + i) * 7U) + 3U);
		comp_buffer[offset + i] = payload[i];
	}

	ret = i2c_transfer(i2c, msgs, ARRAY_SIZE(msgs), addr);
	zassert_equal(ret, 0, "Failed split write");

	return run_direct_read(i2c, addr, addr_width, comp_buffer, offset, len);
}

static int run_address_nack_recovery(const struct device *i2c, uint8_t addr,
				     uint8_t addr_width, const uint8_t *comp_buffer,
				     uint8_t alt_addr)
{
	uint8_t start_addr[2] = { 0U, 0U };
	uint8_t scratch;
	uint8_t nack_addr = 0U;
	int ret;

	for (uint8_t candidate = 0x08U; candidate < 0x78U; candidate++) {
		if ((candidate != addr) && (candidate != alt_addr)) {
			nack_addr = candidate;
			break;
		}
	}

	zassert_not_equal(nack_addr, 0U, "No unused 7-bit address available for NACK test");

	ret = i2c_write_read(i2c, nack_addr, start_addr, (addr_width >> 3), &scratch, 1U);
	zassert_not_equal(ret, 0, "Unexpected ACK on unused address 0x%x", nack_addr);

	return run_full_read(i2c, addr, addr_width, comp_buffer);
}

static int unregister_optional_target(const struct device *target)
{
	if (target == NULL) {
		return 0;
	}

	return i2c_target_driver_unregister(target);
}

static int register_optional_target(const struct device *target)
{
	if (target == NULL) {
		return 0;
	}

	return i2c_target_driver_register(target);
}

static int run_gpio_fault_recovery(const struct device *controller, const struct device *peer_bus,
				   const struct device *controller_target,
				   const struct device *peer_target,
				   const struct gpio_dt_spec *peer_sda,
				   const struct gpio_dt_spec *peer_scl,
				   bool hold_sda, uint8_t addr, uint8_t addr_width,
				   const uint8_t *comp_buffer)
{
	const struct gpio_dt_spec *held_pin = hold_sda ? peer_sda : peer_scl;
	const struct gpio_dt_spec *released_pin = hold_sda ? peer_scl : peer_sda;
	const char *line_name = hold_sda ? "SDA" : "SCL";
	bool controller_target_unregistered = false;
	bool peer_target_unregistered = false;
	bool peer_bus_deinitialized = false;
	int ret;

	if ((peer_target == NULL) || (held_pin->port == NULL) || (released_pin->port == NULL)) {
		return 0;
	}

	if (!device_is_ready(held_pin->port) || !device_is_ready(released_pin->port)) {
		return 0;
	}

	TC_PRINT("Testing %s-low bus fault recovery. Master: %s, fault bus: %s\n",
		 line_name, controller->name, peer_bus->name);

	ret = unregister_optional_target(controller_target);
	zassert_equal(ret, 0, "Failed to unregister controller target before %s fault", line_name);
	controller_target_unregistered = (controller_target != NULL);

	ret = unregister_optional_target(peer_target);
	zassert_equal(ret, 0, "Failed to unregister peer target before %s fault", line_name);
	peer_target_unregistered = true;

	ret = device_deinit(peer_bus);
	if (ret == -ENOTSUP) {
		TC_PRINT("  device deinit not supported on %s, skipping %s-low fault test\n",
			 peer_bus->name, line_name);
		goto restore_targets;
	}

	zassert_equal(ret, 0, "Failed to deinit %s for %s-low fault", peer_bus->name, line_name);
	peer_bus_deinitialized = true;

	zassert_ok(gpio_pin_configure_dt(released_pin, GPIO_INPUT));
	zassert_ok(gpio_pin_configure_dt(held_pin, GPIO_OUTPUT_LOW | GPIO_OPEN_DRAIN));
	zassert_equal(gpio_pin_get_dt(held_pin), 0, "%s line did not latch low", line_name);

	ret = i2c_recover_bus(controller);
	if ((ret == -ENOSYS) || (ret == -ENOTSUP)) {
		TC_PRINT("  recover_bus not supported on %s, skipping %s-low recovery check\n",
			 controller->name, line_name);
	} else if (ret == 0) {
		TC_PRINT("  recover_bus reported success while %s was externally held low\n",
			 line_name);
	} else {
		TC_PRINT("  recover_bus reported %d with %s held low\n", ret, line_name);
	}

	zassert_ok(gpio_pin_configure_dt(held_pin, GPIO_INPUT));

	if (peer_bus_deinitialized) {
		zassert_ok(device_init(peer_bus));
	}

restore_targets:
	if (peer_target_unregistered) {
		zassert_ok(register_optional_target(peer_target));
	}

	ret = i2c_recover_bus(controller);
	if ((ret != -ENOSYS) && (ret != -ENOTSUP)) {
		zassert_equal(ret, 0, "Failed recovery after releasing %s", line_name);
	}

	if (controller_target_unregistered) {
		zassert_ok(register_optional_target(controller_target));
	}

	return run_full_read(controller, addr, addr_width, comp_buffer);
}

static int run_boundary_direct_reads(const struct device *i2c, uint8_t addr,
				     uint8_t addr_width, const uint8_t *comp_buffer)
{
	size_t last_len = 0U;

	for (size_t i = 0U; i < ARRAY_SIZE(transfer_boundary_lengths); i++) {
		size_t len = transfer_boundary_lengths[i];
		int ret;

		if (!should_run_length(len) || (len == last_len)) {
			continue;
		}

		last_len = len;

		ret = run_direct_read(i2c, addr, addr_width, comp_buffer, 0U, len);
		if (ret != 0) {
			return ret;
		}

		ret = run_direct_read(i2c, addr, addr_width, comp_buffer, TEST_DATA_SIZE - len, len);
		if (ret != 0) {
			return ret;
		}
	}

	if (last_len != TEST_DATA_SIZE) {
		return run_direct_read(i2c, addr, addr_width, comp_buffer, 0U, TEST_DATA_SIZE);
	}

	return 0;
}

static int run_boundary_split_reads(const struct device *i2c, uint8_t addr,
				    uint8_t addr_width, const uint8_t *comp_buffer)
{
	for (size_t i = 0U; i < ARRAY_SIZE(transfer_boundary_lengths); i++) {
		size_t len = transfer_boundary_lengths[i];
		size_t first_len;
		size_t second_len;
		unsigned int offset;
		int ret;

		if (!should_run_length(len) || (len < 2U)) {
			continue;
		}

		first_len = len / 2U;
		second_len = len - first_len;
		offset = TEST_DATA_SIZE - len;

		ret = run_split_read(i2c, addr, addr_width, comp_buffer, offset, first_len, second_len);
		if (ret != 0) {
			return ret;
		}

		ret = run_combined_split_read(i2c, addr, addr_width, comp_buffer, offset,
					      first_len, second_len);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

static int run_boundary_split_writes(const struct device *i2c, uint8_t addr,
				     uint8_t addr_width, uint8_t *comp_buffer)
{
	for (size_t i = 0U; i < ARRAY_SIZE(transfer_boundary_lengths); i++) {
		size_t len = transfer_boundary_lengths[i];
		unsigned int offset;
		int ret;

		if (!should_run_length(len)) {
			continue;
		}

		offset = TEST_DATA_SIZE - len;
		ret = run_split_write_read(i2c, addr, addr_width, comp_buffer, offset, len);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

static int run_mixed_stress(const struct device *i2c, uint8_t addr,
			    uint8_t addr_width, uint8_t *comp_buffer)
{
	for (int round = 0; round < CONFIG_I2C_TEST_STRESS_REPEAT_COUNT; round++) {
		size_t len = 1U + ((size_t)round % MIN((size_t)17U, sizeof(i2c_buffer)));
		unsigned int offset = (unsigned int)((round * 5U) % (TEST_DATA_SIZE - len + 1U));
		int ret;

		switch (round % 4U) {
		case 0:
			ret = run_direct_read(i2c, addr, addr_width, comp_buffer, offset, len);
			break;
		case 1:
			if (len < 2U) {
				ret = run_direct_read(i2c, addr, addr_width, comp_buffer, offset, len);
			} else {
				size_t first_len = len / 2U;

				ret = run_split_read(i2c, addr, addr_width, comp_buffer, offset,
						    first_len, len - first_len);
			}
			break;
		case 2:
			if (len < 2U) {
				ret = run_split_write_read(i2c, addr, addr_width, comp_buffer, offset, len);
			} else {
				size_t first_len = len / 2U;

				ret = run_combined_split_read(i2c, addr, addr_width, comp_buffer, offset,
							      first_len, len - first_len);
			}
			break;
		default:
			ret = run_split_write_read(i2c, addr, addr_width, comp_buffer, offset, len);
			break;
		}

		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

static int run_program_read(const struct device *i2c, uint8_t addr,
			    uint8_t addr_width, unsigned int offset)
{
	int ret, i;
	uint8_t buf[TEST_DATA_SIZE + 2];
	uint8_t addr_size;

	TC_PRINT("Testing program. Master: %s, address: 0x%x, off=%d\n",
		i2c->name, addr, offset);

	switch (addr_width) {
	case 8:
		buf[0] = (uint8_t) (offset & 0xFF);
		addr_size = 1;
	break;
	case 16:
		sys_put_be16((uint16_t)(offset & 0xFFFF), buf);
		addr_size = 2;
	break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < TEST_DATA_SIZE - offset; ++i) {
		buf[i + addr_size] = i & 0xFF;
	}

	ret = i2c_write(i2c, &buf[0], TEST_DATA_SIZE - offset + addr_size, addr);
	zassert_equal(ret, 0, "Failed to write EEPROM");

	/* Read back EEPROM from I2C Master requests, then compare */
	ret = i2c_write_read(i2c, addr, buf, addr_size, i2c_buffer, TEST_DATA_SIZE - offset);
	zassert_equal(ret, 0, "Failed to read EEPROM");

	for (i = 0 ; i < TEST_DATA_SIZE-offset ; ++i) {
		if (i2c_buffer[i] != (i & 0xFF)) {
			to_display_format(i2c_buffer, TEST_DATA_SIZE-offset,
					  buffer_print_i2c);
			TC_PRINT("Error: Unexpected %u (%02x) buffer content: %s\n",
				 i, i2c_buffer[i], buffer_print_i2c);
			return -EIO;
		}
	}

	return 0;
}

static int run_repeated_direct_reads(const struct device *i2c, uint8_t addr,
				     uint8_t addr_width, const uint8_t *comp_buffer)
{
	for (int round = 0; round < CONFIG_I2C_TEST_STRESS_REPEAT_COUNT; round++) {
		size_t len = 1U + ((size_t)round % MIN((size_t)4U, sizeof(i2c_buffer)));
		unsigned int max_offset = TEST_DATA_SIZE - len;
		unsigned int offset = (unsigned int)round % (max_offset + 1U);
		int ret;

		ret = run_direct_read(i2c, addr, addr_width, comp_buffer, offset, len);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

static int test_target_write_requested(struct i2c_target_config *config)
{
	struct test_i2c_target *target = CONTAINER_OF(config, struct test_i2c_target, config);

	target->addr_bytes_seen = 0U;
	target->data_bytes_seen = 0U;

	return 0;
}

static int test_target_read_requested(struct i2c_target_config *config, uint8_t *val)
{
	struct test_i2c_target *target = CONTAINER_OF(config, struct test_i2c_target, config);

	*val = target->data[target->offset % sizeof(target->data)];

	return 0;
}

static int test_target_write_received(struct i2c_target_config *config, uint8_t val)
{
	struct test_i2c_target *target = CONTAINER_OF(config, struct test_i2c_target, config);

	if (target->addr_bytes_seen == 0U) {
		target->offset = val;
		target->addr_bytes_seen = 1U;
	} else {
		target->data[target->offset % sizeof(target->data)] = val;
		target->offset++;
		target->data_bytes_seen++;
		if (target->nack_on_write_received &&
		    (target->data_bytes_seen >= target->nack_after_data)) {
			return -EIO;
		}
	}

	return 0;
}

static int test_target_read_processed(struct i2c_target_config *config, uint8_t *val)
{
	struct test_i2c_target *target = CONTAINER_OF(config, struct test_i2c_target, config);

	target->offset++;
	*val = target->data[target->offset % sizeof(target->data)];

	return 0;
}

static int test_target_stop(struct i2c_target_config *config)
{
	struct test_i2c_target *target = CONTAINER_OF(config, struct test_i2c_target, config);

	target->addr_bytes_seen = 0U;
	target->data_bytes_seen = 0U;

	return 0;
}

static const struct i2c_target_callbacks test_target_callbacks = {
	.write_requested = test_target_write_requested,
	.read_requested = test_target_read_requested,
	.write_received = test_target_write_received,
	.read_processed = test_target_read_processed,
	.stop = test_target_stop,
};

static int run_10bit_target_readback(const struct device *i2c, uint16_t addr)
{
	uint8_t offset = 3U;
	uint8_t readback[8];
	struct i2c_msg msgs[2] = {
		{
			.buf = &offset,
			.len = 1U,
			.flags = I2C_MSG_WRITE | I2C_MSG_ADDR_10_BITS,
		},
		{
			.buf = readback,
			.len = sizeof(readback),
			.flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP | I2C_MSG_ADDR_10_BITS,
		},
	};
	uint8_t expected[sizeof(readback)];
	int ret;

	for (size_t i = 0U; i < sizeof(expected); i++) {
		expected[i] = (uint8_t)(0x30U + i + offset);
	}

	ret = i2c_transfer(i2c, msgs, ARRAY_SIZE(msgs), addr);
	if (ret == -ENOTSUP) {
		return ret;
	}

	zassert_equal(ret, 0, "Failed 10-bit write-read transfer");

	return compare_buffer(readback, expected, sizeof(readback));
}

static int run_10bit_target_writeback(const struct device *i2c, uint16_t addr)
{
	uint8_t payload[] = { 6U, 0xA1U, 0xA2U, 0xA3U, 0xA4U };
	uint8_t readback[ARRAY_SIZE(payload) - 1U];
	struct i2c_msg write_msg = {
		.buf = payload,
		.len = ARRAY_SIZE(payload),
		.flags = I2C_MSG_WRITE | I2C_MSG_STOP | I2C_MSG_ADDR_10_BITS,
	};
	struct i2c_msg read_msgs[2] = {
		{
			.buf = payload,
			.len = 1U,
			.flags = I2C_MSG_WRITE | I2C_MSG_ADDR_10_BITS,
		},
		{
			.buf = readback,
			.len = sizeof(readback),
			.flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP | I2C_MSG_ADDR_10_BITS,
		},
	};
	int ret;

	ret = i2c_transfer(i2c, &write_msg, 1U, addr);
	if (ret == -ENOTSUP) {
		return ret;
	}

	zassert_equal(ret, 0, "Failed 10-bit write transfer");

	ret = i2c_transfer(i2c, read_msgs, ARRAY_SIZE(read_msgs), addr);
	zassert_equal(ret, 0, "Failed 10-bit readback transfer");

	return compare_buffer(readback, &payload[1], sizeof(readback));
}

static int run_target_data_nack_recovery(const struct device *i2c_target_bus,
					 const struct device *i2c_master_bus)
{
	struct test_i2c_target target = {
		.config = {
			.address = 0x62U,
			.callbacks = &test_target_callbacks,
		},
		.nack_after_data = 1U,
		.nack_on_write_received = true,
	};
	uint8_t failing_payload[] = { 4U, 0xD1U, 0xD2U, 0xD3U };
	uint8_t good_payload[] = { 7U, 0xE1U, 0xE2U, 0xE3U };
	uint8_t offset = good_payload[0];
	uint8_t readback[ARRAY_SIZE(good_payload) - 1U];
	int ret;

	for (size_t i = 0U; i < sizeof(target.data); i++) {
		target.data[i] = (uint8_t)(0x20U + i);
	}

	ret = i2c_target_register(i2c_target_bus, &target.config);
	if (ret == -ENOTSUP) {
		return ret;
	}

	zassert_equal(ret, 0, "Failed to register raw target for data NACK test");

	ret = i2c_write(i2c_master_bus, failing_payload, sizeof(failing_payload), target.config.address);
	zassert_not_equal(ret, 0, "Unexpected ACK on target data NACK test");

	target.nack_on_write_received = false;
	target.addr_bytes_seen = 0U;
	target.data_bytes_seen = 0U;

	ret = i2c_write(i2c_master_bus, good_payload, sizeof(good_payload), target.config.address);
	zassert_equal(ret, 0, "Failed write after target data NACK");

	ret = i2c_write_read(i2c_master_bus, target.config.address, &offset, 1U,
			     readback, sizeof(readback));
	zassert_equal(ret, 0, "Failed readback after target data NACK");

	ret = compare_buffer(readback, &good_payload[1], sizeof(readback));

	zassert_ok(i2c_target_unregister(i2c_target_bus, &target.config));

	return ret;
}

ZTEST(i2c_eeprom_target, test_deinit)
{
	const struct device *const i2c_0 = DEVICE_DT_GET(DT_BUS(NODE_EP0));
	const struct device *const i2c_1 = DEVICE_DT_GET(DT_BUS(NODE_EP1));
	const struct gpio_dt_spec sda_pin_0 =
		GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), sda0_gpios, {});
	const struct gpio_dt_spec scl_pin_0 =
		GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), scl0_gpios, {});
	const struct gpio_dt_spec sda_pin_1 =
		GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), sda1_gpios, {});
	const struct gpio_dt_spec scl_pin_1 =
		GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), scl1_gpios, {});
	int ret;

	if (i2c_0 == i2c_1) {
		TC_PRINT("  gpio loopback required for test\n");
		ztest_test_skip();
	}

	if (scl_pin_0.port == NULL || sda_pin_0.port == NULL ||
	    scl_pin_1.port == NULL || sda_pin_1.port == NULL) {
		TC_PRINT("  bus gpios not specified in zephyr,path\n");
		ztest_test_skip();
	}

	ret = device_deinit(i2c_0);
	if (ret == -ENOTSUP) {
		TC_PRINT("  device deinit not supported\n");
		ztest_test_skip();
	}

	zassert_ok(ret);

	ret = device_deinit(i2c_1);
	if (ret == -ENOTSUP) {
		TC_PRINT("  device deinit not supported\n");
		zassert_ok(device_init(i2c_0));
		ztest_test_skip();
	}

	zassert_ok(gpio_pin_configure_dt(&sda_pin_0, GPIO_INPUT));
	zassert_ok(gpio_pin_configure_dt(&sda_pin_1, GPIO_OUTPUT_INACTIVE));
	zassert_ok(gpio_pin_configure_dt(&scl_pin_0, GPIO_INPUT));
	zassert_ok(gpio_pin_configure_dt(&scl_pin_1, GPIO_OUTPUT_INACTIVE));
	zassert_equal(gpio_pin_get_dt(&sda_pin_0), 0);
	zassert_equal(gpio_pin_get_dt(&scl_pin_0), 0);
	zassert_ok(gpio_pin_set_dt(&sda_pin_1, 1));
	zassert_ok(gpio_pin_set_dt(&scl_pin_1, 1));
	zassert_equal(gpio_pin_get_dt(&sda_pin_0), 1);
	zassert_equal(gpio_pin_get_dt(&scl_pin_0), 1);
	zassert_ok(gpio_pin_configure_dt(&sda_pin_1, GPIO_INPUT));
	zassert_ok(gpio_pin_configure_dt(&scl_pin_1, GPIO_INPUT));
	zassert_ok(device_init(i2c_0));
	zassert_ok(device_init(i2c_1));
}

ZTEST(i2c_eeprom_target, test_eeprom_target)
{
	const struct device *const eeprom_0 = DEVICE_DT_GET(NODE_EP0);
	const struct device *const i2c_0 = DEVICE_DT_GET(DT_BUS(NODE_EP0));
	const struct gpio_dt_spec sda_pin_0 =
		GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), sda0_gpios, {});
	const struct gpio_dt_spec scl_pin_0 =
		GPIO_DT_SPEC_GET_OR(DT_PATH(zephyr_user), scl0_gpios, {});
	int addr_0 = DT_REG_ADDR(NODE_EP0);
	uint8_t addr_0_width = DT_PROP_OR(NODE_EP0, address_width, 8);
	const struct device *const eeprom_1 = DEVICE_DT_GET(NODE_EP1);
	const struct device *const i2c_1 = DEVICE_DT_GET(DT_BUS(NODE_EP1));
	int addr_1 = DT_REG_ADDR(NODE_EP1);
	uint8_t addr_1_width = DT_PROP_OR(NODE_EP1, address_width, 8);
	int ret, offset;

	init_eeprom_test_data();

	zassert_not_null(i2c_0, "EEPROM 0 - I2C bus not found");
	zassert_not_null(eeprom_0, "EEPROM 0 device not found");

	zassert_true(device_is_ready(i2c_0), "EEPROM 0 - I2C bus not ready");

	TC_PRINT("Found EEPROM 0 on I2C bus device %s at addr %02x\n",
		 i2c_0->name, addr_0);

	zassert_not_null(i2c_1, "EEPROM 1 - I2C device not found");
	zassert_not_null(eeprom_1, "EEPROM 1 device not found");

	zassert_true(device_is_ready(i2c_1), "EEPROM 1 - I2C bus not ready");

	TC_PRINT("Found EEPROM 1 on I2C bus device %s at addr %02x\n",
		 i2c_1->name, addr_1);

	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		TC_PRINT("Testing dual-role\n");
	} else {
		TC_PRINT("Testing single-role\n");
	}

	/* Program differentiable data into the two devices through a back door
	 * that doesn't use I2C.
	 */
	ret = eeprom_target_write_data(eeprom_0, 0, eeprom_0_data, TEST_DATA_SIZE);
	zassert_equal(ret, 0, "Failed to program EEPROM 0");
	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		ret = eeprom_target_write_data(eeprom_1, 0, eeprom_1_data,
					       TEST_DATA_SIZE);
		zassert_equal(ret, 0, "Failed to program EEPROM 1");
	}

	/* Attach each EEPROM to its owning bus as a target device. */
	ret = i2c_target_driver_register(eeprom_0);
	zassert_equal(ret, 0, "Failed to register EEPROM 0");

	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		ret = i2c_target_driver_register(eeprom_1);
		zassert_equal(ret, 0, "Failed to register EEPROM 1");
	}

	/* The simulated EP0 is configured to be accessed as a target device
	 * at addr_0 on i2c_0 and should expose eeprom_0_data.  The validation
	 * uses i2c_1 as a bus master to access this device, which works because
	 * i2c_0 and i2_c have their SDA (SCL) pins shorted (they are on the
	 * same physical bus).  Thus in these calls i2c_1 is a master device
	 * operating on the target address addr_0.
	 *
	 * Similarly validation of EP1 uses i2c_0 as a master with addr_1 and
	 * eeprom_1_data for validation.
	 */
	ret = run_full_read(i2c_1, addr_0, addr_0_width, eeprom_0_data);
	zassert_equal(ret, 0,
		     "Full I2C read from EP0 failed");
	ret = run_boundary_direct_reads(i2c_1, addr_0, addr_0_width, eeprom_0_data);
	zassert_equal(ret, 0, "Boundary direct I2C reads from EP0 failed");
	ret = run_boundary_split_reads(i2c_1, addr_0, addr_0_width, eeprom_0_data);
	zassert_equal(ret, 0, "Boundary split reads from EP0 failed");
	ret = run_address_nack_recovery(i2c_1, addr_0, addr_0_width, eeprom_0_data, addr_1);
	zassert_equal(ret, 0, "Address NACK recovery from EP0 failed");
	ret = run_gpio_fault_recovery(i2c_1, i2c_0,
				      IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C) ? eeprom_1 : NULL,
				      eeprom_0, &sda_pin_0, &scl_pin_0, true,
				      addr_0, addr_0_width, eeprom_0_data);
	zassert_equal(ret, 0, "SDA-low recovery from EP0 failed");
	ret = run_gpio_fault_recovery(i2c_1, i2c_0,
				      IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C) ? eeprom_1 : NULL,
				      eeprom_0, &sda_pin_0, &scl_pin_0, false,
				      addr_0, addr_0_width, eeprom_0_data);
	zassert_equal(ret, 0, "SCL-low recovery from EP0 failed");
	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		ret = run_full_read(i2c_0, addr_1, addr_1_width, eeprom_1_data);
		zassert_equal(ret, 0,
			      "Full I2C read from EP1 failed");
		ret = run_boundary_direct_reads(i2c_0, addr_1, addr_1_width, eeprom_1_data);
		zassert_equal(ret, 0, "Boundary direct I2C reads from EP1 failed");
		ret = run_boundary_split_reads(i2c_0, addr_1, addr_1_width, eeprom_1_data);
		zassert_equal(ret, 0, "Boundary split reads from EP1 failed");
		ret = run_address_nack_recovery(i2c_0, addr_1, addr_1_width, eeprom_1_data,
						addr_0);
		zassert_equal(ret, 0, "Address NACK recovery from EP1 failed");
	}

	for (offset = 0 ; offset < TEST_DATA_SIZE ; ++offset) {
		if (!should_run_offset(offset)) {
			continue;
		}

		zassert_equal(0, run_partial_read(i2c_1, addr_0,
			      addr_0_width, eeprom_0_data, offset),
			      "Partial I2C read EP0 failed");
		if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
			zassert_equal(0, run_partial_read(i2c_0, addr_1,
							  addr_1_width,
							  eeprom_1_data,
							  offset),
				      "Partial I2C read EP1 failed");
		}
	}

	zassert_equal(0, run_repeated_direct_reads(i2c_1, addr_0, addr_0_width, eeprom_0_data),
		      "Repeated direct I2C reads from EP0 failed");
	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		zassert_equal(0, run_repeated_direct_reads(i2c_0, addr_1, addr_1_width,
							  eeprom_1_data),
			      "Repeated direct I2C reads from EP1 failed");
	}

	memcpy(expected_0_data, eeprom_0_data, TEST_DATA_SIZE);
	zassert_equal(0, run_boundary_split_writes(i2c_1, addr_0, addr_0_width, expected_0_data),
		      "Split write/readback EP0 failed");
	zassert_equal(0, run_mixed_stress(i2c_1, addr_0, addr_0_width, expected_0_data),
		      "Mixed stress EP0 failed");
	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		memcpy(expected_1_data, eeprom_1_data, TEST_DATA_SIZE);
		zassert_equal(0, run_boundary_split_writes(i2c_0, addr_1, addr_1_width,
							expected_1_data),
			      "Split write/readback EP1 failed");
		zassert_equal(0, run_mixed_stress(i2c_0, addr_1, addr_1_width, expected_1_data),
			      "Mixed stress EP1 failed");
	}

	ret = eeprom_target_write_data(eeprom_0, 0, eeprom_0_data, TEST_DATA_SIZE);
	zassert_equal(ret, 0, "Failed to restore EEPROM 0 after stress");
	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		ret = eeprom_target_write_data(eeprom_1, 0, eeprom_1_data, TEST_DATA_SIZE);
		zassert_equal(ret, 0, "Failed to restore EEPROM 1 after stress");
	}

	for (offset = 0 ; offset < TEST_DATA_SIZE ; ++offset) {
		if (!should_run_offset(offset)) {
			continue;
		}

		zassert_equal(0, run_program_read(i2c_1, addr_0,
							  addr_0_width, offset),
			      "Program I2C read EP0 failed");
		if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
			zassert_equal(0, run_program_read(i2c_0, addr_1,
							  addr_1_width, offset),
				      "Program I2C read EP1 failed");
		}
	}

	/* Detach EEPROM */
	ret = i2c_target_driver_unregister(eeprom_0);
	zassert_equal(ret, 0, "Failed to unregister EEPROM 0");

	if (IS_ENABLED(CONFIG_APP_DUAL_ROLE_I2C)) {
		ret = i2c_target_driver_unregister(eeprom_1);
		zassert_equal(ret, 0, "Failed to unregister EEPROM 1");
	}
}

ZTEST(i2c_eeprom_target, test_optional_10bit_target)
{
	const struct device *const i2c_target_bus = DEVICE_DT_GET(DT_BUS(NODE_EP0));
	const struct device *const i2c_master_bus = DEVICE_DT_GET(DT_BUS(NODE_EP1));
	struct test_i2c_target target = {
		.config = {
			.flags = I2C_TARGET_FLAGS_ADDR_10_BITS,
			.address = 0x2A5U,
			.callbacks = &test_target_callbacks,
		},
	};
	int ret;

	if (i2c_target_bus == i2c_master_bus) {
		TC_PRINT("  separate controller/target buses required for 10-bit target test\n");
		ztest_test_skip();
	}

	for (size_t i = 0U; i < sizeof(target.data); i++) {
		target.data[i] = (uint8_t)(0x30U + i);
	}

	ret = i2c_target_register(i2c_target_bus, &target.config);
	if (ret == -ENOTSUP) {
		TC_PRINT("  target 10-bit addressing not supported\n");
		ztest_test_skip();
	}

	zassert_equal(ret, 0, "Failed to register 10-bit raw target");

	ret = run_10bit_target_readback(i2c_master_bus, target.config.address);
	zassert_equal(ret, 0, "10-bit read path failed");
	ret = run_10bit_target_writeback(i2c_master_bus, target.config.address);
	zassert_equal(ret, 0, "10-bit write path failed");

	zassert_ok(i2c_target_unregister(i2c_target_bus, &target.config));
}

ZTEST(i2c_eeprom_target, test_target_data_nack_recovery)
{
	const struct device *const i2c_target_bus = DEVICE_DT_GET(DT_BUS(NODE_EP0));
	const struct device *const i2c_master_bus = DEVICE_DT_GET(DT_BUS(NODE_EP1));
	int ret;

	if (i2c_target_bus == i2c_master_bus) {
		TC_PRINT("  separate controller/target buses required for target data NACK test\n");
		ztest_test_skip();
	}

	ret = run_target_data_nack_recovery(i2c_target_bus, i2c_master_bus);
	if (ret == -ENOTSUP) {
		TC_PRINT("  target callbacks do not support data NACK injection\n");
		ztest_test_skip();
	}

	zassert_equal(ret, 0, "Target data NACK recovery failed");
}

ZTEST_SUITE(i2c_eeprom_target, NULL, NULL, NULL, NULL, NULL);
