/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_trng

#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include <gd32_trng.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(entropy_gd32, CONFIG_ENTROPY_LOG_LEVEL);

struct entropy_gd32_config {
    uint16_t clkid;
    struct reset_dt_spec reset;
};

struct entropy_gd32_data {
    struct k_sem lock;
};

static inline int trng_wait_ready(void)
{
    for (;;) {
        if (SET == trng_flag_get(TRNG_FLAG_DRDY)) {
            return 0;
        }
        if ((SET == trng_flag_get(TRNG_FLAG_SECS)) ||
            (SET == trng_flag_get(TRNG_FLAG_CECS))) {
            trng_interrupt_flag_clear(TRNG_INT_FLAG_SEIF);
            trng_interrupt_flag_clear(TRNG_INT_FLAG_CEIF);
            trng_deinit();
            trng_enable();
        }
        k_sleep(K_USEC(50));
    }
}

static int entropy_gd32_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
    struct entropy_gd32_data *data = dev->data;
    uint8_t *p = buffer;

    k_sem_take(&data->lock, K_FOREVER);

    while (length > 0) {
        int ret = trng_wait_ready();
        if (ret) {
            k_sem_give(&data->lock);
            return ret;
        }

        uint32_t val = trng_get_true_random_data();
        uint16_t n = MIN((uint16_t)sizeof(val), length);
        memcpy(p, &val, n);
        p += n;
        length -= n;
    }

    k_sem_give(&data->lock);
    return 0;
}

static DEVICE_API(entropy, entropy_gd32_api) = {
    .get_entropy = entropy_gd32_get_entropy,
};

static int entropy_gd32_init(const struct device *dev)
{
    const struct entropy_gd32_config *cfg = dev->config;
    struct entropy_gd32_data *data = dev->data;

    (void)clock_control_on(GD32_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);
    (void)reset_line_toggle_dt(&cfg->reset);

    k_sem_init(&data->lock, 1, 1);

    trng_deinit();
    trng_enable();
    trng_interrupt_disable();

    return 0;
}

static const struct entropy_gd32_config entropy_gd32_config_0 = {
    .clkid = DT_INST_CLOCKS_CELL(0, id),
    .reset = RESET_DT_SPEC_INST_GET(0),
};

static struct entropy_gd32_data entropy_gd32_data_0;

DEVICE_DT_INST_DEFINE(0,
                      entropy_gd32_init, NULL,
                      &entropy_gd32_data_0, &entropy_gd32_config_0,
                      PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY,
                      &entropy_gd32_api);
