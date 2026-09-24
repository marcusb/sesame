/*
 * Copyright 2026 Marcus Better
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_mw320_bbram

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/bbram.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(bbram_mw320, CONFIG_BBRAM_LOG_LEVEL);

#define BBRAM_REG(base, idx) (((volatile uint32_t*)(base))[(idx)])

struct bbram_mw320_config {
    uintptr_t base_addr;
    size_t size;
};

static int bbram_mw320_read(const struct device* dev, size_t offset,
                            size_t size, uint8_t* data) {
    const struct bbram_mw320_config* config = dev->config;

    if (size < 1 || offset + size > config->size) {
        return -EINVAL;
    }

    for (size_t read = 0; read < size;) {
        size_t word_idx = (offset + read) / sizeof(uint32_t);
        size_t byte_idx = (offset + read) % sizeof(uint32_t);
        size_t to_copy = MIN(sizeof(uint32_t) - byte_idx, size - read);
        uint32_t val = BBRAM_REG(config->base_addr, word_idx);

        memcpy(data + read, ((uint8_t*)&val) + byte_idx, to_copy);
        read += to_copy;
    }

    return 0;
}

static int bbram_mw320_write(const struct device* dev, size_t offset,
                             size_t size, const uint8_t* data) {
    const struct bbram_mw320_config* config = dev->config;

    if (size < 1 || offset + size > config->size) {
        return -EINVAL;
    }

    for (size_t written = 0; written < size;) {
        size_t word_idx = (offset + written) / sizeof(uint32_t);
        size_t byte_idx = (offset + written) % sizeof(uint32_t);
        size_t to_copy = MIN(sizeof(uint32_t) - byte_idx, size - written);

        if (to_copy == sizeof(uint32_t)) {
            uint32_t val;
            memcpy(&val, data + written, sizeof(uint32_t));
            BBRAM_REG(config->base_addr, word_idx) = val;
        } else {
            uint32_t val = BBRAM_REG(config->base_addr, word_idx);
            memcpy(((uint8_t*)&val) + byte_idx, data + written, to_copy);
            BBRAM_REG(config->base_addr, word_idx) = val;
        }
        written += to_copy;
    }

    return 0;
}

static int bbram_mw320_get_size(const struct device* dev, size_t* size) {
    const struct bbram_mw320_config* config = dev->config;

    if (size == NULL) {
        return -EINVAL;
    }

    *size = config->size;
    return 0;
}

static DEVICE_API(bbram, bbram_mw320_driver_api) = {
    .read = bbram_mw320_read,
    .write = bbram_mw320_write,
    .get_size = bbram_mw320_get_size,
};

static int bbram_mw320_init(const struct device* dev) {
    ARG_UNUSED(dev);
    return 0;
}

#define BBRAM_MW320_INIT(n)                                           \
    static const struct bbram_mw320_config bbram_mw320_config_##n = { \
        .base_addr = DT_INST_REG_ADDR(n),                             \
        .size = DT_INST_REG_SIZE(n),                                  \
    };                                                                \
                                                                      \
    DEVICE_DT_INST_DEFINE(                                            \
        n, bbram_mw320_init, NULL, NULL, &bbram_mw320_config_##n,     \
        PRE_KERNEL_1, CONFIG_BBRAM_INIT_PRIORITY, &bbram_mw320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BBRAM_MW320_INIT)
