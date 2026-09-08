#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#define SRAM1_BGN DT_REG_ADDR(DT_NODELABEL(sram1))
#define SRAM1_LEN DT_REG_SIZE(DT_NODELABEL(sram1))

static uint8_t s_entropy_pool[32];
static size_t s_entropy_idx = 0;

static void mix_pool(uint32_t counter) {
    // Simple mixing function (FNV-1a style)
    uint32_t hash = 2166136261u ^ counter;
    for (int i = 0; i < sizeof(s_entropy_pool); i++) {
        hash ^= s_entropy_pool[i];
        hash *= 16777619u;
        s_entropy_pool[i] = (uint8_t)(hash & 0xFF);
        hash >>= 3;  // Shift to mix bits
    }
}

static int entropy_mw320_get_entropy(const struct device* dev, uint8_t* buffer,
                                     uint16_t length) {
    while (length > 0) {
        size_t to_copy = MIN(length, sizeof(s_entropy_pool) - s_entropy_idx);
        if (to_copy == 0) {
            static uint32_t counter = 0;
            mix_pool(counter++);
            s_entropy_idx = 0;
            to_copy = MIN(length, sizeof(s_entropy_pool));
        }
        memcpy(buffer, &s_entropy_pool[s_entropy_idx], to_copy);
        s_entropy_idx += to_copy;
        buffer += to_copy;
        length -= to_copy;
    }
    return 0;
}

static int entropy_mw320_get_entropy_isr(const struct device* dev,
                                         uint8_t* buffer, uint16_t length,
                                         uint32_t flags) {
    return entropy_mw320_get_entropy(dev, buffer, length);
}

static const struct entropy_driver_api entropy_mw320_api = {
    .get_entropy = entropy_mw320_get_entropy,
    .get_entropy_isr = entropy_mw320_get_entropy_isr,
};

static int entropy_mw320_init(const struct device* dev) {
    // Hash uninitialized SRAM1 memory to generate the initial seed
    uint8_t* sram = (uint8_t*)SRAM1_BGN;
    uint32_t sram1_len = SRAM1_LEN;

    // Mix SRAM1 into the 32-byte pool
    for (uint32_t i = 0; i < sram1_len; i++) {
        s_entropy_pool[i % 32] ^= sram[i];
    }

    mix_pool(0x1337);
    return 0;
}

#define DT_DRV_COMPAT marvell_mw320_entropy

DEVICE_DT_DEFINE(DT_NODELABEL(entropy_mw320), entropy_mw320_init, NULL, NULL,
                 NULL, PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY,
                 &entropy_mw320_api);
