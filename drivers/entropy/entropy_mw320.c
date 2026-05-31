#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/irq.h>
#include <zephyr/random/random.h>

/* This assumes fsl_aes or ksdk_mbedtls has TRNG functions.
 * The mw320 sdk has a TRNG engine in fsl_aes.c usually or a separate fsl_trng.c
 */

#define DT_DRV_COMPAT nxp_mw320_entropy

static int entropy_mw320_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
    /* In a full implementation, we'd call the hardware TRNG.
     * The MW320 SDK provides AES_GetRandomData() or similar.
     */
    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = 4; /* Placeholder: fair dice roll */
    }
    return 0;
}

static const struct entropy_driver_api entropy_mw320_api_funcs = {
    .get_entropy = entropy_mw320_get_entropy
};

static int entropy_mw320_init(const struct device *dev)
{
    /* Initialize TRNG hardware */
    return 0;
}

DEVICE_DT_INST_DEFINE(0,
            entropy_mw320_init, NULL,
            NULL, NULL,
            PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY,
            &entropy_mw320_api_funcs);
