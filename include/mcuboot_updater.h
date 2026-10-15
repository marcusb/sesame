#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCUBOOT_MAX_IMG_SIZE (128 * 1024)

/**
 * Validates an in-memory mcuboot.bin image.
 * Verifies size constraints and Marvell firmware headers (magic, signature,
 * segments, CRC).
 *
 * @param data Pointer to buffer
 * @param len Size of data in bytes
 * @return 0 if valid, negative errno code if invalid.
 */
int mcuboot_validate_image(const uint8_t* data, size_t len);

/**
 * Resets the RAM buffer and updater state.
 */
void mcuboot_updater_reset(void);

/**
 * Appends a received data chunk to the in-memory RAM buffer.
 *
 * @param data Chunk bytes
 * @param len Chunk length
 * @return 0 on success, -ENOMEM or -EINVAL on failure.
 */
int mcuboot_updater_process_chunk(const uint8_t* data, size_t len);

/**
 * Completes the MCUboot update:
 * 1. Validates the complete buffered image.
 * 2. Erases the boot_partition in flash.
 * 3. Writes the buffered image to flash.
 * 4. Reads back and verifies flash against the RAM buffer.
 * 5. Reboots device on success.
 *
 * @return 0 on success, negative errno code on failure.
 */
int mcuboot_updater_complete(void);

/**
 * Aborts the update and frees RAM buffer.
 */
/**
 * Aborts the update and frees RAM buffer.
 */
void mcuboot_updater_abort(void);

/**
 * Promotes the currently running MCUboot image by marking it confirmed.
 *
 * @return 0 on success, negative errno on failure.
 */
int mcuboot_updater_promote(void);

#ifdef __cplusplus
}
#endif
