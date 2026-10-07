#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * Reset updater state machine before receiving a new upload.
 */
void pic_updater_reset(void);

/**
 * Process a chunk of Intel HEX data received over HTTP.
 * Automatically enters SBL mode on the first chunk.
 * Returns 0 on success, negative error code on failure.
 */
int pic_updater_process_chunk(const uint8_t* data, size_t len);

/**
 * Finalize the update after all HTTP chunks have been received.
 * Returns 0 on success, negative error code on failure.
 */
int pic_updater_complete(void);

/**
 * Abort an ongoing update, restore PIC to normal run mode.
 */
void pic_updater_abort(void);
