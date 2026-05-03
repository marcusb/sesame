#pragma once

#include <stddef.h>
#include <stdint.h>

#include "psm.h"

void stub_psm_set_data(const uint8_t* data, size_t len);
void stub_psm_get_written(const uint8_t** out, size_t* out_len);
void stub_psm_set_error(int err);
void stub_psm_clear(void);

/* PSM base functions are declared in psm.h with appropriate types for the
 * platform. */
