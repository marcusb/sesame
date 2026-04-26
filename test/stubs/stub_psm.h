#pragma once

#include <stddef.h>
#include <stdint.h>

void stub_psm_set_data(const uint8_t* data, size_t len);
void stub_psm_get_written(const uint8_t** out, size_t* out_len);
void stub_psm_set_error(int err);

typedef void* psm_hnd_t;
int psm_get_variable(psm_hnd_t phandle, const char* variable, void* value,
                     uint32_t max_len);
int psm_set_variable(psm_hnd_t phandle, const char* variable, const void* value,
                     uint32_t len);
int psm_object_delete(psm_hnd_t phandle, const char* variable);
