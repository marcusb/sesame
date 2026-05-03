#include "stub_psm.h"

#include <stdint.h>
#include <string.h>

#include "psm.h"

static uint8_t psm_buf[1024];
static size_t psm_len = 0;
static int psm_error = 0;

void stub_psm_set_data(const uint8_t* data, size_t len) {
    if (len > sizeof(psm_buf)) len = sizeof(psm_buf);
    memcpy(psm_buf, data, len);
    psm_len = len;
    psm_error = 0;
}

void stub_psm_get_written(const uint8_t** out, size_t* out_len) {
    *out = psm_buf;
    *out_len = psm_len;
}

void stub_psm_set_error(int err) {
    psm_error = err;
    if (err) psm_len = 0;
}

void stub_psm_clear(void) {
    psm_len = 0;
    psm_error = 0;
    memset(psm_buf, 0, sizeof(psm_buf));
}

#ifdef QEMU
int psm_module_init(void* fdesc, psm_hnd_t* phandle, void* psm_cfg) {
#else
int psm_module_init(flash_desc_t* fdesc, psm_hnd_t* phandle,
                    psm_cfg_t* psm_cfg) {
#endif
    (void)fdesc;
    (void)psm_cfg;
    if (phandle) *phandle = (psm_hnd_t)1;
    return 0;
}

int psm_get_variable(psm_hnd_t phandle, const char* variable, void* value,
                     uint32_t max_len) {
    (void)phandle;
    (void)variable;
    if (psm_error) return psm_error;
    if (psm_len == 0) return -1;
    uint32_t copy = (psm_len < max_len) ? (uint32_t)psm_len : max_len;
    memcpy(value, psm_buf, copy);
    return (int)copy;
}

int psm_set_variable(psm_hnd_t phandle, const char* variable, const void* value,
                     uint32_t len) {
    (void)phandle;
    (void)variable;
    if (psm_error) return psm_error;
    if (len > sizeof(psm_buf)) len = sizeof(psm_buf);
    memcpy(psm_buf, value, len);
    psm_len = len;
    return 0;
}

int psm_object_delete(psm_hnd_t phandle, const char* variable) {
    (void)phandle;
    (void)variable;
    psm_len = 0;
    return 0;
}
