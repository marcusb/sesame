#include "psm_safe.h"

#include <string.h>

#include "FreeRTOS.h"
#include "app_logging.h"
#include "portable.h"

#define TAG "psm_safe"

#define XIP_FLASH_BASE 0x1F000000U
#define XIP_FLASH_MASK 0xFF000000U

static inline int in_xip(const void* p) {
    return ((uintptr_t)p & XIP_FLASH_MASK) == XIP_FLASH_BASE;
}

int psm_set_variable_safe(psm_hnd_t handle, const char* variable,
                          const void* value, uint32_t len) {
    char* tmp_name = NULL;
    void* tmp_value = NULL;
    const char* name_to_use = variable;
    const void* value_to_use = value;

    if (variable && in_xip(variable)) {
        size_t n = strlen(variable) + 1;
        tmp_name = pvPortMalloc(n);
        if (!tmp_name) return -1;
        memcpy(tmp_name, variable, n);
        name_to_use = tmp_name;
    }

    if (value && len > 0 && in_xip(value)) {
        tmp_value = pvPortMalloc(len);
        if (!tmp_value) {
            if (tmp_name) vPortFree(tmp_name);
            return -1;
        }
        memcpy(tmp_value, value, len);
        value_to_use = tmp_value;
    }

    int rv = psm_set_variable(handle, name_to_use, value_to_use, len);

    if (tmp_name) vPortFree(tmp_name);
    if (tmp_value) vPortFree(tmp_value);
    return rv;
}
