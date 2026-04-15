#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "psm-v2.h"

int psm_set_variable(psm_hnd_t hnd, const char* variable, const void* value,
                     uint32_t len) {
    (void)hnd;
    (void)variable;
    (void)value;
    (void)len;
    return 0;
}

int psm_get_variable(psm_hnd_t hnd, const char* variable, void* value,
                     uint32_t len) {
    (void)hnd;
    (void)variable;
    (void)value;
    (void)len;
    return -1;  // Not found
}

int psm_get_variable_size(psm_hnd_t hnd, const char* variable) {
    (void)hnd;
    (void)variable;
    return -1;
}

int psm_object_delete(psm_hnd_t hnd, const char* variable) {
    (void)hnd;
    (void)variable;
    return 0;
}

psm_hnd_t psm_hnd;
