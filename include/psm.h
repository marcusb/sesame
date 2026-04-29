#ifndef SESAME_PSM_H
#define SESAME_PSM_H

#ifdef QEMU
#include <stdint.h>
#include <stdbool.h>

typedef void *psm_hnd_t;

int psm_module_init(void *fdesc, psm_hnd_t *phandle, void *psm_cfg);
int psm_set_variable(psm_hnd_t phandle, const char *variable, const void *value, uint32_t len);
int psm_get_variable(psm_hnd_t phandle, const char *variable, void *value, uint32_t max_len);
int psm_get_variable_size(psm_hnd_t phandle, const char *variable);
int psm_object_delete(psm_hnd_t phandle, const char *variable);

static inline bool psm_is_variable_present(psm_hnd_t phandle, const char *variable)
{
    return (psm_get_variable_size(phandle, variable) >= 0);
}

#else
#include "psm-v2.h"
#endif

#endif // SESAME_PSM_H
