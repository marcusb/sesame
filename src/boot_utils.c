#include "boot_utils.h"

#include <string.h>

#if defined(CONFIG_SOC_88MW320)
#define BOOT_FLAGS ((volatile uint32_t*)0x480C0024)
#endif

#if !defined(CONFIG_SOC_88MW320)
static struct mcuboot_info s_sim_info;
#endif

uint32_t boot_get_flags(void) {
#if defined(CONFIG_SOC_88MW320)
    return *BOOT_FLAGS;
#else
    return 0;
#endif
}

uint32_t boot_get_slot(void) {
#if defined(CONFIG_SOC_88MW320)
    return (*BOOT_FLAGS) & 0x03;
#else
    return s_sim_info.slot;
#endif
}

void boot_info_write(const char* version_str, uint32_t slot, uint32_t gen_level,
                     uint32_t confirmed) {
#if defined(CONFIG_SOC_88MW320)
    struct mcuboot_info* info = (struct mcuboot_info*)MCUBOOT_INFO_ADDR;
    memset(info, 0, sizeof(*info));
    info->magic = MCUBOOT_INFO_MAGIC;
    if (version_str) {
        strncpy(info->version_str, version_str, sizeof(info->version_str) - 1);
    }
    info->slot = slot;
    info->gen_level = gen_level;
    info->confirmed = confirmed;
#else
    memset(&s_sim_info, 0, sizeof(s_sim_info));
    s_sim_info.magic = MCUBOOT_INFO_MAGIC;
    if (version_str) {
        strncpy(s_sim_info.version_str, version_str,
                sizeof(s_sim_info.version_str) - 1);
    }
    s_sim_info.slot = slot;
    s_sim_info.gen_level = gen_level;
    s_sim_info.confirmed = confirmed;
#endif
}

const struct mcuboot_info* boot_info_read(void) {
#if defined(CONFIG_SOC_88MW320)
    struct mcuboot_info* info = (struct mcuboot_info*)MCUBOOT_INFO_ADDR;
    if (info->magic == MCUBOOT_INFO_MAGIC) {
        return info;
    }
    return NULL;
#else
    if (s_sim_info.magic == MCUBOOT_INFO_MAGIC) {
        return &s_sim_info;
    }
    return NULL;
#endif
}
