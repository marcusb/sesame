#include "ota.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "app_logging.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "transport_plaintext.h"

// mw320
#include "boot_flags.h"
#include "crc32.h"
#include "mflash_drv.h"
#include "partition.h"

// application
#include "controller.h"
#include "leds.h"

#define OTA_TEST_MODE_FLAG (1UL << 31U)
#define FW_MAGIC_STR (('M' << 0) | ('R' << 8) | ('V' << 16) | ('L' << 24))
#define FW_MAGIC_SIG \
    ((0x7BUL << 0) | (0xF1UL << 8) | (0x9CUL << 16) | (0x2EUL << 24))
#define FW_BLK_LOADABLE_SEGMENT 2

ota_status_t ota_status = OTA_STATUS_NONE;

/*
 * Firmware magic signature
 *
 * First word is the string "MRVL" and is endian invariant.
 * Second word = magic value 0x2e9cf17b.
 * Third word = time stamp (seconds since 00:00:00 UTC, January 1, 1970).
 */
struct img_hdr {
    uint32_t magic_str;
    uint32_t magic_sig;
    uint32_t time;
    uint32_t seg_cnt;
    uint32_t entry;
};

/* Maximum number of segments */
#define SEG_CNT 9

struct seg_hdr {
    uint32_t type;
    uint32_t offset;
    uint32_t len;
    uint32_t laddr;
    uint32_t crc;
};

static uint32_t calculate_image_crc(uint32_t flash_addr, uint32_t size) {
    uint32_t buf[32];
    uint32_t crc = 0;
    uint32_t end = flash_addr + size;

    while (flash_addr < end) {
        int32_t n = min(sizeof(buf), end - flash_addr);
        if (mflash_drv_read(flash_addr, buf, n) != kStatus_Success) {
            return 0;
        }
        crc = soft_crc32(buf, n, crc);
        flash_addr += n;
    }
    return crc;
}

static int validate_update_image(uint32_t flash_addr, uint32_t size) {
    struct img_hdr ih;
    struct seg_hdr sh;
    int32_t result;

    if (size < sizeof(ih) + sizeof(sh)) {
        return -1;
    }

    result = mflash_drv_read(flash_addr, (uint32_t*)&ih, sizeof(ih));
    if (result != kStatus_Success) {
        return -2;
    }

    if ((ih.magic_str != FW_MAGIC_STR) || (ih.magic_sig != FW_MAGIC_SIG)) {
        return -3;
    }

    uint32_t sh_addr = flash_addr + sizeof(ih);
    for (int i = 0; i < min(ih.seg_cnt, SEG_CNT); i++) {
        result = mflash_drv_read(sh_addr, (uint32_t*)&sh, sizeof(sh));
        sh_addr += sizeof(sh);
        if (result != kStatus_Success) {
            return -4;
        }
        if (sh.type != FW_BLK_LOADABLE_SEGMENT) {
            continue;
        }
        if (calculate_image_crc(flash_addr + sh.offset, sh.len) != sh.crc) {
            return -5;
        }
    }

    return 0;
}

int ota_init(ota_upd_state_t* ota_state) {
    struct partition_entry* part = part_get_passive_partition_by_name("mcufw");
    if (!part) {
        LogError(("mcufw partition not found"));
        return -1;
    }
    LogInfo(("updating partition at %p, gen=%d", part->start, part->gen_level));

    if (mflash_drv_erase(part->start, part->size) != kStatus_Success) {
        return -1;
    }

    ota_state->part = part;
    ota_state->flash_addr = part->start;
    ota_state->bytes_stored = 0;
    return 0;
}

int ota_write_chunk(ota_upd_state_t* ota_state, const uint8_t* buf,
                    uint32_t len) {
    int32_t res = mflash_drv_write(ota_state->flash_addr, (uint32_t*)buf, len);
    if (res != kStatus_Success) {
        LogError(("flash write failed at addr=%x, len=%u",
                  ota_state->flash_addr, len));
        return -1;
    }
    ota_state->flash_addr += len;
    ota_state->bytes_stored += len;
    return 0;
}

int ota_finish(ota_upd_state_t* ota_state) {
    int res =
        validate_update_image(ota_state->part->start, ota_state->bytes_stored);
    if (res) {
        LogWarn(("invalid firmware image (%d)", res));
        return res;
    }
    ota_status = OTA_STATUS_UPLOADED;
    ota_state->part->gen_level = OTA_TEST_MODE_FLAG;
    res = part_write_layout();
    LogInfo(("new OTA test image uploaded (%d)", res));
    if (res == WM_SUCCESS) {
        LogInfo(("rebooting"));
        vTaskDelay(pdMS_TO_TICKS(250));
        reboot();
    }
    return res;
}

void check_ota_test_image() {
    struct partition_entry* part = part_get_active_partition_by_name("mcufw");

    if (part->gen_level & OTA_TEST_MODE_FLAG) {
        ota_status = OTA_STATUS_TESTING;
        part->gen_level = 0;
        part_write_layout();
        LogInfo(("OTA test image running"));
    }
}

int ota_promote_image() {
    if (ota_status == OTA_STATUS_TESTING) {
        struct partition_entry* part_prod =
            part_get_active_partition_by_name("mcufw");
        // running test image now has gen_level 0
        struct partition_entry* part_test =
            part_get_passive_partition_by_name("mcufw");
        part_test->gen_level = part_prod->gen_level + 1;
        int res = part_write_layout();
        LogInfo(("promoted OTA test image with new gen_level %d, result=%d",
                 part_test->gen_level, res));
        ota_status = OTA_STATUS_NONE;
        set_ota_led_pattern(LED_GREEN, LED_GREEN, LED_OFF, LED_OFF);
        return res;
    }
    return -1;
}
