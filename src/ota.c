#include "ota.h"

#include <bootutil/boot_status.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/retention/blinfo.h>

#include "app_logging.h"

// application
#include <bootutil/bootutil_public.h>
#include <zephyr/dfu/flash_img.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/reboot.h>

#include "controller.h"

LOG_MODULE_REGISTER(ota, LOG_LEVEL_INF);

// application
#include "controller.h"
#include "leds.h"

ota_status_t ota_status = OTA_STATUS_NONE;

#include <core_cm4.h>

#include "88MW320.h"

int my_boot_fetch_active_slot() {
    if (SCB->VTOR >= 0x20000000) {
        return 2;  // RAM load
    }
    if (FLASHC->FAOFFR > DT_REG_ADDR(DT_NODELABEL(slot0_partition))) {
        return 1;  // Slot 1
    }
    return 0;  // Slot 0
}

int ota_init(ota_upd_state_t* ota_state) {
    LOG_INF("Initializing OTA update");

    int active_slot = my_boot_fetch_active_slot();

    uint8_t target_area_id;
    if (active_slot == 1) {
        target_area_id = PARTITION_ID(slot0_partition);
    } else {
        target_area_id = PARTITION_ID(slot1_partition);
    }

    int res = flash_img_init_id(&ota_state->ctx, target_area_id);
    if (res < 0) {
        LOG_ERR("flash_img_init_id failed: %d", res);
    }
    return res;
}

int ota_write_chunk(ota_upd_state_t* ota_state, const uint8_t* buf,
                    uint32_t len) {
    int res = flash_img_buffered_write(&ota_state->ctx, buf, len, false);
    if (res < 0) {
        LOG_ERR("flash_img_buffered_write failed: %d", res);
        return res;
    }
    return 0;
}

int my_boot_request_upgrade(void) {
    int active_slot = my_boot_fetch_active_slot();
    uint8_t target_area_id;
    if (active_slot == 1) {
        target_area_id = PARTITION_ID(slot0_partition);
    } else {
        target_area_id = PARTITION_ID(slot1_partition);
    }

    const struct flash_area* fap;
    int rc = flash_area_open(target_area_id, &fap);
    if (rc != 0) {
        return rc;
    }

    rc = boot_set_next(fap, false, false);

    flash_area_close(fap);
    return rc;
}

int ota_finish(ota_upd_state_t* ota_state) {
    int res = flash_img_buffered_write(&ota_state->ctx, NULL, 0, true);
    if (res < 0) {
        LOG_ERR("flash_img_buffered_write (flush) failed: %d", res);
        return res;
    }

    res = my_boot_request_upgrade();
    if (res) {
        LOG_ERR("boot_request_upgrade failed: %d", res);
        return res;
    }

    ota_status = OTA_STATUS_UPLOADED;
    LOG_INF("new OTA test image uploaded successfully. rebooting");
    k_msleep(250);
    sys_reboot(SYS_REBOOT_COLD);
    return res;
}

extern int boot_read_swap_state(const struct flash_area* fap,
                                struct boot_swap_state* state);

bool my_boot_is_img_confirmed(void) {
    int active_slot = my_boot_fetch_active_slot();
    uint8_t target_area_id;
    if (active_slot == 1) {
        target_area_id = PARTITION_ID(slot1_partition);
    } else {
        target_area_id = PARTITION_ID(slot0_partition);
    }
    const struct flash_area* fa;
    int rc = flash_area_open(target_area_id, &fa);
    if (rc) return false;
    struct boot_swap_state state;
    rc = boot_read_swap_state(fa, &state);
    flash_area_close(fa);
    if (rc != 0) return false;

    if (state.magic == BOOT_MAGIC_UNSET) {
        return true;
    }

    LOG_INF(
        "my_boot_is_img_confirmed: active_slot=%d, target_area_id=%d, "
        "magic=%x, image_ok=%x, copy_done=%x",
        active_slot, target_area_id, state.magic, state.image_ok,
        state.copy_done);
    return state.magic == BOOT_MAGIC_GOOD && state.image_ok == BOOT_FLAG_SET;
}

void check_ota_test_image() {
    if (!my_boot_is_img_confirmed()) {
        ota_status = OTA_STATUS_TESTING;
        LOG_INF("OTA test image running");
    }
}

int my_boot_set_confirmed(void) {
    int active_slot = my_boot_fetch_active_slot();
    uint8_t target_area_id;
    if (active_slot == 1) {
        target_area_id = PARTITION_ID(slot1_partition);
    } else {
        target_area_id = PARTITION_ID(slot0_partition);
    }

    const struct flash_area* fap;
    int rc = flash_area_open(target_area_id, &fap);
    if (rc != 0) {
        return rc;
    }

    rc = boot_set_next(fap, true, true);

    flash_area_close(fap);
    return rc;
}

int ota_promote_image() {
    if (ota_status == OTA_STATUS_TESTING) {
        int res = my_boot_set_confirmed();
        LOG_INF("promoted OTA test image, result=%d", res);
        ota_status = OTA_STATUS_NONE;
        set_ota_led_pattern(LED_GREEN, LED_GREEN, LED_OFF, LED_OFF);
        return res;
    }
    return -1;
}
