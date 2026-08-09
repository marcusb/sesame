#include "ota.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "app_logging.h"

// application
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

int ota_init(ota_upd_state_t* ota_state) {
    LOG_INF("Initializing OTA update");

    int active_slot = boot_fetch_active_slot();
    uint8_t target_area_id =
        FIXED_PARTITION_ID(slot1_partition);  // Default to slot 1

    if (active_slot == 1) {
        LOG_INF("Currently running from slot 1, will write update to slot 0");
        target_area_id = FIXED_PARTITION_ID(slot0_partition);
    } else {
        LOG_INF("Currently running from slot 0, will write update to slot 1");
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

int ota_finish(ota_upd_state_t* ota_state) {
    int res = flash_img_buffered_write(&ota_state->ctx, NULL, 0, true);
    if (res < 0) {
        LOG_ERR("flash_img_buffered_write (flush) failed: %d", res);
        return res;
    }

    res = boot_request_upgrade(BOOT_UPGRADE_TEST);
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

void check_ota_test_image() {
    if (!boot_is_img_confirmed()) {
        ota_status = OTA_STATUS_TESTING;
        LOG_INF("OTA test image running");
    }
}

int ota_promote_image() {
    if (ota_status == OTA_STATUS_TESTING) {
        int res = boot_write_img_confirmed();
        LOG_INF("promoted OTA test image, result=%d", res);
        ota_status = OTA_STATUS_NONE;
        set_ota_led_pattern(LED_GREEN, LED_GREEN, LED_OFF, LED_OFF);
        return res;
    }
    return -1;
}
