#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "controller.h"
#include "ota.h"

extern void host_inspector_emit(const char* line);

static uint32_t running_checksum;
static uint32_t bytes_written;
static int ota_finish_called;

void fake_ota_reset(void) {
    running_checksum = 0;
    bytes_written = 0;
    ota_finish_called = 0;
}

int fake_ota_was_finish_called(void) { return ota_finish_called; }

uint32_t fake_ota_get_checksum(void) { return running_checksum; }

uint32_t fake_ota_get_bytes_written(void) { return bytes_written; }

int ota_init(ota_upd_state_t* ota_state) {
    fake_ota_reset();
    return 0;
}

int ota_write_chunk(ota_upd_state_t* ota_state, const uint8_t* buf,
                    uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        running_checksum = running_checksum ^ (buf[i] << ((i % 4) * 8));
    }
    bytes_written += len;
    return 0;
}

int ota_finish(ota_upd_state_t* ota_state) {
    ota_finish_called = 1;
    char line[64];
    snprintf(line, sizeof(line), "OTA checksum=0x%x bytes=%u", running_checksum,
             bytes_written);
    host_inspector_emit(line);
    return 0;
}

ota_status_t ota_status = OTA_STATUS_NONE;

void check_ota_test_image() {}

int ota_promote_image() { return 0; }
