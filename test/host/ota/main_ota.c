#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "api.pb.h"
#include "controller.h"
#include "harness.h"
#include "ota.h"
#include "task.h"

static QueueHandle_t ota_queue;

static void start_ota(void) {
    xTaskCreate(ota_task, "OTA", 4096, ota_queue, tskIDLE_PRIORITY + 3, NULL);
}

static void on_cmd(const char* line) {
    if (strncmp(line, "upgrade ", 8) == 0) {
        ota_msg_t msg = {.cmd = OTA_CMD_UPGRADE};
        strncpy(msg.msg.upgrade_msg.url, line + 8,
                sizeof(msg.msg.upgrade_msg.url) - 1);
        xQueueSend(ota_queue, &msg, 0);
    } else if (strncmp(line, "promote", 7) == 0) {
        ota_msg_t msg = {.cmd = OTA_CMD_PROMOTE};
        xQueueSend(ota_queue, &msg, 0);
    }
}

int main(void) {
    ota_queue = xQueueCreate(4, sizeof(ota_msg_t));
    if (!ota_queue) {
        fprintf(stderr, "failed to create ota_queue\n");
        return 1;
    }

    harness_init();
    harness_on_cmd(on_cmd);
    harness_set_guest_port(0, 80);
    start_ota();
    harness_run();
    return 0;
}
