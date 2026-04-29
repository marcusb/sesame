#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_config.pb.h"
#include "controller.h"
#include "harness.h"
#include "idcm_msg.h"
#include "mqtt.h"
#include "task.h"

static MqttConfig mqtt_cfg;

static void start_mqtt(void) {
    xTaskCreate(mqtt_task, "MQTT", 4096, &mqtt_cfg, tskIDLE_PRIORITY + 3, NULL);
}

static void on_cmd(const char* line) {
    /*
     * Commands understood by mqtt_it:
     *   start HOST PORT PREFIX   configure + launch mqtt_task
     *   pubstate STATE DIR POS   call publish_state() with an update
     *
     *   STATE: 0=closed 1=open
     *   DIR:   0=down 1=up 2=stopped
     */
    if (strncmp(line, "start ", 6) == 0) {
        memset(&mqtt_cfg, 0, sizeof(mqtt_cfg));
        mqtt_cfg.enabled = true;
        unsigned port = 0;
        char host[64];
        char prefix[32];
        if (sscanf(line + 6, "%63s %u %31s", host, &port, prefix) == 3) {
            strncpy(mqtt_cfg.broker_host, host,
                    sizeof(mqtt_cfg.broker_host) - 1);
            mqtt_cfg.broker_port = (uint16_t)port;
            strncpy(mqtt_cfg.prefix, prefix, sizeof(mqtt_cfg.prefix) - 1);
            strncpy(mqtt_cfg.client_id, prefix, sizeof(mqtt_cfg.client_id) - 1);
            start_mqtt();
        }
    } else if (strncmp(line, "pubstate ", 9) == 0) {
        int state = 0, dir = 0, pos = 0;
        if (sscanf(line + 9, "%d %d %d", &state, &dir, &pos) == 3) {
            door_state_msg_t msg = {
                .state = (door_open_state_t)state,
                .direction = (door_direction_t)dir,
                .pos = pos,
            };
            publish_state(&msg);
        }
    }
}

int main(void) {
    setup_heap();
    harness_init();
    harness_on_cmd(on_cmd);
    harness_run();
    return 0;
}
