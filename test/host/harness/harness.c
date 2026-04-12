#include "harness.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "FreeRTOS_ARP.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "controller.h"
#include "logging.h"
#include "queue.h"
#include "task.h"

/* Network parameters — unused when DHCP is enabled, but FreeRTOS_IPInit still
   needs something to hand back on DHCP failure. */
static const uint8_t ucIPAddress[4] = {10, 0, 2, 15};
static const uint8_t ucNetMask[4] = {255, 255, 255, 0};
static const uint8_t ucGatewayAddress[4] = {10, 0, 2, 2};
static const uint8_t ucDNSServerAddress[4] = {10, 0, 2, 3};
static const uint8_t ucMACAddress[6] = {0x52, 0x54, 0x00, 0x12, 0x34, 0x56};

/* Globals the SUT expects from the embedded firmware. */
QueueHandle_t ctrl_queue;

/* POSIX socket helpers in host_sockets.c (separate TU to avoid
   FreeRTOS_Sockets.h macro collisions). */
extern void host_inspector_open(void);
extern void host_inspector_emit(const char *line);
extern int host_cmd_open(int port);
extern int host_cmd_recv(char *buf, int size);

static harness_net_up_cb net_up_cb;
static harness_cmd_handler_t cmd_handler;
static bool network_up;

extern void log_stdout_backend(const log_msg_t *log);

static const char* ctrl_msg_name(ctrl_msg_type_t t) {
    switch (t) {
        case CTRL_MSG_UNKNOWN:
            return "UNKNOWN";
        case CTRL_MSG_OTA_UPGRADE:
            return "OTA_UPGRADE";
        case CTRL_MSG_OTA_PROMOTE:
            return "OTA_PROMOTE";
        case CTRL_MSG_DOOR_CONTROL:
            return "DOOR_CONTROL";
        case CTRL_MSG_DOOR_STATE_UPDATE:
            return "DOOR_STATE_UPDATE";
        case CTRL_MSG_WIFI_BUTTON:
            return "WIFI_BUTTON";
        case CTRL_MSG_OTA_BUTTON:
            return "OTA_BUTTON";
        case CTRL_MSG_WIFI_CONFIG:
            return "WIFI_CONFIG";
        case CTRL_MSG_MQTT_CONFIG:
            return "MQTT_CONFIG";
        case CTRL_MSG_LOGGING_CONFIG:
            return "LOGGING_CONFIG";
    }
    return "?";
}

static void inspector_task(void* arg) {
    (void)arg;
    ctrl_msg_t msg;
    char line[256];
    for (;;) {
        if (xQueueReceive(ctrl_queue, &msg, portMAX_DELAY) != pdPASS) continue;
        int n =
            snprintf(line, sizeof(line), "CTRL %s", ctrl_msg_name(msg.type));
        switch (msg.type) {
            case CTRL_MSG_DOOR_CONTROL:
                snprintf(line + n, sizeof(line) - n, " cmd=%d",
                         (int)msg.msg.door_control.command);
                break;
            case CTRL_MSG_MQTT_CONFIG:
                snprintf(line + n, sizeof(line) - n,
                         " enabled=%d host=%s port=%u prefix=%s",
                         msg.msg.mqtt_cfg.enabled, msg.msg.mqtt_cfg.broker_host,
                         (unsigned)msg.msg.mqtt_cfg.broker_port,
                         msg.msg.mqtt_cfg.prefix);
                break;
            case CTRL_MSG_WIFI_CONFIG:
                snprintf(line + n, sizeof(line) - n,
                         " hostname=%s ssid=%s security=%d",
                         msg.msg.network_cfg.hostname, msg.msg.network_cfg.ssid,
                         (int)msg.msg.network_cfg.security);
                break;
            case CTRL_MSG_LOGGING_CONFIG:
                snprintf(
                    line + n, sizeof(line) - n,
                    " syslog_enabled=%d host=%s port=%u",
                    msg.msg.logging_cfg.syslog_config.enabled,
                    msg.msg.logging_cfg.syslog_config.syslog_host,
                    (unsigned)msg.msg.logging_cfg.syslog_config.syslog_port);
                break;
            default:
                break;
        }
        host_inspector_emit(line);
    }
}


void harness_set_guest_port(int udp, uint16_t guest_port) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", guest_port);
    setenv("HARNESS_GUEST_PORT", buf, 1);
    setenv("HARNESS_GUEST_UDP", udp ? "1" : "0", 1);
}

void harness_on_network_up(harness_net_up_cb cb) { net_up_cb = cb; }
void harness_on_cmd(harness_cmd_handler_t h) { cmd_handler = h; }

static void cmd_task(void *arg) {
    (void)arg;
    char buf[1024];
    for (;;) {
        int n = host_cmd_recv(buf, sizeof(buf) - 1);
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        buf[n] = '\0';
        /* Strip trailing newline so handlers get a clean string. */
        while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) {
            buf[--n] = '\0';
        }
        if (cmd_handler) cmd_handler(buf);
    }
}


void vApplicationIPNetworkEventHook_Multi(eIPCallbackEvent_t eNetworkEvent,
                                          struct xNetworkEndPoint* ep) {
    (void)ep;
    if (eNetworkEvent == eNetworkUp && !network_up) {
        network_up = true;
        uint32_t ip, mask, gw, dns;
        FreeRTOS_GetAddressConfiguration(&ip, &mask, &gw, &dns);
        /* Prime the ARP cache for the slirp gateway so the first outbound
           UDP from the SUT isn't dropped pending resolution. */
        if (gw != 0) {
            FreeRTOS_OutputARPRequest(gw);
        }
        const char* host = getenv("HARNESS_HOST_HOST");
        const char* hp = getenv("HARNESS_HOST_PORT");
        const char* ip_p = getenv("HARNESS_INSPECTOR_PORT");
        fprintf(stdout,
                "READY host=%s port=%s inspector=%s guest_ip=%u.%u.%u.%u\n",
                host ? host : "127.0.0.1", hp ? hp : "?", ip_p ? ip_p : "?",
                (unsigned)(ip & 0xff), (unsigned)((ip >> 8) & 0xff),
                (unsigned)((ip >> 16) & 0xff), (unsigned)((ip >> 24) & 0xff));
        fflush(stdout);
        if (net_up_cb) net_up_cb();
    }
}

void harness_init(void) {
    ctrl_queue = xQueueCreate(16, sizeof(ctrl_msg_t));
    configASSERT(ctrl_queue);

    init_logging(2048, tskIDLE_PRIORITY + 1, 32);
    register_log_backend(log_stdout_backend);

    host_inspector_open();
    xTaskCreate(inspector_task, "Inspector", 2048, NULL, tskIDLE_PRIORITY + 1,
                NULL);

    const char *p = getenv("HARNESS_CMD_PORT");
    if (p && host_cmd_open(atoi(p)) >= 0) {
        xTaskCreate(cmd_task, "HostCmd", 4096, NULL, tskIDLE_PRIORITY + 1,
                    NULL);
    }
}

void harness_run(void) {
    FreeRTOS_IPInit(ucIPAddress, ucNetMask, ucGatewayAddress,
                    ucDNSServerAddress, ucMACAddress);
    vTaskStartScheduler();
    /* Never reached. */
    for (;;) sleep(1);
}

/* ---- FreeRTOS hooks required by the kernel/port -------------------------- */

void vApplicationMallocFailedHook(void) {
    fprintf(stderr, "harness: malloc failed\n");
    abort();
}

void vApplicationIdleHook(void) {}

void vApplicationStackOverflowHook(TaskHandle_t task, char* name) {
    (void)task;
    fprintf(stderr, "harness: stack overflow in %s\n", name);
    abort();
}

void vApplicationGetIdleTaskMemory(
    StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer,
    configSTACK_DEPTH_TYPE* pulIdleTaskStackSize) {
    static StaticTask_t tcb;
    static StackType_t stack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer = &tcb;
    *ppxIdleTaskStackBuffer = stack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(
    StaticTask_t** ppxTimerTaskTCBBuffer, StackType_t** ppxTimerTaskStackBuffer,
    configSTACK_DEPTH_TYPE* pulTimerTaskStackSize) {
    static StaticTask_t tcb;
    static StackType_t stack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer = &tcb;
    *ppxTimerTaskStackBuffer = stack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

BaseType_t xApplicationGetRandomNumber(uint32_t* pulNumber) {
    *pulNumber = (uint32_t)rand();
    return pdTRUE;
}

uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress,
                                            uint16_t usSourcePort,
                                            uint32_t ulDestinationAddress,
                                            uint16_t usDestinationPort) {
    (void)ulSourceAddress;
    (void)usSourcePort;
    (void)ulDestinationAddress;
    (void)usDestinationPort;
    return (uint32_t)rand();
}

const char* pcApplicationHostnameHook(void) { return "sesame-host-test"; }

size_t xPortGetFreeHeapSize(void) { return configTOTAL_HEAP_SIZE; }
size_t xPortGetMinimumEverFreeHeapSize(void) { return configTOTAL_HEAP_SIZE; }

void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent) {
    vApplicationIPNetworkEventHook_Multi(eNetworkEvent, NULL);
}
