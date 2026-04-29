#include "harness.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "FreeRTOS_ARP.h"
#include "FreeRTOS_IP.h"
#include "NetworkInterface.h"
#include "controller.h"
#include "logging.h"
#include "logging_levels.h"
#include "queue.h"
#include "task.h"

/* Network parameters for QEMU (SLIRP default) */
static const uint8_t default_ip_addr[4] = {10, 0, 2, 15};
static const uint8_t netmask[4] = {255, 255, 255, 0};
static const uint8_t gateway_addr[4] = {10, 0, 2, 2};
static const uint8_t dns_server_addr[4] = {10, 0, 2, 3};
static const uint8_t hwaddr[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

static NetworkInterface_t eth_iface;

uint32_t SystemCoreClock = 25000000;

/* Globals the SUT expects from the embedded firmware. */
QueueHandle_t ctrl_queue;

/* Strong stubs for DNS hooks required by FreeRTOS+TCP */
DNSRecord_t* xApplicationDNSRecordQueryHook_Multi(
    struct xNetworkEndPoint* pxEndPoint, UBaseType_t* outLen) {
    (void)pxEndPoint;
    *outLen = 0;
    return NULL;
}
DNSRecord_t* xApplicationDNSRecordQueryHook(UBaseType_t* outLen) {
    *outLen = 0;
    return NULL;
}
void xApplicationDNSRecordsMatchedHook(void) {}

static harness_net_up_cb net_up_cb;
static harness_cmd_handler_t cmd_handler;
static bool network_up;

static char level_char(uint8_t level) {
    switch (level) {
        case LOG_ERROR:
            return 'E';
        case LOG_WARN:
            return 'W';
        case LOG_INFO:
            return 'I';
        case LOG_DEBUG:
            return 'D';
        default:
            return '-';
    }
}

void log_stdout_backend(const log_msg_t* log) {
    vTaskSuspendAll();
    printf("SUT: %c [%s] %s\n", level_char(log->level), log->task_name,
           log->msg ? log->msg : "");
    fflush(stdout);
    xTaskResumeAll();
}

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

void host_inspector_emit(const char* line) {
    vTaskSuspendAll();
    printf("\nINSPECTOR:%s\n", line);
    fflush(stdout);
    xTaskResumeAll();
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

void harness_on_network_up(harness_net_up_cb cb) { net_up_cb = cb; }
void harness_on_cmd(harness_cmd_handler_t h) { cmd_handler = h; }

static void cmd_task(void* arg) {
    (void)arg;
    char buf[1024];
    int idx = 0;

    printf("harness: cmd_task started\n");
    fflush(stdout);
    for (;;) {
        char c;
        int res = read(0, &c, 1);
        if (res > 0) {
            if (c == '\n' || c == '\r') {
                if (idx > 0) {
                    buf[idx] = '\0';
                    printf("harness: received cmd: %s\n", buf);
                    fflush(stdout);
                    if (cmd_handler) cmd_handler(buf);
                    idx = 0;
                }
            } else if (idx < (int)sizeof(buf) - 1) {
                buf[idx++] = c;
            }
            vTaskDelay(1); /* Yield if we are busy-looping on garbage! */
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void vApplicationIPNetworkEventHook_Multi(eIPCallbackEvent_t eNetworkEvent,
                                          struct xNetworkEndPoint* ep) {
    if (eNetworkEvent == eNetworkUp && !network_up && ep != NULL) {
        network_up = true;
        uint32_t ip, mask, gw, dns;
        FreeRTOS_GetEndPointConfiguration(&ip, &mask, &gw, &dns, ep);
        if (gw != 0) {
            FreeRTOS_OutputARPRequest(gw);
        }
        printf("READY host=127.0.0.1 port=0 inspector=0 guest_ip=%u.%u.%u.%u\n",
               (unsigned)(ip & 0xff), (unsigned)((ip >> 8) & 0xff),
               (unsigned)((ip >> 16) & 0xff), (unsigned)((ip >> 24) & 0xff));
        fflush(stdout);
        if (net_up_cb) net_up_cb();
    }
}

void configure_netif() {
    static NetworkEndPoint_t eps[1];
    extern NetworkInterface_t* pxMPS2_FillInterfaceDescriptor(
        BaseType_t xEMACIndex, NetworkInterface_t * pxInterface);
    pxMPS2_FillInterfaceDescriptor(0, &eth_iface);

    /* Set Ethernet IRQ (13) priority to something safe for FreeRTOS */
    *(volatile uint32_t*)(0xE000E400 + 3 * 4) = (0xE0 << 8);

    FreeRTOS_FillEndPoint(&eth_iface, &eps[0], default_ip_addr, netmask,
                          gateway_addr, dns_server_addr, hwaddr);
    eps[0].bits.bWantDHCP = pdTRUE;

    int res = FreeRTOS_IPInit_Multi();
    configASSERT(res);
}

void harness_init(void) {
    ctrl_queue = xQueueCreate(16, sizeof(ctrl_msg_t));
    configASSERT(ctrl_queue);

    init_logging(2048, tskIDLE_PRIORITY + 1, 32);
    register_log_backend(log_stdout_backend);

    xTaskCreate(inspector_task, "Inspector", 2048, NULL, tskIDLE_PRIORITY + 1,
                NULL);
    xTaskCreate(cmd_task, "HostCmd", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
}

void harness_run(void) {
    configure_netif();
    vTaskStartScheduler();
    for (;;);
}

void vApplicationMallocFailedHook(void) { abort(); }
void vApplicationIdleHook(void) {}
void vApplicationStackOverflowHook(TaskHandle_t task, char* name) {
    (void)task;
    (void)name;
    abort();
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
const char* pcApplicationHostnameHook(void) { return "sesame-it"; }
void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent) {
    vApplicationIPNetworkEventHook_Multi(eNetworkEvent, NULL);
}

extern unsigned __HeapBase, __HeapLimit;
void setup_heap() {
    unsigned heap_size = (unsigned)&__HeapLimit - (unsigned)&__HeapBase;
    HeapRegion_t xHeapRegions[] = {{(uint8_t*)&__HeapBase, heap_size},
                                   {NULL, 0}};
    vPortDefineHeapRegions(xHeapRegions);
}
