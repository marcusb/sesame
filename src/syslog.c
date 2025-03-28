// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

// Application
#include "config_manager.h"
#include "syslog.h"

static struct freertos_sockaddr udp_log_addr;
static Socket_t syslog_sock;

static void create_log_socket(void *p1, uint32_t p2) {
    const SyslogConfig *cfg = (const SyslogConfig *)p1;
    udp_log_addr = (struct freertos_sockaddr){
        sizeof(struct freertos_sockaddr),
        FREERTOS_AF_INET,
        FreeRTOS_htons(cfg->syslog_port),
        0,
        {.ulIP_IPv4 = FreeRTOS_inet_addr(cfg->syslog_host)}};

    Socket_t sock = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM,
                                    FREERTOS_IPPROTO_UDP);
    wm_printf("socket created %p\n", sock);
    if (sock != FREERTOS_INVALID_SOCKET) {
        static const TickType_t send_to = pdMS_TO_TICKS(0);
        FreeRTOS_setsockopt(sock, 0, FREERTOS_SO_SNDTIMEO, &send_to,
                            sizeof(send_to));
        FreeRTOS_bind(sock, NULL, 0);
        syslog_sock = sock;
        wm_printf("socket bound\n");
    }
}

void configure_logging(const SyslogConfig *cfg) {
    wm_printf("enabled: %d, host: %s, port: %u", cfg->enabled, cfg->syslog_host,
              cfg->syslog_port);
    if (!cfg->enabled) {
        return;
    }

    // Schedule socket creation because this function is called from the IP task
    // and the IP task cannot itself wait for a socket to bind.
    xTimerPendFunctionCall(create_log_socket, (void *)cfg, 0, 10);
}

static void log_syslog(const char *msg) {
    if (syslog_sock != FREERTOS_INVALID_SOCKET) {
        FreeRTOS_sendto(syslog_sock, msg, strlen(msg), 0, &udp_log_addr,
                        sizeof(udp_log_addr));
    }
}

void prvLoggingTask(void *params) {
    QueueHandle_t xQueue = (QueueHandle_t)params;
    char *msg;

    for (;;) {
        /* Block to wait for the next string to print. */
        if (xQueueReceive(xQueue, &msg, portMAX_DELAY) == pdPASS) {
            wm_printf(msg);
            log_syslog(msg);
            vPortFree(msg);
        }
    }
}
