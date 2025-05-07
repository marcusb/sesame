#include <time.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

// Application

#include "app_config.pb.h"
#include "logging.h"
#include "syslog.h"

#define LOG_FACILITY_LOCAL0 16

static struct freertos_sockaddr udp_log_addr;
static Socket_t syslog_sock;

static void do_configure(void *p1, uint32_t p2) {
    const SyslogConfig *cfg = (const SyslogConfig *)p1;

    uint32_t host_ip = FreeRTOS_gethostbyname(cfg->syslog_host);
    if (!host_ip) {
        return;
    }
    udp_log_addr = (struct freertos_sockaddr){sizeof(struct freertos_sockaddr),
                                              FREERTOS_AF_INET,
                                              FreeRTOS_htons(cfg->syslog_port),
                                              0,
                                              {.ulIP_IPv4 = host_ip}};

    Socket_t sock = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM,
                                    FREERTOS_IPPROTO_UDP);
    if (sock != FREERTOS_INVALID_SOCKET) {
        static const TickType_t send_to = pdMS_TO_TICKS(0);
        FreeRTOS_setsockopt(sock, 0, FREERTOS_SO_SNDTIMEO, &send_to,
                            sizeof(send_to));
        FreeRTOS_bind(sock, NULL, 0);
        syslog_sock = sock;
    }
}

void configure_logging(const SyslogConfig *cfg) {
    if (!cfg->enabled) {
        return;
    }

    // Schedule socket creation because this function is called from the IP task
    // and the IP task cannot itself wait for a socket to bind.
    xTimerPendFunctionCall(do_configure, (void *)cfg, 0, 100);
}

void log_syslog(const log_msg_t *log) {
    // allocate some extra space for metadata
    char buf[strlen(log->msg) + 64];
    static const int severities[] = {7, 3, 4, 6, 7};
    configASSERT(log->level < sizeof(severities));
    int severity = severities[log->level];
    int pri = (LOG_FACILITY_LOCAL0 << 3) + severity;

    char *p = buf;
    int remaining = sizeof(buf);
    int n = snprintf(p, remaining, "<%03d>1 ", pri);
    if (n < 0 || n >= remaining) {
        return;
    }
    p += n;
    remaining -= n;

    // include timestamp if it's realistic
    // struct tm tm;
    // if (wmtime_time_get(&tm) == 0 && tm.tm_year > 2024 &&
    //     (n = strftime(p, remaining, "%FT%T%z", &tm)) > 0) {
    //     p += n;
    //     remaining -= n;
    // } else {
        *p++ = '-';
        remaining--;
    // }

    n = snprintf(p, remaining, " %s sesame %s %lu - %s",
                 pcApplicationHostnameHook(), log->task_name, log->msg_id,
                 log->msg);
    if (n < 0) {
        return;
    }

    if (syslog_sock != FREERTOS_INVALID_SOCKET) {
        FreeRTOS_sendto(syslog_sock, buf, strlen(buf), 0, &udp_log_addr,
                        sizeof(udp_log_addr));
    }
}
