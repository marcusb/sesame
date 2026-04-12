#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_config.pb.h"
#include "harness.h"
#include "logging.h"
#include "syslog.h"
#include "task.h"

static void on_cmd(const char *line) {
    /*
     * Commands understood by host_test_syslog:
     *   cfg HOST PORT            configure_logging() → register log_syslog backend
     *   log LEVEL MSG            emit a log record at LEVEL
     *
     * LEVEL ∈ {error, warn, info, debug}
     */
    if (strncmp(line, "cfg ", 4) == 0) {
        SyslogConfig cfg = {0};
        cfg.enabled = true;
        unsigned port = 514;
        if (sscanf(line + 4, "%63s %u", cfg.syslog_host, &port) == 2) {
            cfg.syslog_port = (uint16_t)port;
            configure_logging(&cfg);
            register_log_backend(log_syslog);
        }
    } else if (strncmp(line, "log ", 4) == 0) {
        const char *p = line + 4;
        const char *msg = strchr(p, ' ');
        if (!msg) return;
        char level[8];
        size_t ll = msg - p;
        if (ll >= sizeof(level)) ll = sizeof(level) - 1;
        memcpy(level, p, ll);
        level[ll] = '\0';
        msg++;
        if (strcmp(level, "error") == 0)      vLoggingPrintfError("%s", msg);
        else if (strcmp(level, "warn") == 0)  vLoggingPrintfWarn("%s", msg);
        else if (strcmp(level, "info") == 0)  vLoggingPrintfInfo("%s", msg);
        else if (strcmp(level, "debug") == 0) vLoggingPrintfDebug("%s", msg);
    }
}

int main(void) {
    /* No guest-side port; syslog is outbound UDP. libslirp NATs it natively,
       so guest traffic to 10.0.2.2:<port> arrives at host 127.0.0.1:<port>. */
    harness_init();
    harness_on_cmd(on_cmd);
    harness_run();
    return 0;
}
