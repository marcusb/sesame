/*
 * Logging backend that writes one line per log record to stdout prefixed with
 * "SUT:" so pytest can distinguish SUT logs from harness/driver output.
 */

#include <stdio.h>

#include "logging.h"
#include "logging_levels.h"

static char level_char(uint8_t level) {
    switch (level) {
        case LOG_ERROR: return 'E';
        case LOG_WARN:  return 'W';
        case LOG_INFO:  return 'I';
        case LOG_DEBUG: return 'D';
        default:        return '-';
    }
}

void log_stdout_backend(const log_msg_t *log) {
    fprintf(stdout, "SUT: %c [%s] %s\n",
            level_char(log->level), log->task_name, log->msg ? log->msg : "");
    fflush(stdout);
}
