#pragma once

#include "app_config.pb.h"
#include "logging.h"

void configure_logging(const SyslogConfig *cfg);
void log_syslog(const log_msg_t *log);
