#pragma once

typedef enum {
    LOG_NONE = 0,
    LOG_ERROR = 1,
    LOG_WARN = 2,
    LOG_INFO = 3,
    LOG_DEBUG = 4,
    LOG_LEVEL_LAST = LOG_DEBUG
} log_level_t;
