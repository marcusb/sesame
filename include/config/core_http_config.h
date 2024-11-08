#ifndef _CORE_HTTP_CONFIG_H
#define _CORE_HTTP_CONFIG_H

#include "FreeRTOS.h"

#include "logging_levels.h"

#ifndef LIBRARY_LOG_NAME
    #define LIBRARY_LOG_NAME    "HTTP"
#endif

#ifndef LIBRARY_LOG_LEVEL
    #define LIBRARY_LOG_LEVEL    LOG_INFO
#endif

#include "logging_stack.h"

#define HTTP_USER_AGENT_VALUE    "sesame"

#endif
