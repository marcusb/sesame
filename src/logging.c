#include <stdarg.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

// Application
#include "logging.h"
#include "string_util.h"

#ifndef configPRINT_STRING
#error configPRINT_STRING( x ) must be defined in FreeRTOSConfig.h to use this logging file.  Set configPRINT_STRING( x ) to a function that outputs a string, where X is the string.  For example, #define configPRINT_STRING( x ) MyUARTWriteString( X )
#endif

#ifndef configLOGGING_MAX_MESSAGE_LENGTH
#error configLOGGING_MAX_MESSAGE_LENGTH must be defined in FreeRTOSConfig.h to use this logging file.  configLOGGING_MAX_MESSAGE_LENGTH sets the size of the buffer into which formatted text is written, so also sets the maximum log message length.
#endif

static QueueHandle_t log_queue = NULL;

#define MAX_LOG_BACKENDS 2
static log_backend_func log_backends[MAX_LOG_BACKENDS];

int register_log_backend(log_backend_func f) {
    for (int i = 0; i < MAX_LOG_BACKENDS; i++) {
        if (log_backends[i] == NULL) {
            log_backends[i] = f;
            return 0;
        }
    }
    return -1;
}

void logging_task(void *params) {
    QueueHandle_t xQueue = (QueueHandle_t)params;
    log_msg_t log;

    for (;;) {
        if (xQueueReceive(xQueue, &log, portMAX_DELAY) == pdPASS) {
            for (int i = 0; i < MAX_LOG_BACKENDS; i++) {
                if (log_backends[i] != NULL) {
                    log_backends[i](&log);
                }
            }
            vPortFree(log.msg);
        }
    }
}

BaseType_t init_logging(uint16_t stack_size, UBaseType_t priority,
                        UBaseType_t queue_length) {
    log_queue = xQueueCreate(queue_length, sizeof(log_msg_t));

    if (log_queue != NULL) {
        if (xTaskCreate(logging_task, "Logging", stack_size, log_queue,
                        priority, NULL) == pdPASS) {
            return pdPASS;
        } else {
            /* Could not create the task, so delete the queue again. */
            vQueueDelete(log_queue);
        }
    }
    return pdFAIL;
}

static void log_prepare(uint8_t log_level, const char *filename,
                        size_t line_num, const char *fmt, va_list args) {
    static unsigned long msg_id = 0;

    configASSERT(fmt != NULL);

    log_msg_t log = {++msg_id, log_level, filename, line_num,
                     xTaskGetTickCount()};

    const char *task_name;
    static const char *no_task = "-";
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        task_name = pcTaskGetName(NULL);
    } else {
        task_name = no_task;
    }
    strtcpy(log.task_name, task_name, sizeof(log.task_name));

    int n = vsnprintf(NULL, 0, fmt, args);
    if (n < 0) {
        return;
    }
    // allocate some extra for the filename info
    n += 24;
    char *p = log.msg = pvPortMalloc(n);
    if (p == NULL) {
        return;
    }

    if (filename != NULL) {
        const char *file;

        /* If a file path is provided, extract only the file name from the
         * string by looking for '/' or '\' directory seperator. */
        if (strrchr(filename, '\\') != NULL) {
            file = strrchr(filename, '\\') + 1;
        } else if (strrchr(filename, '/') != NULL) {
            file = strrchr(filename, '/') + 1;
        } else {
            file = filename;
        }

        int m = snprintf(p, n, "[%.12s:%d]", file, line_num);
        if (m >= 0 && m < n) {
            p += m;
            n -= m;
        }
    }
    vsnprintf(p, n, fmt, args);
    if (xQueueSend(log_queue, &log, 0) != pdPASS) {
        vPortFree(log.msg);
    }
}

void vLoggingPrintfError(const char *pcFormat, ...) {
    va_list args;
    va_start(args, pcFormat);
    log_prepare(LOG_ERROR, NULL, 0, pcFormat, args);
    va_end(args);
}

void vLoggingPrintfWarn(const char *pcFormat, ...) {
    va_list args;
    va_start(args, pcFormat);
    log_prepare(LOG_WARN, NULL, 0, pcFormat, args);
    va_end(args);
}

void vLoggingPrintfInfo(const char *pcFormat, ...) {
    va_list args;
    va_start(args, pcFormat);
    log_prepare(LOG_INFO, NULL, 0, pcFormat, args);
    va_end(args);
}

void vLoggingPrintfDebug(const char *pcFormat, ...) {
    va_list args;
    va_start(args, pcFormat);
    log_prepare(LOG_DEBUG, NULL, 0, pcFormat, args);
    va_end(args);
}

void vLoggingPrintfWithFileAndLine(const char *pcFile, size_t fileLineNo,
                                   const char *pcFormat, ...) {
    configASSERT(pcFile != NULL);
    va_list args;
    va_start(args, pcFormat);
    log_prepare(LOG_NONE, pcFile, fileLineNo, pcFormat, args);
    va_end(args);
}

/*!
 * \brief Formats a string to be printed and sends it
 * to the print queue.
 *
 * Appends the message number, time (in ticks), and task
 * that called vLoggingPrintf to the beginning of each
 * print statement.
 *
 */
void vLoggingPrintf(const char *pcFormat, ...) {
    va_list args;
    va_start(args, pcFormat);
    log_prepare(LOG_NONE, NULL, 0, pcFormat, args);
    va_end(args);
}

void vLoggingPrint(const char *pcMessage) { vLoggingPrintf(pcMessage); }
