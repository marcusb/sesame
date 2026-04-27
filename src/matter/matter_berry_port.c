/*
 * Minimal Berry VM port for sesame.
 *
 * Provides be_writebuffer / be_readstring and a sesame-specific module table.
 * File I/O (be_fopen/etc) lives in matter_tasmota_shim.c and is PSM-backed.
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_logging.h"
#include "berry.h"
#include "fsl_debug_console.h"
#include "task.h"

#if BE_USE_STRING_MODULE
be_extern_native_module(string);
#endif
#if BE_USE_JSON_MODULE
be_extern_native_module(json);
#endif
#if BE_USE_MATH_MODULE
be_extern_native_module(math);
#endif
#if BE_USE_TIME_MODULE
be_extern_native_module(time);
#endif
#if BE_USE_OS_MODULE
be_extern_native_module(os);
#endif
#if BE_USE_GLOBAL_MODULE
be_extern_native_module(global);
#endif
#if BE_USE_SYS_MODULE
be_extern_native_module(sys);
#endif
#if BE_USE_DEBUG_MODULE
be_extern_native_module(debug);
#endif
#if BE_USE_GC_MODULE
be_extern_native_module(gc);
#endif
#if BE_USE_SOLIDIFY_MODULE
be_extern_native_module(solidify);
#endif
#if BE_USE_INTROSPECT_MODULE
be_extern_native_module(introspect);
#endif
#if BE_USE_STRICT_MODULE
be_extern_native_module(strict);
#endif
be_extern_native_module(undefined);
be_extern_native_module(cb);

be_extern_native_module(tasmota);
be_extern_native_module(persist);
be_extern_native_module(path);
be_extern_native_module(sesame);
be_extern_native_module(mdns);
be_extern_native_module(matter);

be_extern_native_class(bytes);
be_extern_native_class(list);
be_extern_native_class(map);
be_extern_native_class(range);
be_extern_native_class(re_pattern);
be_extern_native_class(int64);
be_extern_native_class(udp);
be_extern_native_class(Matter_Counter);
be_extern_native_class(Matter_Verhoeff);

/*
 * NOTE: The 'crypto' module (and EC_P256 class) are loaded dynamically
 * via be_load_crypto_module() instead of being listed in the static tables
 * below. This is done to avoid complex static macro definitions for
 * non-solidified modules and to ensure proper stack management during module
 * construction, which was previously causing memory corruption and hangs during
 * EC operations.
 */

const bntvmodule_t* const be_module_table[] = {
#if BE_USE_STRING_MODULE
    &be_native_module(string),
#endif
#if BE_USE_JSON_MODULE
    &be_native_module(json),
#endif
#if BE_USE_MATH_MODULE
    &be_native_module(math),
#endif
#if BE_USE_TIME_MODULE
    &be_native_module(time),
#endif
#if BE_USE_OS_MODULE
    &be_native_module(os),
#endif
#if BE_USE_GLOBAL_MODULE
    &be_native_module(global),
#endif
#if BE_USE_SYS_MODULE
    &be_native_module(sys),
#endif
#if BE_USE_DEBUG_MODULE
    &be_native_module(debug),
#endif
#if BE_USE_GC_MODULE
    &be_native_module(gc),
#endif
#if BE_USE_SOLIDIFY_MODULE
    &be_native_module(solidify),
#endif
#if BE_USE_INTROSPECT_MODULE
    &be_native_module(introspect),
#endif
#if BE_USE_STRICT_MODULE
    &be_native_module(strict),
#endif
    &be_native_module(undefined),
    &be_native_module(cb),
    &be_native_module(tasmota),
    &be_native_module(persist),
    &be_native_module(path),
    &be_native_module(sesame),
    &be_native_module(mdns),
    &be_native_module(matter),
    NULL};

bclass_array be_class_table = {&be_native_class(bytes),
                               &be_native_class(list),
                               &be_native_class(map),
                               &be_native_class(range),
                               &be_native_class(int64),
                               &be_native_class(udp),
                               &be_native_class(Matter_Counter),
                               &be_native_class(Matter_Verhoeff),
                               NULL};

/* Berry's print() lands here in fragments — the formatted arguments
 * followed by a separate "\n". Line-buffer until newline so each print
 * becomes a single LogInfo entry instead of raw PUTCHARs that race
 * with the logging task on the UART.
 *
 * Guards: if the scheduler isn't running yet, or we're inside an ISR,
 * or the logger would recurse back into us, fall back to direct
 * PUTCHAR — log_prepare needs xQueueSend/pvPortMalloc, neither safe
 * in those contexts. */
BERRY_API void be_writebuffer(const char* buffer, size_t length) {
    static char line[160];
    static size_t pos = 0;
    static int in_log = 0;

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED ||
        xPortIsInsideInterrupt() || in_log) {
        for (size_t i = 0; i < length; i++) PUTCHAR(buffer[i]);
        return;
    }

    for (size_t i = 0; i < length; i++) {
        char c = buffer[i];
        if (c == '\n') {
            line[pos] = '\0';
            in_log = 1;
            LogInfo(("%s", line));
            in_log = 0;
            pos = 0;
        } else if (c == '\r') {
            /* drop */
        } else if (pos + 1 < sizeof(line)) {
            line[pos++] = c;
        } else {
            line[pos] = '\0';
            in_log = 1;
            LogInfo(("%s", line));
            in_log = 0;
            pos = 0;
            line[pos++] = c;
        }
    }
}

BERRY_API char* be_readstring(char* buffer, size_t size) {
    (void)buffer;
    (void)size;
    return NULL;
}
