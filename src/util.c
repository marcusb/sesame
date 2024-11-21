#include <stdint.h>
#include <string.h>

#include "app_logging.h"

void debug_hexdump(char dir, const uint8_t *data, unsigned len) {
    static char s[3 * 16 + 4];
    char *p = s;

    if (len == 0) {
        return;
    }
    p += sprintf(p, "%c: %02x ", dir, data[0]);
    for (int i = 1; i < len; i++) {
        if ((i & 0x0f) == 0) {
            *p = '\0';
            LogDebug((s));
            p = s;
            p += sprintf(p, "   ");
        }
        p += sprintf(p, "%02x ", data[i]);
    }
    *p = '\0';
    LogDebug((s));
}
