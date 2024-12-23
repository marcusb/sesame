#include <ctype.h>
#include <stdint.h>
#include <string.h>

#include "app_logging.h"

static char *print_ascii(char *p, const uint8_t *data, const uint8_t *end) {
    for (int i = 0; data < end; i++, data++) {
        if (i == 8) {
            *p++ = ' ';
        }
        *p++ = isprint(*data) ? *data : '.';
    }
    return p;
}

void debug_hexdump(char dir, const uint8_t *data, unsigned len) {
    static char s[4 * 16 + 5];
    char *p = s;

    if (len == 0) {
        return;
    }
    const uint8_t *line = data;
    p += sprintf(p, "%c: %02x ", dir, data[0]);
    int i;
    for (i = 1; i < len; i++) {
        p += sprintf(p, "%02x ", data[i]);
        if ((i & 0x0f) == 0x0f) {
            *p++ = ' ';
            p = print_ascii(p, line, data + i);
            *p = '\0';
            LogDebug((s));
            p = s;
            p += sprintf(p, "   ");
            line = data + i;
        }
    }
    int rem = i & 0x0f;
    if (rem > 0) {
        for (int j = 0; j < 3 * (16 - rem) + 1; j++) {
            *p++ = ' ';
        }
        p = print_ascii(p, line, data + i);
        *p = '\0';
        LogDebug((s));
    }
}
