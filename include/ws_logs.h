#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef CONFIG_BOARD_NATIVE_SIM
#define WS_LOGS_PORT 8081
#else
#define WS_LOGS_PORT 8080
#endif

#ifdef __cplusplus
extern "C" {
#endif

void ws_logs_init(void);
void ws_logs_broadcast(const char* data, size_t len);
bool ws_logs_has_clients(void);

#ifdef __cplusplus
}
#endif
