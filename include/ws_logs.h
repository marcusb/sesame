#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/net/http/server.h>

#ifdef __cplusplus
extern "C" {
#endif

extern struct http_resource_detail_websocket ws_logs_resource_detail;

void ws_logs_init(void);
void ws_logs_broadcast(const char* data, size_t len);
bool ws_logs_has_clients(void);

#ifdef __cplusplus
}
#endif
