#pragma once

#include <stdbool.h>

#include "FreeRTOS.h"
#include "FreeRTOS_DNS_Globals.h"

#ifdef __cplusplus
extern "C" {
#endif

void matter_mdns_init(void);

int matter_mdns_add_hostname(const char* hostname, const char* ipv6,
                             const char* ipv4);

int matter_mdns_add_service(const char* service, const char* proto, int port,
                            const char* txt_record, const char* instance,
                            const char* hostname);

int matter_mdns_add_subtype(const char* service, const char* proto,
                            const char* instance, const char* hostname,
                            const char* subtype);

int matter_mdns_remove_service(const char* service, const char* proto,
                               const char* instance, const char* hostname);

UBaseType_t matter_mdns_snapshot(DNSRecord_t** out);
UBaseType_t matter_mdns_get_view(DNSRecord_t** out);

void matter_mdns_announce(void);

/* Returns true if a service add/remove since the last call requested a
 * proactive re-announce. Used by matter_task to debounce announce spawns. */
bool matter_mdns_take_announce_pending(void);

#ifdef __cplusplus
}
#endif
