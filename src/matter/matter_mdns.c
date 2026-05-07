/*
 * mDNS / DNS-SD logical state store backing the Matter stack and Berry shim.
 */

#include "matter_mdns.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_logging.h"
#include "matter_mdns_internal.h"
#include "mdns_responder.h"
#include "semphr.h"

#define TAG "matter_mdns"

static char* s_hostname = NULL;
static matter_mdns_service_t s_services[MATTER_MDNS_MAX_SERVICES];
static size_t s_service_count = 0;

static SemaphoreHandle_t s_lock = NULL;
static StaticSemaphore_t s_lock_buf;
static volatile bool s_announce_pending = false;

static char* dup_str(const char* s) {
    if (!s) return NULL;
    size_t n = strlen(s) + 1;
    char* r = pvPortMalloc(n);
    if (r) memcpy(r, s, n);
    return r;
}

void matter_mdns_init(void) {
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateRecursiveMutexStatic(&s_lock_buf);
    }
    matter_mdns_lock();
    if (s_hostname) {
        vPortFree(s_hostname);
        s_hostname = NULL;
    }
    for (size_t i = 0; i < s_service_count; i++) {
        vPortFree(s_services[i].service);
        vPortFree(s_services[i].proto);
        vPortFree(s_services[i].instance);
        vPortFree(s_services[i].hostname);
        if (s_services[i].txt) vPortFree(s_services[i].txt);
        for (size_t j = 0; j < s_services[i].subtype_count; j++) {
            vPortFree(s_services[i].subtypes[j]);
        }
        if (s_services[i].subtypes) vPortFree(s_services[i].subtypes);
    }
    s_service_count = 0;
    s_announce_pending = false;
    matter_mdns_unlock();

    mdns_responder_init();
}

void matter_mdns_lock(void) {
    if (s_lock) xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
}

void matter_mdns_unlock(void) {
    if (s_lock) xSemaphoreGiveRecursive(s_lock);
}

const char* matter_mdns_locked_hostname(void) { return s_hostname; }

const matter_mdns_service_t* matter_mdns_locked_services(size_t* out_count) {
    *out_count = s_service_count;
    return s_services;
}

int matter_mdns_add_hostname(const char* hostname, const char* ipv6,
                             const char* ipv4) {
    (void)ipv6;
    (void)ipv4;
    matter_mdns_lock();
    if (s_hostname) vPortFree(s_hostname);
    char fqdn[64];
    snprintf(fqdn, sizeof(fqdn), "%s.local", hostname);
    s_hostname = dup_str(fqdn);
    s_announce_pending = true;
    matter_mdns_unlock();
    return 0;
}

static matter_mdns_service_t* find_service(const char* service,
                                           const char* proto,
                                           const char* instance) {
    for (size_t i = 0; i < s_service_count; i++) {
        if (strcmp(s_services[i].service, service) == 0 &&
            strcmp(s_services[i].proto, proto) == 0 &&
            strcmp(s_services[i].instance, instance) == 0) {
            return &s_services[i];
        }
    }
    return NULL;
}

int matter_mdns_add_service(const char* service, const char* proto,
                            uint16_t port, const uint8_t* txt_bytes,
                            size_t txt_len, const char* instance,
                            const char* hostname) {
    matter_mdns_lock();
    matter_mdns_service_t* s = find_service(service, proto, instance);
    if (!s) {
        if (s_service_count >= MATTER_MDNS_MAX_SERVICES) {
            matter_mdns_unlock();
            return -1;
        }
        s = &s_services[s_service_count++];
        memset(s, 0, sizeof(*s));
        s->service = dup_str(service);
        s->proto = dup_str(proto);
        s->instance = dup_str(instance);
    }

    if (s->hostname) vPortFree(s->hostname);
    char host_fqdn[64];
    snprintf(host_fqdn, sizeof(host_fqdn), "%s.local", hostname);
    s->hostname = dup_str(host_fqdn);
    s->port = port;

    if (s->txt) vPortFree(s->txt);
    s->txt = pvPortMalloc(txt_len);
    if (s->txt) {
        memcpy(s->txt, txt_bytes, txt_len);
        s->txt_len = txt_len;
    }

    s_announce_pending = true;
    matter_mdns_unlock();
    return 0;
}

int matter_mdns_add_subtype(const char* service, const char* proto,
                            const char* instance, const char* hostname,
                            const char* subtype) {
    (void)hostname;
    matter_mdns_lock();
    matter_mdns_service_t* s = find_service(service, proto, instance);
    if (!s) {
        matter_mdns_unlock();
        return -1;
    }

    for (size_t i = 0; i < s->subtype_count; i++) {
        if (strcmp(s->subtypes[i], subtype) == 0) {
            matter_mdns_unlock();
            return 0;
        }
    }

    char** next = pvPortMalloc(sizeof(char*) * (s->subtype_count + 1));
    if (!next) {
        matter_mdns_unlock();
        return -1;
    }
    if (s->subtypes) {
        memcpy(next, s->subtypes, sizeof(char*) * s->subtype_count);
        vPortFree(s->subtypes);
    }
    s->subtypes = next;
    s->subtypes[s->subtype_count++] = dup_str(subtype);

    s_announce_pending = true;
    matter_mdns_unlock();
    return 0;
}

int matter_mdns_remove_service(const char* service, const char* proto,
                               const char* instance, const char* hostname) {
    (void)hostname;
    matter_mdns_lock();
    for (size_t i = 0; i < s_service_count; i++) {
        if (strcmp(s_services[i].service, service) == 0 &&
            strcmp(s_services[i].proto, proto) == 0 &&
            strcmp(s_services[i].instance, instance) == 0) {
            vPortFree(s_services[i].service);
            vPortFree(s_services[i].proto);
            vPortFree(s_services[i].instance);
            vPortFree(s_services[i].hostname);
            if (s_services[i].txt) vPortFree(s_services[i].txt);
            for (size_t j = 0; j < s_services[i].subtype_count; j++) {
                vPortFree(s_services[i].subtypes[j]);
            }
            if (s_services[i].subtypes) vPortFree(s_services[i].subtypes);

            if (i < s_service_count - 1) {
                memmove(
                    &s_services[i], &s_services[i + 1],
                    sizeof(matter_mdns_service_t) * (s_service_count - i - 1));
            }
            s_service_count--;
            s_announce_pending = true;
            break;
        }
    }
    matter_mdns_unlock();
    return 0;
}

void matter_mdns_request_announce(void) { mdns_responder_request_announce(); }

bool matter_mdns_take_announce_pending(void) {
    bool p = s_announce_pending;
    s_announce_pending = false;
    return p;
}
