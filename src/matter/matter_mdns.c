/*
 * mDNS / DNS-SD record store backing the Berry `mdns.*` module.
 *
 * Berry calls (from berry_matter) populate a list of DNSRecord_t that
 * FreeRTOS-Plus-TCP's mDNS responder serves via
 * xApplicationDNSRecordQueryHook_Multi. Strings are heap-duplicated so they
 * outlive the Berry frames that constructed them. Mutex-guarded because the
 * mDNS responder task reads concurrently with the Berry VM task that writes.
 */

#include "matter_mdns.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_DNS_Globals.h"
#include "FreeRTOS_IP.h"
#include "app_logging.h"
#include "semphr.h"

#define TAG "matter_mdns"
#define MATTER_MDNS_MAX_RECORDS 64

typedef struct {
    DNSRecord_t rec;
    char* owned_name;
    char* owned_target;
    char* owned_txt;
    char* owned_ptr;
} matter_mdns_entry_t;

static matter_mdns_entry_t s_entries[MATTER_MDNS_MAX_RECORDS];
static DNSRecord_t s_view[MATTER_MDNS_MAX_RECORDS];
static UBaseType_t s_count;
static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buf;

static char* dup_str(const char* s) {
    if (!s) {
        return NULL;
    }
    size_t n = strlen(s) + 1;
    char* r = pvPortMalloc(n);
    if (r) {
        memcpy(r, s, n);
    }
    return r;
}

static void entry_free(matter_mdns_entry_t* e) {
    if (e->owned_name) vPortFree(e->owned_name);
    if (e->owned_target) vPortFree(e->owned_target);
    if (e->owned_txt) vPortFree(e->owned_txt);
    if (e->owned_ptr) vPortFree(e->owned_ptr);
    memset(e, 0, sizeof(*e));
}

void matter_mdns_init(void) {
    s_lock = xSemaphoreCreateRecursiveMutexStatic(&s_lock_buf);
    s_count = 0;
}

static void lock(void) {
    if (s_lock) xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
}

static void unlock(void) {
    if (s_lock) xSemaphoreGiveRecursive(s_lock);
}

static matter_mdns_entry_t* find_entry(uint16_t type, const char* name,
                                       const char* extra_match) {
    for (UBaseType_t i = 0; i < s_count; i++) {
        matter_mdns_entry_t* e = &s_entries[i];
        if (e->rec.usRecordType != type) continue;
        if (strcmp(e->owned_name ? e->owned_name : "", name) != 0) continue;
        if (!extra_match) return e;
        if (type == dnsTYPE_PTR && e->owned_ptr &&
            strcmp(e->owned_ptr, extra_match) == 0) {
            return e;
        }
    }
    return NULL;
}

static matter_mdns_entry_t* new_entry(void) {
    if (s_count >= MATTER_MDNS_MAX_RECORDS) {
        return NULL;
    }
    return &s_entries[s_count++];
}

static void rebuild_view(void) {
    for (UBaseType_t i = 0; i < s_count; i++) {
        s_view[i] = s_entries[i].rec;
    }
}

int matter_mdns_add_hostname(const char* hostname, const char* ipv4_or_ipv6,
                             const char* ipv4) {
    (void)ipv4_or_ipv6;
    (void)ipv4;
    char fqdn[64];
    snprintf(fqdn, sizeof(fqdn), "%s.local", hostname);
    lock();
    bool added_a = false;
    bool added_aaaa = false;
    if (!find_entry(dnsTYPE_A_HOST, fqdn, NULL)) {
        matter_mdns_entry_t* e = new_entry();
        if (!e) {
            unlock();
            return -1;
        }
        e->owned_name = dup_str(fqdn);
        e->rec.usRecordType = dnsTYPE_A_HOST;
        e->rec.pcName = e->owned_name;
        added_a = true;
    }
    /* The DNS responder fills the actual address bytes from the receiving
     * endpoint when answering A/AAAA queries; we just register the name. */
    if (!find_entry(dnsTYPE_AAAA_HOST, fqdn, NULL)) {
        matter_mdns_entry_t* e = new_entry();
        if (!e) {
            unlock();
            return -1;
        }
        e->owned_name = dup_str(fqdn);
        e->rec.usRecordType = dnsTYPE_AAAA_HOST;
        e->rec.pcName = e->owned_name;
        added_aaaa = true;
    }
    UBaseType_t total = s_count;
    rebuild_view();
    unlock();
    LogInfo(("mdns: add_hostname %s A=%d AAAA=%d total=%u", fqdn, (int)added_a,
             (int)added_aaaa, (unsigned)total));
    return 0;
}

int matter_mdns_add_service(const char* service, const char* proto, int port,
                            const char* txt_record, const char* instance,
                            const char* hostname) {
    char service_name[64];
    char instance_fqdn[96];
    char host_fqdn[64];
    snprintf(service_name, sizeof(service_name), "%s.%s.local", service, proto);
    snprintf(instance_fqdn, sizeof(instance_fqdn), "%s.%s.%s.local", instance,
             service, proto);
    snprintf(host_fqdn, sizeof(host_fqdn), "%s.local", hostname);
    lock();
    if (!find_entry(dnsTYPE_PTR, "_services._dns-sd._udp.local",
                    service_name)) {
        matter_mdns_entry_t* e = new_entry();
        if (e) {
            e->owned_name = dup_str("_services._dns-sd._udp.local");
            e->owned_ptr = dup_str(service_name);
            e->rec.usRecordType = dnsTYPE_PTR;
            e->rec.pcName = e->owned_name;
            e->rec.xData.pcPtrRecord = e->owned_ptr;
        }
    }
    matter_mdns_entry_t* e = new_entry();
    if (e) {
        e->owned_name = dup_str(service_name);
        e->owned_ptr = dup_str(instance_fqdn);
        e->rec.usRecordType = dnsTYPE_PTR;
        e->rec.pcName = e->owned_name;
        e->rec.xData.pcPtrRecord = e->owned_ptr;
    }
    e = new_entry();
    if (e) {
        e->owned_name = dup_str(instance_fqdn);
        e->owned_target = dup_str(host_fqdn);
        e->rec.usRecordType = dnsTYPE_SRV;
        e->rec.pcName = e->owned_name;
        e->rec.xData.xSrvRecord.pcTarget = e->owned_target;
        e->rec.xData.xSrvRecord.usPort = (uint16_t)port;
    }
    e = new_entry();
    if (e) {
        e->owned_name = dup_str(instance_fqdn);
        e->owned_txt = dup_str(txt_record ? txt_record : "");
        e->rec.usRecordType = dnsTYPE_TXT;
        e->rec.pcName = e->owned_name;
        e->rec.xData.pcTxtRecord = e->owned_txt;
    }
    rebuild_view();
    unlock();
    LogInfo(("mdns: add_service %s.%s %s", service, proto, instance));
    return 0;
}

int matter_mdns_add_subtype(const char* service, const char* proto,
                            const char* instance, const char* hostname,
                            const char* subtype) {
    (void)hostname;
    char sub_name[96];
    char instance_fqdn[96];
    snprintf(sub_name, sizeof(sub_name), "%s._sub.%s.%s.local", subtype,
             service, proto);
    snprintf(instance_fqdn, sizeof(instance_fqdn), "%s.%s.%s.local", instance,
             service, proto);
    lock();
    matter_mdns_entry_t* e = new_entry();
    if (e) {
        e->owned_name = dup_str(sub_name);
        e->owned_ptr = dup_str(instance_fqdn);
        e->rec.usRecordType = dnsTYPE_PTR;
        e->rec.pcName = e->owned_name;
        e->rec.xData.pcPtrRecord = e->owned_ptr;
    }
    rebuild_view();
    unlock();
    LogInfo(("mdns: add_subtype %s -> %s", sub_name, instance_fqdn));
    return 0;
}

int matter_mdns_remove_service(const char* service, const char* proto,
                               const char* instance, const char* hostname) {
    (void)hostname;
    char service_name[64];
    char instance_fqdn[96];
    snprintf(service_name, sizeof(service_name), "%s.%s.local", service, proto);
    snprintf(instance_fqdn, sizeof(instance_fqdn), "%s.%s.%s.local", instance,
             service, proto);
    lock();
    UBaseType_t w = 0;
    for (UBaseType_t r = 0; r < s_count; r++) {
        matter_mdns_entry_t* e = &s_entries[r];
        bool remove = false;
        if (e->rec.usRecordType == dnsTYPE_PTR && e->owned_ptr &&
            strcmp(e->owned_ptr, instance_fqdn) == 0) {
            remove = true;
        } else if ((e->rec.usRecordType == dnsTYPE_SRV ||
                    e->rec.usRecordType == dnsTYPE_TXT) &&
                   strcmp(e->owned_name ? e->owned_name : "", instance_fqdn) ==
                       0) {
            remove = true;
        }
        if (remove) {
            entry_free(e);
        } else {
            if (w != r) s_entries[w] = *e;
            w++;
        }
    }
    s_count = w;
    rebuild_view();
    unlock();
    LogInfo(("mdns: remove_service %s %s", service_name, instance_fqdn));
    return 0;
}

UBaseType_t matter_mdns_snapshot(DNSRecord_t** out) {
    lock();
    *out = s_view;
    UBaseType_t n = s_count;
    unlock();
    return n;
}

/* Hooks pulled in by FreeRTOS-Plus-TCP's mDNS responder. */
DNSRecord_t* xApplicationDNSRecordQueryHook_Multi(
    struct xNetworkEndPoint* pxEndPoint, UBaseType_t* outLen) {
    (void)pxEndPoint;
    DNSRecord_t* recs;
    *outLen = matter_mdns_snapshot(&recs);
    return recs;
}

void xApplicationDNSRecordsMatchedHook(void) {}
