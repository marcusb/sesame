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
#include "FreeRTOS_Sockets.h"
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
        if (strcmp(e->rec.pcName ? e->rec.pcName : "", name) != 0) continue;
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
        if (e) {
            e->owned_name = dup_str(fqdn);
            e->rec.usRecordType = dnsTYPE_A_HOST;
            e->rec.pcName = e->owned_name;
            added_a = true;
        }
    }
    if (!find_entry(dnsTYPE_AAAA_HOST, fqdn, NULL)) {
        matter_mdns_entry_t* e = new_entry();
        if (e) {
            e->owned_name = dup_str(fqdn);
            e->rec.usRecordType = dnsTYPE_AAAA_HOST;
            e->rec.pcName = e->owned_name;
            added_aaaa = true;
        }
    }
    rebuild_view();
    unlock();
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
    if (!find_entry(dnsTYPE_PTR, service_name, instance_fqdn)) {
        matter_mdns_entry_t* e = new_entry();
        if (e) {
            e->owned_name = dup_str(service_name);
            e->owned_ptr = dup_str(instance_fqdn);
            e->rec.usRecordType = dnsTYPE_PTR;
            e->rec.pcName = e->owned_name;
            e->rec.xData.pcPtrRecord = e->owned_ptr;
        }
    }
    if (!find_entry(dnsTYPE_SRV, instance_fqdn, NULL)) {
        matter_mdns_entry_t* e = new_entry();
        if (e) {
            e->owned_name = dup_str(instance_fqdn);
            e->owned_target = dup_str(host_fqdn);
            e->rec.usRecordType = dnsTYPE_SRV;
            e->rec.pcName = e->owned_name;
            e->rec.xData.xSrvRecord.pcTarget = e->owned_target;
            e->rec.xData.xSrvRecord.usPort = (uint16_t)port;
        }
    }
    if (!find_entry(dnsTYPE_TXT, instance_fqdn, NULL)) {
        matter_mdns_entry_t* e = new_entry();
        if (e) {
            e->owned_name = dup_str(instance_fqdn);
            e->owned_txt = dup_str(txt_record ? txt_record : "");
            e->rec.usRecordType = dnsTYPE_TXT;
            e->rec.pcName = e->owned_name;
            e->rec.xData.pcTxtRecord = e->owned_txt;
        }
    }
    rebuild_view();
    unlock();
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
    if (!find_entry(dnsTYPE_PTR, sub_name, instance_fqdn)) {
        matter_mdns_entry_t* e = new_entry();
        if (e) {
            e->owned_name = dup_str(sub_name);
            e->owned_ptr = dup_str(instance_fqdn);
            e->rec.usRecordType = dnsTYPE_PTR;
            e->rec.pcName = e->owned_name;
            e->rec.xData.pcPtrRecord = e->owned_ptr;
        }
    }
    rebuild_view();
    unlock();
    return 0;
}

int matter_mdns_remove_service(const char* service, const char* proto,
                               const char* instance, const char* hostname) {
    (void)hostname;
    char instance_fqdn[96];
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
    return 0;
}

UBaseType_t matter_mdns_snapshot(DNSRecord_t** out) {
    lock();
    *out = s_view;
    UBaseType_t n = s_count;
    unlock();
    return n;
}

UBaseType_t matter_mdns_get_view(DNSRecord_t** out) {
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

void matter_mdns_announce(void) {
    LogInfo(("mdns: sending proactive announcement"));
    Socket_t xSocket;
    struct freertos_sockaddr xAddress;
    uint8_t* pucBuffer;
    size_t uxBufferLength = 1024;

    pucBuffer = malloc(uxBufferLength);
    if (!pucBuffer) {
        return;
    }
    memset(pucBuffer, 0, uxBufferLength);

    DNSMessage_t* pxDNSMessage = (DNSMessage_t*)pucBuffer;
    pxDNSMessage->usFlags =
        FreeRTOS_htons(0x8400); /* Response, Authoritative */

    uint8_t* pucWrite = pucBuffer + sizeof(DNSMessage_t);
    int answers = 0;

    lock();
    NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(NULL);
    for (UBaseType_t i = 0; i < s_count; i++) {
        matter_mdns_entry_t* e = &s_entries[i];
        DNSRecord_t* r = &e->rec;

        /* Write record name */
        const char* name = r->pcName;
        const char* p = name;
        while (p && *p) {
            const char* next = strchr(p, '.');
            int len = next ? (next - p) : strlen(p);
            *pucWrite++ = (uint8_t)len;
            memcpy(pucWrite, p, len);
            pucWrite += len;
            if (!next) break;
            p = next + 1;
        }
        *pucWrite++ = 0;

        /* Type */
        pucWrite[0] = (uint8_t)(r->usRecordType >> 8);
        pucWrite[1] = (uint8_t)(r->usRecordType & 0xff);
        pucWrite += 2;
        /* Class (IN + Cache Flush) */
        pucWrite[0] = 0x80;
        pucWrite[1] = 0x01;
        pucWrite += 2;
        /* TTL */
        uint32_t ttl = (r->usRecordType == dnsTYPE_PTR) ? 4500 : 120;
        pucWrite[0] = (uint8_t)(ttl >> 24);
        pucWrite[1] = (uint8_t)(ttl >> 16);
        pucWrite[2] = (uint8_t)(ttl >> 8);
        pucWrite[3] = (uint8_t)(ttl & 0xff);
        pucWrite += 4;

        uint8_t* pucDataLen = pucWrite;
        pucWrite += 2;
        uint8_t* pucDataStart = pucWrite;

        switch (r->usRecordType) {
            case dnsTYPE_PTR:
                p = r->xData.pcPtrRecord;
                while (p && *p) {
                    const char* next = strchr(p, '.');
                    int len = next ? (next - p) : strlen(p);
                    *pucWrite++ = (uint8_t)len;
                    memcpy(pucWrite, p, len);
                    pucWrite += len;
                    if (!next) break;
                    p = next + 1;
                }
                *pucWrite++ = 0;
                break;
            case dnsTYPE_SRV:
                /* Priority & Weight */
                memset(pucWrite, 0, 4);
                pucWrite += 4;
                /* Port */
                pucWrite[0] = (uint8_t)(r->xData.xSrvRecord.usPort >> 8);
                pucWrite[1] = (uint8_t)(r->xData.xSrvRecord.usPort & 0xff);
                pucWrite += 2;
                /* Target */
                p = r->xData.xSrvRecord.pcTarget;
                while (p && *p) {
                    const char* next = strchr(p, '.');
                    int len = next ? (next - p) : strlen(p);
                    *pucWrite++ = (uint8_t)len;
                    memcpy(pucWrite, p, len);
                    pucWrite += len;
                    if (!next) break;
                    p = next + 1;
                }
                *pucWrite++ = 0;
                break;
            case dnsTYPE_TXT: {
                size_t txt_len = strlen(r->xData.pcTxtRecord);
                memcpy(pucWrite, r->xData.pcTxtRecord, txt_len);
                pucWrite += txt_len;
            } break;
            case dnsTYPE_A_HOST:
                if (ep) {
                    uint32_t ip = FreeRTOS_ntohl(ep->ipv4_settings.ulIPAddress);
                    pucWrite[0] = (uint8_t)(ip >> 24);
                    pucWrite[1] = (uint8_t)(ip >> 16);
                    pucWrite[2] = (uint8_t)(ip >> 8);
                    pucWrite[3] = (uint8_t)(ip & 0xff);
                    pucWrite += 4;
                }
                break;
            case dnsTYPE_AAAA_HOST: {
                NetworkEndPoint_t* ep6 = ep;
                while (ep6 && !ep6->bits.bIPv6)
                    ep6 = FreeRTOS_NextEndPoint(NULL, ep6);
                if (ep6) {
                    memcpy(pucWrite, ep6->ipv6_settings.xIPAddress.ucBytes, 16);
                    pucWrite += 16;
                }
            } break;
        }
        uint16_t dlen = (uint16_t)(pucWrite - pucDataStart);
        pucDataLen[0] = (uint8_t)(dlen >> 8);
        pucDataLen[1] = (uint8_t)(dlen & 0xff);
        answers++;
        if (pucWrite - pucBuffer > 900) break;
    }
    unlock();

    pxDNSMessage->usAnswers = FreeRTOS_htons((uint16_t)answers);

    if (answers > 0) {
        size_t packet_len = (size_t)(pucWrite - pucBuffer);

        for (int retry = 0; retry < 2; retry++) {
            /* Send to IPv4 multicast */
            xSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM,
                                      ipPROTOCOL_UDP);
            if (xSocket != FREERTOS_INVALID_SOCKET) {
                xAddress.sin_port = FreeRTOS_htons(5353);
                xAddress.sin_address.ulIP_IPv4 =
                    FreeRTOS_inet_addr("224.0.0.251");
                FreeRTOS_sendto(xSocket, pucBuffer, packet_len, 0, &xAddress,
                                sizeof(xAddress));
                FreeRTOS_closesocket(xSocket);
            }

            /* Send to IPv6 multicast */
            xSocket = FreeRTOS_socket(FREERTOS_AF_INET6, FREERTOS_SOCK_DGRAM,
                                      ipPROTOCOL_UDP);
            if (xSocket != FREERTOS_INVALID_SOCKET) {
                xAddress.sin_port = FreeRTOS_htons(5353);
                xAddress.sin_family = FREERTOS_AF_INET6;
                FreeRTOS_inet_pton(FREERTOS_AF_INET6, "ff02::fb",
                                   xAddress.sin_address.xIP_IPv6.ucBytes);
                FreeRTOS_sendto(xSocket, pucBuffer, packet_len, 0, &xAddress,
                                sizeof(xAddress));
                FreeRTOS_closesocket(xSocket);
            }
            if (retry == 0) vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    free(pucBuffer);
}

void xApplicationDNSRecordsMatchedHook(void) {
    lock();
    bool changed;
    int iterations = 0;
    do {
        changed = false;
        iterations++;
        for (UBaseType_t i = 0; i < s_count; i++) {
            DNSRecord_t* r = &s_view[i];
            if (r->uxServeRecord == 0) continue;

            /* If we are serving a PTR record, also serve the SRV and TXT
             * records it points to. */
            if (r->usRecordType == dnsTYPE_PTR) {
                for (UBaseType_t j = 0; j < s_count; j++) {
                    DNSRecord_t* other = &s_view[j];
                    if (other->uxServeRecord != 0) continue;
                    if ((other->usRecordType == dnsTYPE_SRV ||
                         other->usRecordType == dnsTYPE_TXT) &&
                        strcmp(other->pcName ? other->pcName : "",
                               r->xData.pcPtrRecord ? r->xData.pcPtrRecord
                                                    : "") == 0) {
                        other->uxServeRecord =
                            1; /* dnsRECORD_SERVE_ADDITIONAL */
                        changed = true;
                    }
                }
            }

            /* If we are serving an SRV record, also serve the A/AAAA records
             * for its target host. */
            if (r->usRecordType == dnsTYPE_SRV) {
                for (UBaseType_t j = 0; j < s_count; j++) {
                    DNSRecord_t* other = &s_view[j];
                    if (other->uxServeRecord != 0) continue;
                    if ((other->usRecordType == dnsTYPE_A_HOST ||
                         other->usRecordType == dnsTYPE_AAAA_HOST) &&
                        strcmp(other->pcName ? other->pcName : "",
                               r->xData.xSrvRecord.pcTarget
                                   ? r->xData.xSrvRecord.pcTarget
                                   : "") == 0) {
                        other->uxServeRecord =
                            1; /* dnsRECORD_SERVE_ADDITIONAL */
                        changed = true;
                    }
                }
            }
        }
    } while (changed && iterations < 10);
    unlock();
}
