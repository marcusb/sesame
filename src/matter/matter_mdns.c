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
#define MATTER_MDNS_VIEW_BUFFERS 4

typedef struct {
    DNSRecord_t rec;
    char* owned_name;
    char* owned_target;
    char* owned_txt;
    char* owned_ptr;
} matter_mdns_entry_t;

static matter_mdns_entry_t s_entries[MATTER_MDNS_MAX_RECORDS];
static DNSRecord_t s_view_buffers[MATTER_MDNS_VIEW_BUFFERS]
                                 [MATTER_MDNS_MAX_RECORDS];
static DNSRecord_t* volatile s_current_view = s_view_buffers[0];
static UBaseType_t s_view_index = 0;
static UBaseType_t s_count = 0;
static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buf;

static const char* const s_services_name = "_services._dns-sd._udp.local";

static char* dup_str(const char* s) {
    if (!s) return NULL;
    size_t n = strlen(s) + 1;
    char* r = pvPortMalloc(n);
    if (r) memcpy(r, s, n);
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
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateRecursiveMutexStatic(&s_lock_buf);
    }
    s_count = 0;
    s_view_index = 0;
    memset(s_entries, 0, sizeof(s_entries));
    memset(s_view_buffers, 0, sizeof(s_view_buffers));
    s_current_view = s_view_buffers[0];
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
    if (s_count >= MATTER_MDNS_MAX_RECORDS) return NULL;
    matter_mdns_entry_t* e = &s_entries[s_count++];
    memset(e, 0, sizeof(matter_mdns_entry_t));
    return e;
}

static void rebuild_view(void) {
    /* Write to the next buffer in a cyclic pool to avoid race with IP task. */
    UBaseType_t next_index = (s_view_index + 1) % MATTER_MDNS_VIEW_BUFFERS;
    DNSRecord_t* next_view = s_view_buffers[next_index];
    for (UBaseType_t i = 0; i < s_count; i++) {
        next_view[i] = s_entries[i].rec;
        next_view[i].uxServeRecord = 0;
    }
    s_view_index = next_index;
    s_current_view = next_view;
}

int matter_mdns_add_hostname(const char* hostname, const char* ipv4_or_ipv6,
                             const char* ipv4) {
    (void)ipv4_or_ipv6;
    (void)ipv4;
    char fqdn[64];
    snprintf(fqdn, sizeof(fqdn), "%s.local", hostname);
    lock();
    if (!find_entry(dnsTYPE_A_HOST, fqdn, NULL)) {
        matter_mdns_entry_t* e = new_entry();
        if (e) {
            e->owned_name = dup_str(fqdn);
            e->rec.usRecordType = dnsTYPE_A_HOST;
            e->rec.pcName = e->owned_name;
        }
    }
    if (!find_entry(dnsTYPE_AAAA_HOST, fqdn, NULL)) {
        matter_mdns_entry_t* e = new_entry();
        if (e) {
            e->owned_name = dup_str(fqdn);
            e->rec.usRecordType = dnsTYPE_AAAA_HOST;
            e->rec.pcName = e->owned_name;
        }
    }
    rebuild_view();
    unlock();
    return 0;
}

int matter_mdns_add_service(const char* service, const char* proto, int port,
                            const char* txt_record, const char* instance,
                            const char* hostname) {
    (void)instance;
    char service_name[64];
    char instance_fqdn[96];
    char host_fqdn[64];
    snprintf(service_name, sizeof(service_name), "%s.%s.local", service, proto);
    snprintf(instance_fqdn, sizeof(instance_fqdn), "%s.%s.%s.local", hostname,
             service, proto);
    snprintf(host_fqdn, sizeof(host_fqdn), "%s.local", hostname);
    lock();
    if (!find_entry(dnsTYPE_PTR, s_services_name, service_name)) {
        matter_mdns_entry_t* e = new_entry();
        if (e) {
            e->owned_name = dup_str(s_services_name);
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
    (void)instance;
    char sub_name[96];
    char instance_fqdn[96];
    snprintf(sub_name, sizeof(sub_name), "%s._sub.%s.%s.local", subtype,
             service, proto);
    snprintf(instance_fqdn, sizeof(instance_fqdn), "%s.%s.%s.local", hostname,
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
            strcmp(e->owned_ptr, instance_fqdn) == 0)
            remove = true;
        else if ((e->rec.usRecordType == dnsTYPE_SRV ||
                  e->rec.usRecordType == dnsTYPE_TXT) &&
                 strcmp(e->owned_name ? e->owned_name : "", instance_fqdn) == 0)
            remove = true;
        if (remove)
            entry_free(e);
        else {
            if (w != r) s_entries[w] = *e;
            w++;
        }
    }
    s_count = w;
    rebuild_view();
    unlock();
    return 0;
}

DNSRecord_t* xApplicationDNSRecordQueryHook_Multi(
    struct xNetworkEndPoint* pxEndPoint, UBaseType_t* outLen) {
    (void)pxEndPoint;
    *outLen = s_count;
    return s_current_view;
}

static void serialize_announce_packet(NetworkEndPoint_t* ep, uint8_t* buf,
                                      size_t* outLen) {
    DNSMessage_t* pxDNSMessage = (DNSMessage_t*)buf;
    memset(buf, 0, 1024);
    pxDNSMessage->usFlags = FreeRTOS_htons(0x8400);
    uint8_t* pucWrite = buf + sizeof(DNSMessage_t);
    int answers = 0;
    lock();
    UBaseType_t count = s_count;
    DNSRecord_t* view = s_current_view;
    for (UBaseType_t i = 0; i < count; i++) {
        DNSRecord_t* r = &view[i];
        if (r->usRecordType == dnsTYPE_A_HOST && ep->bits.bIPv6) continue;
        if (r->usRecordType == dnsTYPE_AAAA_HOST && !ep->bits.bIPv6) continue;
        size_t est = strlen(r->pcName ? r->pcName : "") + 12;
        if (r->usRecordType == dnsTYPE_PTR)
            est += strlen(r->xData.pcPtrRecord ? r->xData.pcPtrRecord : "") + 2;
        else if (r->usRecordType == dnsTYPE_SRV)
            est += strlen(r->xData.xSrvRecord.pcTarget
                              ? r->xData.xSrvRecord.pcTarget
                              : "") +
                   8;
        else if (r->usRecordType == dnsTYPE_TXT)
            est += strlen(r->xData.pcTxtRecord ? r->xData.pcTxtRecord : "");
        else if (r->usRecordType == dnsTYPE_A_HOST)
            est += 4;
        else if (r->usRecordType == dnsTYPE_AAAA_HOST)
            est += 16;
        if ((pucWrite - buf) + est > 1000) break;
        const char* p = r->pcName;
        while (p && *p) {
            const char* next = strchr(p, '.');
            int len = next ? (next - p) : (int)strlen(p);
            *pucWrite++ = (uint8_t)len;
            memcpy(pucWrite, p, (size_t)len);
            pucWrite += len;
            if (!next) break;
            p = next + 1;
        }
        *pucWrite++ = 0;
        pucWrite[0] = (uint8_t)(r->usRecordType >> 8);
        pucWrite[1] = (uint8_t)(r->usRecordType & 0xff);
        pucWrite += 2;
        uint16_t cls =
            dnsCLASS_IN | (r->usRecordType != dnsTYPE_PTR ? 0x8000 : 0);
        pucWrite[0] = (uint8_t)(cls >> 8);
        pucWrite[1] = (uint8_t)(cls & 0xff);
        pucWrite += 2;
        uint32_t ttl = (r->usRecordType == dnsTYPE_PTR) ? 4500 : 120;
        pucWrite[0] = (uint8_t)(ttl >> 24);
        pucWrite[1] = (uint8_t)(ttl >> 16);
        pucWrite[2] = (uint8_t)(ttl >> 8);
        pucWrite[3] = (uint8_t)(ttl & 0xff);
        pucWrite += 4;
        uint8_t* pLen = pucWrite;
        pucWrite += 2;
        uint8_t* pStart = pucWrite;
        switch (r->usRecordType) {
            case dnsTYPE_PTR:
                p = r->xData.pcPtrRecord;
                while (p && *p) {
                    const char* next = strchr(p, '.');
                    int len = next ? (next - p) : (int)strlen(p);
                    *pucWrite++ = (uint8_t)len;
                    memcpy(pucWrite, p, (size_t)len);
                    pucWrite += len;
                    if (!next) break;
                    p = next + 1;
                }
                *pucWrite++ = 0;
                break;
            case dnsTYPE_SRV:
                memset(pucWrite, 0, 4);
                pucWrite += 4;
                pucWrite[0] = (uint8_t)(r->xData.xSrvRecord.usPort >> 8);
                pucWrite[1] = (uint8_t)(r->xData.xSrvRecord.usPort & 0xff);
                pucWrite += 2;
                p = r->xData.xSrvRecord.pcTarget;
                while (p && *p) {
                    const char* next = strchr(p, '.');
                    int len = next ? (next - p) : (int)strlen(p);
                    *pucWrite++ = (uint8_t)len;
                    memcpy(pucWrite, p, (size_t)len);
                    pucWrite += len;
                    if (!next) break;
                    p = next + 1;
                }
                *pucWrite++ = 0;
                break;
            case dnsTYPE_TXT: {
                size_t tl = strlen(r->xData.pcTxtRecord);
                if (tl > 0)
                    memcpy(pucWrite, r->xData.pcTxtRecord, tl);
                else
                    *pucWrite = 0;
                pucWrite += (tl > 0 ? tl : 1);
            } break;
            case dnsTYPE_A_HOST: {
                uint32_t ip = FreeRTOS_ntohl(ep->ipv4_settings.ulIPAddress);
                pucWrite[0] = (uint8_t)(ip >> 24);
                pucWrite[1] = (uint8_t)(ip >> 16);
                pucWrite[2] = (uint8_t)(ip >> 8);
                pucWrite[3] = (uint8_t)(ip & 0xff);
                pucWrite += 4;
            } break;
            case dnsTYPE_AAAA_HOST:
                memcpy(pucWrite, ep->ipv6_settings.xIPAddress.ucBytes, 16);
                pucWrite += 16;
                break;
        }
        uint16_t dlen = (uint16_t)(pucWrite - pStart);
        pLen[0] = (uint8_t)(dlen >> 8);
        pLen[1] = (uint8_t)(dlen & 0xff);
        answers++;
    }
    unlock();
    pxDNSMessage->usAnswers = FreeRTOS_htons((uint16_t)answers);
    *outLen = (size_t)(pucWrite - buf);
}

typedef struct {
    uint8_t* buf;
} announce_params_t;
static void announce_task(void* pvParameters) {
    announce_params_t* p = (announce_params_t*)pvParameters;
    struct freertos_sockaddr xAddr;
    for (int retry = 0; retry < 5; retry++) {
        for (NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(NULL); ep != NULL;
             ep = FreeRTOS_NextEndPoint(NULL, ep)) {
            if (!ep->pxNetworkInterface ||
                !ep->pxNetworkInterface->bits.bInterfaceUp ||
                !ep->bits.bEndPointUp)
                continue;
            size_t plen;
            serialize_announce_packet(ep, p->buf, &plen);
            Socket_t s = FreeRTOS_socket(
                ep->bits.bIPv6 ? FREERTOS_AF_INET6 : FREERTOS_AF_INET,
                FREERTOS_SOCK_DGRAM, ipPROTOCOL_UDP);
            if (s != FREERTOS_INVALID_SOCKET) {
                memset(&xAddr, 0, sizeof(xAddr));
                xAddr.sin_len = (uint8_t)sizeof(xAddr);
                xAddr.sin_port = FreeRTOS_htons(5353);
                if (!ep->bits.bIPv6) {
                    xAddr.sin_family = FREERTOS_AF_INET;
                    xAddr.sin_address.ulIP_IPv4 =
                        FreeRTOS_inet_addr("224.0.0.251");
                } else {
                    xAddress.sin_family = FREERTOS_AF_INET6;
                    FreeRTOS_inet_pton(FREERTOS_AF_INET6, "ff02::fb",
                                       xAddress.sin_address.xIP_IPv6.ucBytes);
                }
                FreeRTOS_sendto(s, p->buf, plen, 0, &xAddr, sizeof(xAddr));
                FreeRTOS_closesocket(s);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(500 * (1 << retry)));
    }
    vPortFree(p->buf);
    vPortFree(p);
    vTaskDelete(NULL);
}

void matter_mdns_announce(void) {
    LogInfo(("mdns: spawning proactive discovery task"));
    announce_params_t* params = pvPortMalloc(sizeof(announce_params_t));
    if (!params) return;
    params->buf = pvPortMalloc(1024);
    if (!params->buf) {
        vPortFree(params);
        return;
    }
    if (xTaskCreate(announce_task, "mdns_ann", 2048, params,
                    tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        vPortFree(params->buf);
        vPortFree(params);
    }
}

void xApplicationDNSRecordsMatchedHook(void) {
    lock();
    bool changed;
    int iterations = 0;
    DNSRecord_t* view = s_current_view;
    do {
        changed = false;
        iterations++;
        UBaseType_t count = s_count;
        for (UBaseType_t i = 0; i < count; i++) {
            DNSRecord_t* r = &view[i];
            if (r->uxServeRecord == 0) continue;
            if (r->usRecordType == dnsTYPE_PTR) {
                for (UBaseType_t j = 0; j < count; j++) {
                    DNSRecord_t* other = &view[j];
                    if (other->uxServeRecord != 0) continue;
                    if ((other->usRecordType == dnsTYPE_SRV ||
                         other->usRecordType == dnsTYPE_TXT) &&
                        strcmp(other->pcName ? other->pcName : "",
                               r->xData.pcPtrRecord ? r->xData.pcPtrRecord
                                                    : "") == 0) {
                        other->uxServeRecord = 1;
                        changed = true;
                    }
                }
            }
            if (r->usRecordType == dnsTYPE_SRV) {
                for (UBaseType_t j = 0; j < count; j++) {
                    DNSRecord_t* other = &view[j];
                    if (other->uxServeRecord != 0) continue;
                    if ((other->usRecordType == dnsTYPE_A_HOST ||
                         other->usRecordType == dnsTYPE_AAAA_HOST) &&
                        strcmp(other->pcName ? other->pcName : "",
                               r->xData.xSrvRecord.pcTarget
                                   ? r->xData.xSrvRecord.pcTarget
                                   : "") == 0) {
                        other->uxServeRecord = 1;
                        changed = true;
                    }
                }
            }
        }
    } while (changed && iterations < 10);
    unlock();
}
