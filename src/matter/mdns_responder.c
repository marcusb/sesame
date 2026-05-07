#include "mdns_responder.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "app_logging.h"
#include "backoff_algorithm.h"
#include "matter_mdns_internal.h"

#define TAG "mdns_resp"

#define MDNS_PORT 5353
#define MDNS_V4_ADDR "224.0.0.251"
#define MDNS_V6_ADDR "ff02::fb"

#define DNS_FLAGS_QUERY 0x0000
#define DNS_FLAGS_RESPONSE 0x8400

#define DNS_TYPE_A 1
#define DNS_TYPE_PTR 12
#define DNS_TYPE_TXT 16
#define DNS_TYPE_AAAA 28
#define DNS_TYPE_SRV 33
#define DNS_TYPE_ANY 255

#define DNS_CLASS_IN 1

#define MDNS_MAX_PACKET 1280
#define MDNS_TASK_STACK 4096

typedef struct __attribute__((packed)) {
    uint16_t usID;
    uint16_t usFlags;
    uint16_t usQuestions;
    uint16_t usAnswers;
    uint16_t usAuthoritative;
    uint16_t usAdditional;
} DNSHeader_t;

static TaskHandle_t s_mdns_task = NULL;

/* -------------------------------------------------------------------------- */
/* Wire Helpers                                                               */
/* -------------------------------------------------------------------------- */

static uint8_t* dns_write_u16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xff);
    return p + 2;
}

static uint8_t* dns_write_u32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v & 0xff);
    return p + 4;
}

static uint8_t* dns_write_name(uint8_t* p, uint8_t* end, const char* name) {
    if (!name) return p;
    const char* start = name;
    while (*start) {
        const char* dot = strchr(start, '.');
        size_t len = dot ? (size_t)(dot - start) : strlen(start);
        if (len > 63 || p + len + 1 >= end) return p;
        *p++ = (uint8_t)len;
        memcpy(p, start, len);
        p += len;
        if (!dot) break;
        start = dot + 1;
    }
    if (p < end) *p++ = 0;
    return p;
}

static const uint8_t* dns_read_name(const uint8_t* base, size_t total,
                                    const uint8_t* curr, char* out,
                                    size_t out_sz, const uint8_t** next_off) {
    size_t written = 0;
    const uint8_t* p = curr;
    bool followed_pointer = false;
    const uint8_t* first_next = NULL;
    int labels = 0;

    while (p < base + total && labels < 20) {
        uint8_t len = *p++;
        if (len == 0) {
            if (!followed_pointer) first_next = p;
            break;
        }
        if ((len & 0xc0) == 0xc0) {
            if (p >= base + total) break;
            uint16_t off = ((len & 0x3f) << 8) | *p++;
            if (!followed_pointer) first_next = p;
            if (off >= total) break;
            p = base + off;
            followed_pointer = true;
            continue;
        }
        if (p + len > base + total) break;
        if (written > 0 && written < out_sz - 1) out[written++] = '.';
        size_t to_copy =
            (written + len < out_sz) ? len : (out_sz - written - 1);
        memcpy(out + written, p, to_copy);
        written += to_copy;
        p += len;
        labels++;
    }
    out[written] = '\0';
    if (next_off) *next_off = followed_pointer ? first_next : p;
    return curr;
}

/* -------------------------------------------------------------------------- */
/* Response Builder                                                           */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t* buf;
    uint8_t* p;
    uint8_t* end;
    uint16_t ancount;
} mdns_resp_ctx_t;

static void dns_start_record(mdns_resp_ctx_t* ctx, const char* name,
                             uint16_t type, uint16_t cls, uint32_t ttl) {
    ctx->p = dns_write_name(ctx->p, ctx->end, name);
    if (ctx->p + 10 > ctx->end) return;
    ctx->p = dns_write_u16(ctx->p, type);
    ctx->p = dns_write_u16(ctx->p, cls);
    ctx->p = dns_write_u32(ctx->p, ttl);
    /* Placeholder for RDLEN */
    ctx->p += 2;
}

static void dns_end_record(mdns_resp_ctx_t* ctx, uint8_t* rdata_start) {
    uint16_t rdlen = (uint16_t)(ctx->p - rdata_start);
    dns_write_u16(rdata_start - 2, rdlen);
    ctx->ancount++;
}

static void dns_add_ptr(mdns_resp_ctx_t* ctx, const char* name, const char* ptr,
                        bool flush) {
    if (!name || !ptr || ctx->p + 64 > ctx->end) return;
    dns_start_record(ctx, name, DNS_TYPE_PTR,
                     DNS_CLASS_IN | (flush ? 0x8000 : 0), 4500);
    uint8_t* rdata = ctx->p;
    ctx->p = dns_write_name(ctx->p, ctx->end, ptr);
    dns_end_record(ctx, rdata);
}

static void dns_add_srv(mdns_resp_ctx_t* ctx, const char* name,
                        const char* target, uint16_t port) {
    if (!name || !target || ctx->p + 128 > ctx->end) return;
    dns_start_record(ctx, name, DNS_TYPE_SRV, DNS_CLASS_IN | 0x8000, 120);
    uint8_t* rdata = ctx->p;
    ctx->p = dns_write_u16(ctx->p, 0);  // priority
    ctx->p = dns_write_u16(ctx->p, 0);  // weight
    ctx->p = dns_write_u16(ctx->p, port);
    ctx->p = dns_write_name(ctx->p, ctx->end, target);
    dns_end_record(ctx, rdata);
}

static void dns_add_txt(mdns_resp_ctx_t* ctx, const char* name,
                        const uint8_t* txt, size_t txt_len) {
    if (!name || ctx->p + txt_len + 64 > ctx->end) return;
    dns_start_record(ctx, name, DNS_TYPE_TXT, DNS_CLASS_IN | 0x8000, 120);
    uint8_t* rdata = ctx->p;
    if (txt && txt_len > 0) {
        memcpy(ctx->p, txt, txt_len);
        ctx->p += txt_len;
    } else {
        *ctx->p++ = 0;
    }
    dns_end_record(ctx, rdata);
}

static void dns_add_a(mdns_resp_ctx_t* ctx, const char* name) {
    if (!name) return;
    for (NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(NULL); ep != NULL;
         ep = FreeRTOS_NextEndPoint(NULL, ep)) {
        if (!ep->bits.bEndPointUp || ep->bits.bIPv6) continue;
        if (ep->ipv4_settings.ulIPAddress == 0) continue;
        if (ctx->p + 32 > ctx->end) return;
        dns_start_record(ctx, name, DNS_TYPE_A, DNS_CLASS_IN | 0x8000, 120);
        uint8_t* rdata = ctx->p;
        uint32_t ip = FreeRTOS_ntohl(ep->ipv4_settings.ulIPAddress);
        ctx->p = dns_write_u32(ctx->p, ip);
        dns_end_record(ctx, rdata);
    }
}

static void dns_add_aaaa(mdns_resp_ctx_t* ctx, const char* name) {
    if (!name) return;
    /* Prefer non-link-local */
    for (int pass = 0; pass < 2; pass++) {
        for (NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(NULL); ep != NULL;
             ep = FreeRTOS_NextEndPoint(NULL, ep)) {
            if (!ep->bits.bEndPointUp || !ep->bits.bIPv6) continue;
            bool link_local =
                (ep->ipv6_settings.xIPAddress.ucBytes[0] == 0xfe &&
                 (ep->ipv6_settings.xIPAddress.ucBytes[1] & 0xc0) == 0x80);
            if (pass == 0 && link_local) continue;
            if (pass == 1 && !link_local) continue;

            if (ctx->p + 32 > ctx->end) return;
            dns_start_record(ctx, name, DNS_TYPE_AAAA, DNS_CLASS_IN | 0x8000,
                             120);
            uint8_t* rdata = ctx->p;
            memcpy(ctx->p, ep->ipv6_settings.xIPAddress.ucBytes, 16);
            ctx->p += 16;
            dns_end_record(ctx, rdata);
        }
    }
}

static void add_service_records(mdns_resp_ctx_t* ctx,
                                const matter_mdns_service_t* s,
                                bool include_ptr, const char* ptr_name) {
    char instance_fqdn[128];
    snprintf(instance_fqdn, sizeof(instance_fqdn), "%s.%s.%s.local",
             s->instance, s->service, s->proto);

    if (include_ptr) {
        dns_add_ptr(ctx, ptr_name, instance_fqdn, false);
    }
    dns_add_srv(ctx, instance_fqdn, s->hostname, s->port);
    dns_add_txt(ctx, instance_fqdn, s->txt, s->txt_len);
    dns_add_a(ctx, s->hostname);
    dns_add_aaaa(ctx, s->hostname);
}

static void process_query(mdns_resp_ctx_t* ctx, const char* name,
                          uint16_t type) {
    size_t count;
    const matter_mdns_service_t* services = matter_mdns_locked_services(&count);
    const char* hostname = matter_mdns_locked_hostname();

    /* _services._dns-sd._udp.local */
    if (strcmp(name, "_services._dns-sd._udp.local") == 0) {
        if (type == DNS_TYPE_PTR || type == DNS_TYPE_ANY) {
            char seen[MATTER_MDNS_MAX_SERVICES][64];
            int seen_count = 0;
            for (size_t i = 0; i < count; i++) {
                char sn[64];
                snprintf(sn, sizeof(sn), "%s.%s.local", services[i].service,
                         services[i].proto);
                bool duplicate = false;
                for (int j = 0; j < seen_count; j++) {
                    if (strcmp(seen[j], sn) == 0) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    dns_add_ptr(ctx, name, sn, false);
                    if (seen_count < (int)MATTER_MDNS_MAX_SERVICES)
                        strcpy(seen[seen_count++], sn);
                }
            }
        }
        return;
    }

    /* service.proto.local or instance.service.proto.local or
     * subtype._sub.service.proto.local */
    for (size_t i = 0; i < count; i++) {
        const matter_mdns_service_t* s = &services[i];
        char sn[64], in[128];
        snprintf(sn, sizeof(sn), "%s.%s.local", s->service, s->proto);
        snprintf(in, sizeof(in), "%s.%s.%s.local", s->instance, s->service,
                 s->proto);

        if (strcmp(name, sn) == 0) {
            if (type == DNS_TYPE_PTR || type == DNS_TYPE_ANY) {
                add_service_records(ctx, s, true, sn);
            }
        } else if (strcmp(name, in) == 0) {
            if (type == DNS_TYPE_SRV || type == DNS_TYPE_TXT ||
                type == DNS_TYPE_ANY) {
                add_service_records(ctx, s, false, NULL);
            }
        } else {
            for (size_t j = 0; j < s->subtype_count; j++) {
                char sub[128];
                snprintf(sub, sizeof(sub), "%s._sub.%s.%s.local",
                         s->subtypes[j], s->service, s->proto);
                if (strcmp(name, sub) == 0) {
                    if (type == DNS_TYPE_PTR || type == DNS_TYPE_ANY) {
                        add_service_records(ctx, s, true, sub);
                    }
                    break;
                }
            }
        }
    }

    /* Hostname */
    if (hostname && strcmp(name, hostname) == 0) {
        if (type == DNS_TYPE_A || type == DNS_TYPE_ANY)
            dns_add_a(ctx, hostname);
        if (type == DNS_TYPE_AAAA || type == DNS_TYPE_ANY)
            dns_add_aaaa(ctx, hostname);
    }
}

/* -------------------------------------------------------------------------- */
/* Task                                                                       */
/* -------------------------------------------------------------------------- */

static Socket_t create_mdns_socket(BaseType_t family) {
    Socket_t s = FreeRTOS_socket(family, FREERTOS_SOCK_DGRAM, ipPROTOCOL_UDP);
    if (s == FREERTOS_INVALID_SOCKET) return FREERTOS_INVALID_SOCKET;

    struct freertos_sockaddr bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_port = FreeRTOS_htons(MDNS_PORT);
    bind_addr.sin_family = family;

    if (FreeRTOS_bind(s, &bind_addr, sizeof(bind_addr)) != 0) {
        FreeRTOS_closesocket(s);
        return FREERTOS_INVALID_SOCKET;
    }

    TickType_t timeout = pdMS_TO_TICKS(50);
    FreeRTOS_setsockopt(s, 0, FREERTOS_SO_RCVTIMEO, &timeout, sizeof(timeout));
    return s;
}

static void mdns_responder_task(void* pvParameters) {
    (void)pvParameters;
    Socket_t s4 = create_mdns_socket(FREERTOS_AF_INET);
    Socket_t s6 = create_mdns_socket(FREERTOS_AF_INET6);

    if (s4 == FREERTOS_INVALID_SOCKET && s6 == FREERTOS_INVALID_SOCKET) {
        LogError(("mdns: failed to bind any socket"));
        s_mdns_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    uint8_t* rx_buf = pvPortMalloc(MDNS_MAX_PACKET);
    uint8_t* tx_buf = pvPortMalloc(MDNS_MAX_PACKET);
    if (!rx_buf || !tx_buf) {
        if (rx_buf) vPortFree(rx_buf);
        if (tx_buf) vPortFree(tx_buf);
        if (s4 != FREERTOS_INVALID_SOCKET) FreeRTOS_closesocket(s4);
        if (s6 != FREERTOS_INVALID_SOCKET) FreeRTOS_closesocket(s6);
        s_mdns_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    BackoffAlgorithmContext_t retry_ctx;
    BackoffAlgorithm_InitializeParams(&retry_ctx, 250, 4000, 5);

    while (1) {
        Socket_t sockets[2] = {s4, s6};
        BaseType_t families[2] = {FREERTOS_AF_INET, FREERTOS_AF_INET6};

        for (int i = 0; i < 2; i++) {
            Socket_t s = sockets[i];
            if (s == FREERTOS_INVALID_SOCKET) continue;

            struct freertos_sockaddr from;
            socklen_t from_len = sizeof(from);
            int n = FreeRTOS_recvfrom(s, rx_buf, MDNS_MAX_PACKET, 0, &from,
                                      &from_len);

            if (n >= (int)sizeof(DNSHeader_t)) {
                DNSHeader_t* h = (DNSHeader_t*)rx_buf;
                uint16_t flags = FreeRTOS_ntohs(h->usFlags);
                uint16_t qdcount = FreeRTOS_ntohs(h->usQuestions);

                if ((flags & 0x8000) == 0 && qdcount > 0) {
                    mdns_resp_ctx_t ctx = {.buf = tx_buf,
                                           .p = tx_buf + sizeof(DNSHeader_t),
                                           .end = tx_buf + MDNS_MAX_PACKET,
                                           .ancount = 0};
                    const uint8_t* curr = rx_buf + sizeof(DNSHeader_t);
                    bool unicast_response = false;

                    matter_mdns_lock();
                    for (int j = 0; j < qdcount; j++) {
                        char qname[128];
                        const uint8_t* next = NULL;
                        dns_read_name(rx_buf, (size_t)n, curr, qname,
                                      sizeof(qname), &next);
                        if (!next || next + 4 > rx_buf + n) break;
                        uint16_t qtype = (next[0] << 8) | next[1];
                        uint16_t qclass = (next[2] << 8) | next[3];
                        curr = next + 4;

                        if ((qclass & 0x8000)) unicast_response = true;
                        process_query(&ctx, qname, qtype);
                    }
                    matter_mdns_unlock();

                    if (ctx.ancount > 0) {
                        DNSHeader_t* resp_h = (DNSHeader_t*)tx_buf;
                        resp_h->usID = h->usID;
                        resp_h->usFlags = FreeRTOS_htons(DNS_FLAGS_RESPONSE);
                        resp_h->usQuestions = 0;
                        resp_h->usAnswers = FreeRTOS_htons(ctx.ancount);
                        resp_h->usAuthoritative = 0;
                        resp_h->usAdditional = 0;

                        struct freertos_sockaddr dest;
                        if (unicast_response) {
                            dest = from;
                        } else {
                            dest.sin_port = FreeRTOS_htons(MDNS_PORT);
                            dest.sin_family = families[i];
                            if (families[i] == FREERTOS_AF_INET) {
                                dest.sin_address.ulIP_IPv4 =
                                    FreeRTOS_inet_addr(MDNS_V4_ADDR);
                            } else {
                                FreeRTOS_inet_pton(
                                    FREERTOS_AF_INET6, MDNS_V6_ADDR,
                                    dest.sin_address.xIP_IPv6.ucBytes);
                            }
                        }
                        FreeRTOS_sendto(s, tx_buf, (size_t)(ctx.p - tx_buf), 0,
                                        &dest, sizeof(dest));
                    }
                }
            }
        }

        uint32_t notify_val = 0;
        if (xTaskNotifyWait(0, 0xffffffff, &notify_val, 0) == pdTRUE) {
            LogInfo(("mdns: starting announcement sequence"));
            BackoffAlgorithmContext_t retry_ctx;
            BackoffAlgorithm_InitializeParams(&retry_ctx, 250, 4000, 2);
            BackoffAlgorithmStatus_t retry_status = BackoffAlgorithmSuccess;
            uint16_t next_backoff = 0;

            do {
                mdns_resp_ctx_t ctx = {.buf = tx_buf,
                                       .p = tx_buf + sizeof(DNSHeader_t),
                                       .end = tx_buf + MDNS_MAX_PACKET,
                                       .ancount = 0};
                matter_mdns_lock();
                size_t count;
                const matter_mdns_service_t* services =
                    matter_mdns_locked_services(&count);
                const char* hostname = matter_mdns_locked_hostname();
                for (size_t i = 0; i < count; i++) {
                    char sn[64];
                    snprintf(sn, sizeof(sn), "%s.%s.local", services[i].service,
                             services[i].proto);
                    add_service_records(&ctx, &services[i], true, sn);
                    dns_add_ptr(&ctx, "_services._dns-sd._udp.local", sn,
                                false);
                }
                if (hostname) {
                    dns_add_a(&ctx, hostname);
                    dns_add_aaaa(&ctx, hostname);
                }
                matter_mdns_unlock();

                if (ctx.ancount > 0) {
                    DNSHeader_t* h = (DNSHeader_t*)tx_buf;
                    h->usID = 0;
                    h->usFlags = FreeRTOS_htons(DNS_FLAGS_RESPONSE);
                    h->usQuestions = 0;
                    h->usAnswers = FreeRTOS_htons(ctx.ancount);
                    h->usAuthoritative = 0;
                    h->usAdditional = 0;

                    for (int i = 0; i < 2; i++) {
                        Socket_t s = sockets[i];
                        if (s == FREERTOS_INVALID_SOCKET) continue;
                        struct freertos_sockaddr dest;
                        dest.sin_port = FreeRTOS_htons(MDNS_PORT);
                        dest.sin_family = families[i];
                        if (families[i] == FREERTOS_AF_INET) {
                            dest.sin_address.ulIP_IPv4 =
                                FreeRTOS_inet_addr(MDNS_V4_ADDR);
                        } else {
                            FreeRTOS_inet_pton(
                                FREERTOS_AF_INET6, MDNS_V6_ADDR,
                                dest.sin_address.xIP_IPv6.ucBytes);
                        }
                        FreeRTOS_sendto(s, tx_buf, (size_t)(ctx.p - tx_buf), 0,
                                        &dest, sizeof(dest));
                    }
                }

                retry_status = BackoffAlgorithm_GetNextBackoff(
                    &retry_ctx, (uint32_t)rand(), &next_backoff);
                if (retry_status == BackoffAlgorithmSuccess) {
                    vTaskDelay(pdMS_TO_TICKS(next_backoff));
                }
            } while (retry_status != BackoffAlgorithmRetriesExhausted);
        }
    }
}

void mdns_responder_init(void) {
    if (!s_mdns_task) {
        xTaskCreate(mdns_responder_task, "mdns_resp", MDNS_TASK_STACK, NULL,
                    tskIDLE_PRIORITY + 1, &s_mdns_task);
    }
}

void mdns_responder_request_announce(void) {
    if (s_mdns_task) xTaskNotify(s_mdns_task, 1, eSetBits);
}
