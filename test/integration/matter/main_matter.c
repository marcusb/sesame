/*
 * Matter end-to-end integration SUT.
 *
 * Boots the harness, brings up FreeRTOS+TCP on a tap interface (static IP,
 * no DHCP server is assumed on the host side), and starts the real
 * Matter task once the network is up. The host-side pytest then drives
 * commissioning via chip-tool.
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_DNS_Globals.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_ND.h"
#include "NetworkInterface.h"
#include "controller.h"
#include "harness.h"
#include "matter_mdns.h"
#include "matter_task.h"
#include "psm.h"
#include "queue.h"
#include "task.h"

extern void mbedtls_hardware_init_hash(uint8_t* entropy, size_t len);

/* PSM handle the matter shim and config_manager reference. main.c owns this
 * symbol in production; we re-declare locally so qemu_psm.c can back it. */
psm_hnd_t psm_hnd;

static const uint8_t hwaddr[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

/* Tap subnet — must match the host-side `ip addr add` setup that the test
 * operator created out-of-band. Static IP because there is no DHCP server
 * on a freshly-created tap. */
static const uint8_t guest_ip_addr[4] = {10, 20, 30, 2};
static const uint8_t netmask[4] = {255, 255, 255, 0};
static const uint8_t gateway_addr[4] = {10, 20, 30, 1};
static const uint8_t dns_server_addr[4] = {10, 20, 30, 1};

static NetworkInterface_t eth_iface;

void configure_netif(void) {
    static NetworkEndPoint_t eps[3];

    extern NetworkInterface_t* pxMPS2_FillInterfaceDescriptor(
        BaseType_t xEMACIndex, NetworkInterface_t * pxInterface);
    pxMPS2_FillInterfaceDescriptor(0, &eth_iface);

    /* Ethernet IRQ priority — same value as qemu_main.c uses. */
    *(volatile uint32_t*)(0xE000E400 + 3 * 4) = (0xE0 << 8);

    FreeRTOS_FillEndPoint(&eth_iface, &eps[0], guest_ip_addr, netmask,
                          gateway_addr, dns_server_addr, hwaddr);
    eps[0].bits.bWantDHCP = pdFALSE;

#if ipconfigUSE_IPv6
    IPv6_Address_t local_prefix = {{0xfe, 0x80}};
    IPv6_Address_t gateway_addr_ip6 = {
        {0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};
    IPv6_Address_t eui64_addr = {{0xfe, 0x80, 0, 0, 0, 0, 0, 0,
                                  hwaddr[0] | 0x02, hwaddr[1], hwaddr[2], 0xff,
                                  0xfe, hwaddr[3], hwaddr[4], hwaddr[5]}};
    FreeRTOS_FillEndPoint_IPv6(&eth_iface, &eps[1], &eui64_addr, &local_prefix,
                               64, &gateway_addr_ip6, NULL, hwaddr);
#endif

    FreeRTOS_SetDNSIPPreference(xPreferenceIPv4);

    int res = FreeRTOS_IPInit_Multi();
    configASSERT(res);
}

static const char* dns_type_name(uint16_t t) {
    switch (t) {
        case dnsTYPE_A_HOST:
            return "A";
        case dnsTYPE_AAAA_HOST:
            return "AAAA";
        case dnsTYPE_PTR:
            return "PTR";
        case dnsTYPE_SRV:
            return "SRV";
        case dnsTYPE_TXT:
            return "TXT";
        default:
            return "?";
    }
}

static void dump_endpoints(void) {
    char line[200];
    NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(NULL);
    int idx = 0;
    while (ep != NULL) {
        uint32_t ip = ep->ipv4_settings.ulIPAddress;
        snprintf(line, sizeof(line),
                 "EP[%d] up=%d ipv6=%d ip4_settings=%u.%u.%u.%u "
                 "ip4_defaults=%u.%u.%u.%u",
                 idx, (int)ep->bits.bEndPointUp, (int)ep->bits.bIPv6,
                 (unsigned)((ip) & 0xff), (unsigned)((ip >> 8) & 0xff),
                 (unsigned)((ip >> 16) & 0xff), (unsigned)((ip >> 24) & 0xff),
                 (unsigned)((ep->ipv4_defaults.ulIPAddress) & 0xff),
                 (unsigned)((ep->ipv4_defaults.ulIPAddress >> 8) & 0xff),
                 (unsigned)((ep->ipv4_defaults.ulIPAddress >> 16) & 0xff),
                 (unsigned)((ep->ipv4_defaults.ulIPAddress >> 24) & 0xff));
        host_inspector_emit(line);
        ep = FreeRTOS_NextEndPoint(NULL, ep);
        idx++;
    }
}

static void dump_mdns(void) {
    DNSRecord_t* recs;
    UBaseType_t n = matter_mdns_snapshot(&recs);
    char line[320];
    snprintf(line, sizeof(line), "MDNS count=%u", (unsigned)n);
    host_inspector_emit(line);
    for (UBaseType_t i = 0; i < n; i++) {
        DNSRecord_t* r = &recs[i];
        const char* extra = "";
        char extra_buf[160];
        switch (r->usRecordType) {
            case dnsTYPE_PTR:
                snprintf(extra_buf, sizeof(extra_buf), " ->%s",
                         r->xData.pcPtrRecord ? r->xData.pcPtrRecord : "");
                extra = extra_buf;
                break;
            case dnsTYPE_SRV:
                snprintf(extra_buf, sizeof(extra_buf), " port=%u target=%s",
                         (unsigned)r->xData.xSrvRecord.usPort,
                         r->xData.xSrvRecord.pcTarget
                             ? r->xData.xSrvRecord.pcTarget
                             : "");
                extra = extra_buf;
                break;
            case dnsTYPE_TXT:
                snprintf(extra_buf, sizeof(extra_buf), " txtlen=%u",
                         (unsigned)(r->xData.pcTxtRecord
                                        ? strlen(r->xData.pcTxtRecord)
                                        : 0));
                extra = extra_buf;
                break;
        }
        snprintf(line, sizeof(line), "MDNS [%u] %s %s%s", (unsigned)i,
                 dns_type_name(r->usRecordType),
                 r->pcName ? r->pcName : "(null)", extra);
        host_inspector_emit(line);
    }
}

static void on_cmd(const char* line) {
    /*
     * Commands understood by matter_it:
     *   door_state STATE DIR POS
     *     STATE: 0=CLOSED 1=OPEN 2=UNKNOWN
     *     DIR:   0=DOWN 1=UP 2=STOPPED 3=UNKNOWN
     *     POS:   0..100
     *
     *   Synthesizes a CTRL_MSG_DOOR_STATE_UPDATE on ctrl_queue (which the
     *   inspector task echoes to the host) and pushes the same state into
     *   the Matter shadow so subscribed controllers receive an attribute
     *   report.
     *
     *   dump_mdns
     *     Emits one INSPECTOR line per registered mDNS record. Useful for
     *     tests that need to validate publication state without sniffing the
     *     wire (chip-tool / avahi-browse not always available in CI).
     */
    int state, dir, pos;
    if (sscanf(line, "door_state %d %d %d", &state, &dir, &pos) == 3) {
        door_state_msg_t st = {
            .state = (door_open_state_t)state,
            .direction = (door_direction_t)dir,
            .pos = (uint8_t)pos,
        };
        ctrl_msg_t msg = {
            .type = CTRL_MSG_DOOR_STATE_UPDATE,
            .msg = {.door_state = st},
        };
        xQueueSendToBack(ctrl_queue, &msg, 0);
        matter_report_door_state(&st);
    } else if (strcmp(line, "dump_mdns") == 0) {
        dump_mdns();
    } else if (strcmp(line, "dump_endpoints") == 0) {
        dump_endpoints();
    }
}

int main(void) {
    setup_heap();

    /* QEMU has no hardware entropy; seed the mbedtls entropy pool with the
     * same deterministic placeholder qemu_main.c uses, otherwise
     * ctr_drbg_random returns NO_SOURCE and Matter's Berry code spins
     * generating a passcode. */
    static uint8_t qemu_seed_entropy[32] = {
        0xa1, 0x5e, 0xc0, 0xde, 0xde, 0xad, 0xbe, 0xef, 0x00, 0x11, 0x22,
        0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
        0xee, 0xff, 0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe,
    };
    mbedtls_hardware_init_hash(qemu_seed_entropy, sizeof(qemu_seed_entropy));

    psm_module_init(NULL, &psm_hnd, NULL);

    harness_init();
    harness_on_network_up(matter_init);
    harness_on_cmd(on_cmd);
    harness_run();
    return 0;
}
