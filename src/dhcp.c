#include "dhcp.h"

#include <stdlib.h>

// FreeRTOS
#include "FreeRTOS_IP.h"
#include "task.h"

// wmsdk
#include "crc32.h"
#include "wmtime.h"

// Application
#include "app_logging.h"
#include "dhcp.h"
#include "util.h"

// lease time 24 hours
#define MAX_LEASES 12

/* a DHCP client must be prepared to receive a message of up to 576 octets */
#define DHCP_MESSAGE_SIZE 576

/* UDP port numbers for DHCP */
#define DHCP_SERVER_PORT 67 /* port for server to listen on */
#define DHCP_CLIENT_PORT 68 /* port for client to use */

#define DNS_SERVER_PORT 53 /* port for server to listed on */

/* DHCP message OP code */
#define DHCP_BOOTREQUEST 1
#define DHCP_BOOTREPLY 2

/* DHCP message type */
#define DHCP_DISCOVER 1
#define DHCP_OFFER 2
#define DHCP_REQUEST 3
#define DHCP_DECLINE 4
#define DHCP_ACK 5
#define DHCP_NAK 6
#define DHCP_RELEASE 7
#define DHCP_INFORM 8

#define DHCP_LEASETIME \
    ((long)60 * 60 * 24)  // Lease Time: 1 day == { 00, 01, 51, 80 }

/* DHCP lease status */
#define DHCP_LEASE_AVAIL 0
#define DHCP_LEASE_OFFER 1
#define DHCP_LEASE_ACK 2

typedef struct {
    uint32_t maccrc;
    time_t expires;
    int status;
    uint32_t hostcrc;
} lease_t;

/**
 * @brief	DHCP option and value (cf. RFC1533)
 */
enum {
    dhcpPadOption = 0,
    dhcpSubnetMask = 1,
    dhcpTimerOffset = 2,
    dhcpRoutersOnSubnet = 3,
    dhcpTimeServer = 4,
    dhcpNameServer = 5,
    dhcpDns = 6,
    dhcpLogServer = 7,
    dhcpCookieServer = 8,
    dhcpLprServer = 9,
    dhcpImpressServer = 10,
    dhcpResourceLocationServer = 11,
    dhcpHostName = 12,
    dhcpBootFileSize = 13,
    dhcpMeritDumpFile = 14,
    dhcpDomainName = 15,
    dhcpSwapServer = 16,
    dhcpRootPath = 17,
    dhcpExtentionsPath = 18,
    dhcpIPforwarding = 19,
    dhcpNonLocalSourceRouting = 20,
    dhcpPolicyFilter = 21,
    dhcpMaxDgramReasmSize = 22,
    dhcpDefaultIPTTL = 23,
    dhcpPathMTUagingTimeout = 24,
    dhcpPathMTUplateauTable = 25,
    dhcpIfMTU = 26,
    dhcpAllSubnetsLocal = 27,
    dhcpBroadcastAddr = 28,
    dhcpPerformMaskDiscovery = 29,
    dhcpMaskSupplier = 30,
    dhcpPerformRouterDiscovery = 31,
    dhcpRouterSolicitationAddr = 32,
    dhcpStaticRoute = 33,
    dhcpTrailerEncapsulation = 34,
    dhcpArpCacheTimeout = 35,
    dhcpEthernetEncapsulation = 36,
    dhcpTcpDefaultTTL = 37,
    dhcpTcpKeepaliveInterval = 38,
    dhcpTcpKeepaliveGarbage = 39,
    dhcpNisDomainName = 40,
    dhcpNisServers = 41,
    dhcpNtpServers = 42,
    dhcpVendorSpecificInfo = 43,
    dhcpNetBIOSnameServer = 44,
    dhcpNetBIOSdgramDistServer = 45,
    dhcpNetBIOSnodeType = 46,
    dhcpNetBIOSscope = 47,
    dhcpXFontServer = 48,
    dhcpXDisplayManager = 49,
    dhcpRequestedIPaddr = 50,
    dhcpIPaddrLeaseTime = 51,
    dhcpOptionOverload = 52,
    dhcpMessageType = 53,
    dhcpServerIdentifier = 54,
    dhcpParamRequest = 55,
    dhcpMsg = 56,
    dhcpMaxMsgSize = 57,
    dhcpT1value = 58,
    dhcpT2value = 59,
    dhcpClassIdentifier = 60,
    dhcpClientIdentifier = 61,
    dhcpEndOption = 255
};

/**
 * @brief DHCP message
 */
typedef struct {
    uint8_t op;
    uint8_t htype;
    uint8_t hlen;
    uint8_t hops;
    uint32_t xid;
    uint16_t secs;
#define DHCP_FLAG_BROADCAST (0x8000)
    uint16_t flags;
    uint8_t ciaddr[4];   // Client IP
    uint8_t yiaddr[4];   // Your IP
    uint8_t siaddr[4];   // Server IP
    uint8_t giaddr[4];   // Gateway IP
    uint8_t chaddr[16];  // Client hardware address (zero padded)
    uint8_t sname[64];
    uint8_t file[128];
#define DHCP_MAGIC (0x63825363)
    uint8_t magic[4];
    uint8_t opt[];  // 240 offset
} __attribute__((packed)) dhcp_msg_t;
_Static_assert(sizeof(dhcp_msg_t) == 240, "msg size");

static uint8_t buf[DHCP_MESSAGE_SIZE];
static lease_t leases[MAX_LEASES];

uint8_t *long2quad(unsigned long value) {
    static uint8_t quads[4];
    for (int k = 0; k < 4; k++) quads[3 - k] = value >> (k * 8);
    return quads;
}

int get_option(int option, uint32_t *options, int optionSize,
               int *optionLength) {
    for (int i = 0; i < optionSize && (options[i] != dhcpEndOption);
         i += 2 + options[i + 1]) {
        if (options[i] == option) {
            if (optionLength) {
                *optionLength = (int)options[i + 1];
            }
            return i + 2;
        }
    }
    if (optionLength) {
        *optionLength = 0;
    }
    return 0;
}

void set_lease(int i, uint32_t crc, long expires, int status,
               uint32_t hostcrc) {
    if (i >= 0 && i < MAX_LEASES) {
        leases[i].maccrc = crc;
        leases[i].expires = expires;
        leases[i].status = status;
        leases[i].hostcrc = hostcrc;
    }
}

int get_lease(unsigned long crc) {
    for (int i = 0; i < MAX_LEASES; i++) {
        if (leases[i].maccrc == crc) {
            return i;
        }
    }

    // Clean up expired leases; need to do after we check for existing leases
    // because of this iOS bug
    // http://www.net.princeton.edu/apple-ios/ios41-allows-lease-to-expire-keeps-using-IP-address.html
    // Don't need to check again AFTER the clean up as for DHCP REQUEST the
    // client should already have the lease and for DHCP DISCOVER we will check
    // once more to assign a new lease
    time_t now = wmtime_time_get_posix();
    for (int i = 0; i < MAX_LEASES; i++) {
        if (leases[i].expires < now) {
            set_lease(i, 0, 0, DHCP_LEASE_AVAIL, 0);
        }
    }
    return -1;
}

int get_lease_by_host(uint32_t crc) {
    for (int i = 0; i < MAX_LEASES; i++) {
        if (leases[i].hostcrc == crc && leases[i].status == DHCP_LEASE_ACK) {
            return i;
        }
    }
    return -1;
}

int populate_packet(uint8_t *packet, int currLoc, uint8_t marker, uint8_t *what,
                    int dataSize) {
    packet[currLoc] = marker;
    packet[currLoc + 1] = dataSize;
    memcpy(packet + currLoc + 2, what, dataSize);
    return dataSize + 2;
}

int process_dhcp_msg(dhcp_msg_t *packet, int32_t packetSize, uint32_t serverIP,
                     uint32_t netmask) {
    if (packet->op != DHCP_BOOTREQUEST) {
        return 0;
    }

    int opt_offset = (uint8_t *)packet->opt - (uint8_t *)packet;

    packet->op = DHCP_BOOTREPLY;
    packet->secs = 0;
    uint32_t crc = crc32(packet->chaddr, packet->hlen, 0);

    int ofs = get_option(dhcpMessageType, (uint32_t *)packet->opt,
                         packetSize - opt_offset, NULL);
    uint8_t msg_type = packet->opt[ofs];
    debug_hexdump('R', (uint8_t *)packet, packetSize);

    int lease = get_lease(crc);
    uint8_t response = DHCP_NAK;
    time_t now = wmtime_time_get_posix();
    if (msg_type == DHCP_DISCOVER) {
        if (lease == -1) {
            // use existing lease or get a new one
            lease = get_lease(0);
        }
        if (lease == -1) {
            response = DHCP_OFFER;
            // offer valid 10 seconds
            set_lease(lease, crc, now + 10000, DHCP_LEASE_OFFER, 0);
        }
    } else if (msg_type == DHCP_REQUEST) {
        if (lease != -1) {
            response = DHCP_ACK;

            // find hostname option in the request and store to provide DNS info
            int hostNameLength;
            int hostNameOffset =
                get_option(dhcpHostName, (uint32_t *)packet->opt,
                           packetSize - opt_offset, &hostNameLength);
            unsigned long nameCrc =
                hostNameOffset
                    ? crc32(packet->opt + hostNameOffset, hostNameLength, 0)
                    : 0;
            set_lease(lease, crc, now + DHCP_LEASETIME * 1000, DHCP_LEASE_ACK,
                      nameCrc);  // DHCP_LEASETIME is in seconds
        }
    }

    if (lease != -1) {
        // Dynamic IP configuration
        *(uint32_t *)packet->yiaddr = serverIP;
        packet->yiaddr[3] += lease + 1;
    }

    int currLoc = 0;
    packet->opt[currLoc++] = dhcpMessageType;
    packet->opt[currLoc++] = 1;
    packet->opt[currLoc++] = response;

    int reqLength;
    int reqListOffset = get_option(dhcpParamRequest, (uint32_t *)packet->opt,
                                   packetSize - opt_offset, &reqLength);
    uint8_t reqList[12];
    if (reqLength > 12) {
        reqLength = 12;
    }
    memcpy(reqList, packet->opt + reqListOffset, reqLength);

    // iPod with iOS 4 doesn't want to process DHCP OFFER if
    // dhcpServerIdentifier does not follow dhcpMessageType Windows Vista
    // and Ubuntu 11.04 don't seem to care
    currLoc += populate_packet(packet->opt, currLoc, dhcpServerIdentifier,
                               (uint8_t *)&serverIP, 4);

    // DHCP lease timers:
    // http://www.tcpipguide.com/free/t_DHCPLeaseLifeCycleOverviewAllocationReallocationRe.htm
    // Renewal Timer (T1): This timer is set by default to 50% of the lease
    // period. Rebinding Timer (T2): This timer is set by default to 87.5%
    // of the length of the lease.
    currLoc += populate_packet(packet->opt, currLoc, dhcpIPaddrLeaseTime,
                               long2quad(DHCP_LEASETIME), 4);
    currLoc += populate_packet(packet->opt, currLoc, dhcpT1value,
                               long2quad(DHCP_LEASETIME * 0.5), 4);
    currLoc += populate_packet(packet->opt, currLoc, dhcpT2value,
                               long2quad(DHCP_LEASETIME * 0.875), 4);

    for (int i = 0; i < reqLength; i++) {
        switch (reqList[i]) {
            case dhcpSubnetMask:
                currLoc += populate_packet(packet->opt, currLoc, reqList[i],
                                           (uint8_t *)&netmask, 4);
                break;
            case dhcpLogServer:
                currLoc += populate_packet(packet->opt, currLoc, reqList[i],
                                           long2quad(0), 4);
                break;
        }
    }
    packet->opt[currLoc++] = dhcpEndOption;

    return opt_offset + currLoc;
}

void dhcpd_task(void *params) {
    // make a deep copy
    dhcp_task_params_t p;
    memcpy(&p, params, sizeof(dhcp_task_params_t));

    Socket_t listen_sock = FreeRTOS_socket(
        FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);
    if (listen_sock == FREERTOS_INVALID_SOCKET) {
        LogError(("failed to open socket"));
        goto err;
    }

    struct freertos_sockaddr bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_port = FreeRTOS_htons(DHCP_SERVER_PORT);
    bind_addr.sin_family = FREERTOS_AF_INET;
    BaseType_t res = FreeRTOS_bind(listen_sock, &bind_addr, sizeof(bind_addr));
    if (res) {
        LogError(("bind failed %d", res));
        goto err;
    }
    LogInfo(("DHCP server running"));

    for (;;) {
        struct freertos_sockaddr client;
        uint32_t client_len = sizeof(client);
        int32_t n_bytes = FreeRTOS_recvfrom(listen_sock, buf, sizeof(buf), 0,
                                            &client, &client_len);

        if (n_bytes >= 0) {
            LogInfo(("got %d bytes", n_bytes));
            process_dhcp_msg((dhcp_msg_t *)buf, n_bytes, p.ip_addr, p.netmask);
        } else {
            FreeRTOS_strerror_r(n_bytes, (char *)buf, sizeof(buf));
            LogWarn(("socket read failed: %s", buf));
        }
    }
err:
    vTaskDelete(NULL);
}