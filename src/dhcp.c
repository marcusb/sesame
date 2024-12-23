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

uint8_t *get_option(int option, dhcp_msg_t *msg, int msg_size,
                    int *optionLength) {
    for (uint8_t *p = msg->opt;
         p < (uint8_t *)msg + msg_size && *p != dhcpEndOption; p++) {
        uint8_t opt = *p++;
        if (opt == option) {
            if (optionLength) {
                *optionLength = *p;
            }
            return p + 1;
        } else if (opt == dhcpPadOption) {
            continue;
        } else {
            p += *p;
        }
    }
    if (optionLength) {
        *optionLength = 0;
    }
    return NULL;
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

// Send a DHCP reply packet. Because the client does not yet have an
// IP address, it does not respond to ARP, so we cannot use the  regular UDP
// sendto function, and have to send a raw IP frame instead.
void send_dhcp_reply(NetworkEndPoint_t *endpoint, dhcp_msg_t *msg,
                     int32_t msg_size) {
    NetworkBufferDescriptor_t *buf =
        pxGetNetworkBufferWithDescriptor(sizeof(UDPPacket_t) + msg_size, 0);
    if (buf == NULL) {
        return;
    }
    buf->xIPAddress.ulIP_IPv4 = *(uint32_t *)msg->yiaddr;
    buf->pxEndPoint = endpoint;
    buf->pxInterface = endpoint->pxNetworkInterface;
    buf->xDataLength = sizeof(UDPPacket_t) + msg_size;

    // clang-format off
    static const uint8_t common_frame_header[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Ethernet source MAC address. */
        0x08, 0x00,                         /* Ethernet frame type. */
        ipIPV4_VERSION_HEADER_LENGTH_MIN,   /* ucVersionHeaderLength. */
        0x00,                               /* ucDifferentiatedServicesCode. */
        0x00, 0x00,                         /* usLength. */
        0x00, 0x00,                         /* usIdentification. */
        0x00, 0x00,                         /* usFragmentOffset. */
        ipconfigUDP_TIME_TO_LIVE,           /* ucTimeToLive */
        ipPROTOCOL_UDP,                     /* ucProtocol. */
        0x00, 0x00,                         /* usHeaderChecksum. */
        0x00, 0x00, 0x00, 0x00              /* Source IP address. */
    };
    // clang-format on

    // copy the constant part of the UDP header to the buffer, starting at the
    // source MAC address (after the destination MAC address)
    memcpy(buf->pucEthernetBuffer + sizeof(MACAddress_t), common_frame_header,
           sizeof(common_frame_header));

    UDPPacket_t *udp_packet = ((UDPPacket_t *)buf->pucEthernetBuffer);
    UDPHeader_t *udp_header = &(udp_packet->xUDPHeader);
    udp_header->usDestinationPort = FreeRTOS_htons(DHCP_CLIENT_PORT);
    udp_header->usSourcePort = FreeRTOS_htons(DHCP_SERVER_PORT);
    udp_header->usLength = FreeRTOS_htons(msg_size + sizeof(UDPHeader_t));
    udp_header->usChecksum = 0;
    memcpy(buf->pucEthernetBuffer + sizeof(UDPPacket_t), msg, msg_size);

    IPHeader_t *ip_header = &udp_packet->xIPHeader;
    ip_header->usLength =
        FreeRTOS_htons(msg_size + sizeof(IPHeader_t) + sizeof(UDPHeader_t));
    ip_header->ulDestinationIPAddress = buf->xIPAddress.ulIP_IPv4;
    ip_header->ulSourceIPAddress = buf->pxEndPoint->ipv4_settings.ulIPAddress;
    ip_header->usFragmentOffset = 0;
    ip_header->usHeaderChecksum = 0U;
    ip_header->usHeaderChecksum = usGenerateChecksum(
        0, &ip_header->ucVersionHeaderLength, uxIPHeaderSizePacket(buf));
    ip_header->usHeaderChecksum = ~FreeRTOS_htons(ip_header->usHeaderChecksum);
    usGenerateProtocolChecksum((uint8_t *)udp_packet, buf->xDataLength, pdTRUE);

    EthernetHeader_t *eth_header = (EthernetHeader_t *)buf->pucEthernetBuffer;
    memcpy(eth_header->xSourceAddress.ucBytes,
           buf->pxEndPoint->xMACAddress.ucBytes, ipMAC_ADDRESS_LENGTH_BYTES);
    memcpy(eth_header->xDestinationAddress.ucBytes, msg->chaddr,
           ipMAC_ADDRESS_LENGTH_BYTES);

    debug_hexdump('T', buf->pucEthernetBuffer, buf->xDataLength);
    IPStackEvent_t ev = {eNetworkTxEvent, buf};
    if (xSendEventStructToIPTask(&ev, 0) != pdPASS) {
        LogWarn(("failed to send DHCP reply"));
    }
}

void process_dhcp_msg(NetworkEndPoint_t *endpoint, dhcp_msg_t *packet,
                      int32_t packet_size) {
    if (packet->op != DHCP_BOOTREQUEST) {
        return;
    }
    debug_hexdump('R', (uint8_t *)packet, packet_size);

    int opt_offset = (uint8_t *)packet->opt - (uint8_t *)packet;

    packet->op = DHCP_BOOTREPLY;
    packet->secs = 0;
    uint32_t crc = crc32(packet->chaddr, packet->hlen, 0);

    uint8_t *msg_type_p =
        get_option(dhcpMessageType, packet, packet_size, NULL);
    if (msg_type_p == NULL) {
        LogWarn(("DHCP message type missing"));
        return;
    }
    uint8_t msg_type = *msg_type_p;

    int lease = get_lease(crc);
    uint8_t response = DHCP_NAK;
    time_t now = wmtime_time_get_posix();
    if (msg_type == DHCP_DISCOVER) {
        if (lease == -1) {
            // use existing lease or get a new one
            lease = get_lease(0);
        }
        if (lease != -1) {
            response = DHCP_OFFER;
            // offer valid 10 seconds
            set_lease(lease, crc, now + 10000, DHCP_LEASE_OFFER, 0);
        }
    } else if (msg_type == DHCP_REQUEST) {
        if (lease != -1) {
            response = DHCP_ACK;

            // find hostname option in the request and store to provide DNS info
            int hostNameLength;
            uint8_t *hostname =
                get_option(dhcpHostName, packet, packet_size, &hostNameLength);
            unsigned long nameCrc =
                hostname ? crc32(hostname, hostNameLength, 0) : 0;
            set_lease(lease, crc, now + DHCP_LEASETIME * 1000, DHCP_LEASE_ACK,
                      nameCrc);
        }
    }

    if (lease != -1) {
        // Dynamic IP configuration
        *(uint32_t *)packet->yiaddr = endpoint->ipv4_settings.ulIPAddress;
        packet->yiaddr[3] += lease + 1;
    }

    int currLoc = 0;
    packet->opt[currLoc++] = dhcpMessageType;
    packet->opt[currLoc++] = 1;
    packet->opt[currLoc++] = response;

    int reqLength;
    uint8_t *reqs =
        get_option(dhcpParamRequest, packet, packet_size, &reqLength);
    uint8_t reqList[12];
    if (reqLength > 12) {
        reqLength = 12;
    }
    memcpy(reqList, reqs, reqLength);

    // iPod with iOS 4 doesn't want to process DHCP OFFER if
    // dhcpServerIdentifier does not follow dhcpMessageType Windows Vista
    // and Ubuntu 11.04 don't seem to care
    currLoc +=
        populate_packet(packet->opt, currLoc, dhcpServerIdentifier,
                        (uint8_t *)&endpoint->ipv4_settings.ulIPAddress, 4);

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
                currLoc += populate_packet(
                    packet->opt, currLoc, reqList[i],
                    (uint8_t *)&endpoint->ipv4_settings.ulNetMask, 4);
                break;
            case dhcpLogServer:
                currLoc += populate_packet(packet->opt, currLoc, reqList[i],
                                           long2quad(0), 4);
                break;
        }
    }
    packet->opt[currLoc++] = dhcpEndOption;

    send_dhcp_reply(endpoint, packet, opt_offset + currLoc);
}

void dhcpd_task(void *const params) {
    dhcp_task_params_t *const task_params = (dhcp_task_params_t *)params;
    NetworkEndPoint_t *endpoint = task_params->endpoint;

    Socket_t socket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM,
                                      FREERTOS_IPPROTO_UDP);
    if (socket == FREERTOS_INVALID_SOCKET) {
        LogError(("failed to open socket"));
        goto err;
    }
    const TickType_t recv_tmout = portMAX_DELAY;
    FreeRTOS_setsockopt(socket, 0, FREERTOS_SO_RCVTIMEO, &recv_tmout, 0);
    const TickType_t send_tmout = 0;
    FreeRTOS_setsockopt(socket, 0, FREERTOS_SO_SNDTIMEO, &send_tmout, 0);

    struct freertos_sockaddr bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_port = FreeRTOS_htons(DHCP_SERVER_PORT);
    bind_addr.sin_family = FREERTOS_AF_INET;
    BaseType_t res = FreeRTOS_bind(socket, &bind_addr, sizeof(bind_addr));
    if (res) {
        LogError(("bind failed %d", res));
        goto err;
    }
    LogInfo(("DHCP server running"));

    // Socket_t reply_sock = FreeRTOS_socket(FREERTOS_AF_INET,
    // FREERTOS_SOCK_DGRAM,
    //                                       FREERTOS_IPPROTO_UDP);
    // if (socket == FREERTOS_INVALID_SOCKET) {
    //     LogError(("failed to open socket"));
    //     goto err;
    // }

    for (;;) {
        struct freertos_sockaddr client;
        uint32_t client_len = sizeof(client);
        int32_t n_bytes = FreeRTOS_recvfrom(socket, buf, sizeof(buf), 0,
                                            &client, &client_len);

        if (n_bytes >= 0) {
            process_dhcp_msg(endpoint, (dhcp_msg_t *)buf, n_bytes);
        } else {
            // FreeRTOS_strerror_r(n_bytes, (char *)buf, sizeof(buf));
            // LogWarn(("socket read failed: %s", buf));
            LogWarn(("socket read failed: %d", n_bytes));
        }
    }
err:
    vTaskDelete(NULL);
}