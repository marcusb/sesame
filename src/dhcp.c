#include "dhcp.h"

#include <stdlib.h>

// FreeRTOS
#include "FreeRTOS_DHCP.h"
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

#define dhcpIPv4_RENEWAL_TIME_OPTION_CODE 58
#define dhcpIPv4_REBINDING_TIME_OPTION_CODE 59

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

static uint8_t buf[DHCP_MESSAGE_SIZE];
static lease_t leases[MAX_LEASES];

static uint8_t *get_option(int option, const DHCPMessage_IPv4_t *msg,
                           int msg_size, int *optionLength) {
    for (uint8_t *p = (uint8_t *)msg + dhcpFIRST_OPTION_BYTE_OFFSET;
         p < (uint8_t *)msg + msg_size && *p != dhcpOPTION_END_BYTE; p++) {
        uint8_t opt = *p++;
        if (opt == option) {
            if (optionLength) {
                *optionLength = *p;
            }
            return p + 1;
        } else if (opt == dhcpIPv4_ZERO_PAD_OPTION_CODE) {
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

static void set_lease(int i, uint32_t crc, long expires, int status,
                      uint32_t hostcrc) {
    if (i >= 0 && i < MAX_LEASES) {
        leases[i].maccrc = crc;
        leases[i].expires = expires;
        leases[i].status = status;
        leases[i].hostcrc = hostcrc;
    }
}

static int get_lease(unsigned long crc) {
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

static int put_option(uint8_t *p, uint8_t code, int length,
                      const uint8_t *value) {
    *p++ = code;
    *p++ = length;
    memcpy(p, value, length);
    return length + 2;
}

static int put_long_option(uint8_t *p, uint8_t code, uint32_t value) {
    *p++ = code;
    *p++ = 4;
    *(uint32_t *)p = FreeRTOS_htonl(value);
    return 6;
}

// Send a DHCP reply packet. Because the client does not yet have an
// IP address, it does not respond to ARP, so we cannot use the regular UDP
// sendto function, and have to send a raw IP frame instead.
static void send_dhcp_reply(NetworkEndPoint_t *endpoint,
                            DHCPMessage_IPv4_t *msg, int32_t msg_size) {
    NetworkBufferDescriptor_t *buf =
        pxGetNetworkBufferWithDescriptor(sizeof(UDPPacket_t) + msg_size, 0);
    if (buf == NULL) {
        return;
    }
    buf->xIPAddress.ulIP_IPv4 = msg->ulYourIPAddress_yiaddr;
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
    udp_header->usDestinationPort = dhcpCLIENT_PORT_IPv4;
    udp_header->usSourcePort = dhcpSERVER_PORT_IPv4;
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
    memcpy(eth_header->xDestinationAddress.ucBytes,
           msg->ucClientHardwareAddress, ipMAC_ADDRESS_LENGTH_BYTES);

    debug_hexdump('T', buf->pucEthernetBuffer, buf->xDataLength);
    IPStackEvent_t ev = {eNetworkTxEvent, buf};
    if (xSendEventStructToIPTask(&ev, 0) != pdPASS) {
        LogWarn(("failed to send DHCP reply"));
    }
}

static void process_dhcp_msg(NetworkEndPoint_t *endpoint,
                             DHCPMessage_IPv4_t *packet, int32_t packet_size) {
    if (packet->ucOpcode != dhcpREQUEST_OPCODE) {
        return;
    }
    debug_hexdump('R', (uint8_t *)packet, packet_size);

    packet->ucOpcode = dhcpREPLY_OPCODE;
    packet->usElapsedTime = 0;
    uint32_t crc =
        crc32(packet->ucClientHardwareAddress, packet->ucAddressLength, 0);

    uint8_t *msg_type_p = get_option(dhcpIPv4_MESSAGE_TYPE_OPTION_CODE, packet,
                                     packet_size, NULL);
    if (msg_type_p == NULL) {
        LogWarn(("DHCP message type missing"));
        return;
    }
    uint8_t msg_type = *msg_type_p;

    int lease = get_lease(crc);
    uint8_t response = dhcpMESSAGE_TYPE_NACK;
    time_t now = wmtime_time_get_posix();
    if (msg_type == dhcpMESSAGE_TYPE_DISCOVER) {
        if (lease == -1) {
            // use existing lease or get a new one
            lease = get_lease(0);
        }
        if (lease != -1) {
            response = dhcpMESSAGE_TYPE_OFFER;
            // offer valid 10 seconds
            set_lease(lease, crc, now + 10000, DHCP_LEASE_OFFER, 0);
        }
    } else if (msg_type == dhcpMESSAGE_TYPE_REQUEST) {
        if (lease != -1) {
            response = dhcpMESSAGE_TYPE_ACK;

            // find hostname option in the request and store to provide DNS info
            int hostNameLength;
            uint8_t *hostname =
                get_option(dhcpIPv4_DNS_HOSTNAME_OPTIONS_CODE, packet,
                           packet_size, &hostNameLength);
            unsigned long nameCrc =
                hostname ? crc32(hostname, hostNameLength, 0) : 0;
            set_lease(lease, crc, now + DHCP_LEASETIME * 1000, DHCP_LEASE_ACK,
                      nameCrc);
        }
    }

    if (lease != -1) {
        // Dynamic IP configuration
        packet->ulYourIPAddress_yiaddr = endpoint->ipv4_settings.ulIPAddress;
        *((uint8_t *)&packet->ulYourIPAddress_yiaddr + 3) += lease + 1;
    }

    uint8_t *p = (uint8_t *)packet + dhcpFIRST_OPTION_BYTE_OFFSET;
    *p++ = dhcpIPv4_MESSAGE_TYPE_OPTION_CODE;
    *p++ = 1;
    *p++ = response;

    int reqLength;
    uint8_t *reqs = get_option(dhcpIPv4_PARAMETER_REQUEST_OPTION_CODE, packet,
                               packet_size, &reqLength);
    uint8_t reqList[12];
    if (reqLength > 12) {
        reqLength = 12;
    }
    memcpy(reqList, reqs, reqLength);

    // iPod with iOS 4 doesn't want to process DHCP OFFER if
    // dhcpServerIdentifier does not follow dhcpMessageType Windows Vista
    // and Ubuntu 11.04 don't seem to care
    p += put_option(p, dhcpIPv4_SERVER_IP_ADDRESS_OPTION_CODE, 4,
                    (uint8_t *)&endpoint->ipv4_settings.ulIPAddress);

    // DHCP lease timers:
    // http://www.tcpipguide.com/free/t_DHCPLeaseLifeCycleOverviewAllocationReallocationRe.htm
    // Renewal Timer (T1): This timer is set by default to 50% of the lease
    // period. Rebinding Timer (T2): This timer is set by default to 87.5%
    // of the length of the lease.
    p += put_long_option(p, dhcpIPv4_LEASE_TIME_OPTION_CODE, DHCP_LEASETIME);
    p += put_long_option(p, dhcpIPv4_RENEWAL_TIME_OPTION_CODE,
                         DHCP_LEASETIME * 0.5);
    p += put_long_option(p, dhcpIPv4_REBINDING_TIME_OPTION_CODE,
                         DHCP_LEASETIME * 0.875);

    for (int i = 0; i < reqLength; i++) {
        switch (reqList[i]) {
            case dhcpIPv4_SUBNET_MASK_OPTION_CODE:
                p += put_option(p, reqList[i], 4,
                                (uint8_t *)&endpoint->ipv4_settings.ulNetMask);
                break;
        }
    }
    *p++ = dhcpOPTION_END_BYTE;

    send_dhcp_reply(endpoint, packet, p - (uint8_t *)packet);
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
    bind_addr.sin_port = dhcpSERVER_PORT_IPv4;
    bind_addr.sin_family = FREERTOS_AF_INET;
    BaseType_t res = FreeRTOS_bind(socket, &bind_addr, sizeof(bind_addr));
    if (res) {
        LogError(("bind failed %d", res));
        goto err;
    }
    LogInfo(("DHCP server running"));

    for (;;) {
        struct freertos_sockaddr client;
        uint32_t client_len = sizeof(client);
        int32_t n_bytes = FreeRTOS_recvfrom(socket, buf, sizeof(buf), 0,
                                            &client, &client_len);

        if (n_bytes >= 0) {
            process_dhcp_msg(endpoint, (DHCPMessage_IPv4_t *)buf, n_bytes);
        } else {
            // FreeRTOS_strerror_r(n_bytes, (char *)buf, sizeof(buf));
            // LogWarn(("socket read failed: %s", buf));
            LogWarn(("socket read failed: %d", n_bytes));
        }
    }
err:
    vTaskDelete(NULL);
}
