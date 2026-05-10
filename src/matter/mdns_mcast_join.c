#include "matter/mdns_mcast_join.h"

#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "app_logging.h"

#define TAG "mcast_join"

#define MIN_FRAME 60

static uint16_t inet_chksum(const uint8_t* data, size_t len, uint32_t init) {
    uint32_t sum = init;
    for (size_t i = 0; i + 1 < len; i += 2) {
        sum += ((uint16_t)data[i] << 8) | data[i + 1];
    }
    if (len & 1) {
        sum += (uint16_t)data[len - 1] << 8;
    }
    while (sum >> 16) sum = (sum & 0xffffU) + (sum >> 16);
    return (uint16_t)(~sum & 0xffffU);
}

static void put_u16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static void send_buffer(NetworkBufferDescriptor_t* buf) {
    while (buf->xDataLength < MIN_FRAME) {
        buf->pucEthernetBuffer[buf->xDataLength++] = 0;
    }
    IPStackEvent_t ev = {.eEventType = eNetworkTxEvent, .pvData = buf};
    if (xSendEventStructToIPTask(&ev, pdMS_TO_TICKS(50)) != pdPASS) {
        vReleaseNetworkBufferAndDescriptor(buf);
    }
}

/* IGMPv2 Membership Report for group_addr (network-byte order). */
static void send_igmp_v2_report(NetworkEndPoint_t* ep, uint32_t group_addr_n) {
    /* eth(14) + ipv4 with router-alert(24) + igmp(8) = 46 */
    const size_t total = 14 + 24 + 8;
    NetworkBufferDescriptor_t* buf =
        pxGetNetworkBufferWithDescriptor(MIN_FRAME, 0);
    if (!buf) {
        LogWarn(("mcast_join: no buffer for IGMP"));
        return;
    }
    memset(buf->pucEthernetBuffer, 0, MIN_FRAME);

    uint8_t* p = buf->pucEthernetBuffer;
    uint32_t group_h = FreeRTOS_ntohl(group_addr_n);

    /* Ethernet: dst 01:00:5e:GG:GG:GG (low 23 bits of group) */
    p[0] = 0x01;
    p[1] = 0x00;
    p[2] = 0x5e;
    p[3] = (uint8_t)((group_h >> 16) & 0x7fU);
    p[4] = (uint8_t)((group_h >> 8) & 0xffU);
    p[5] = (uint8_t)(group_h & 0xffU);
    memcpy(p + 6, ep->xMACAddress.ucBytes, 6);
    put_u16(p + 12, 0x0800U);

    /* IPv4 header (24 bytes incl. router alert) */
    uint8_t* ip = p + 14;
    ip[0] = 0x46;         /* version 4, IHL 6 (24 bytes) */
    ip[1] = 0xc0;         /* DS=internetwork control */
    put_u16(ip + 2, 32U); /* total length: ip(24) + igmp(8) */
    put_u16(ip + 4, 0);   /* identification */
    put_u16(ip + 6, 0);   /* flags + frag offset */
    ip[8] = 1;            /* TTL */
    ip[9] = 2;            /* protocol IGMP */
    put_u16(ip + 10, 0);  /* header checksum (filled below) */
    memcpy(ip + 12, &ep->ipv4_settings.ulIPAddress, 4);
    memcpy(ip + 16, &group_addr_n, 4);
    /* Router Alert option (RFC 2113): type=0x94, len=4, value=0 */
    ip[20] = 0x94;
    ip[21] = 0x04;
    ip[22] = 0;
    ip[23] = 0;

    /* IGMP message */
    uint8_t* igmp = ip + 24;
    igmp[0] = 0x16;       /* IGMPv2 Membership Report */
    igmp[1] = 0;          /* max resp time */
    put_u16(igmp + 2, 0); /* checksum (filled below) */
    memcpy(igmp + 4, &group_addr_n, 4);
    put_u16(igmp + 2, inet_chksum(igmp, 8, 0));

    /* IPv4 header checksum */
    put_u16(ip + 10, inet_chksum(ip, 24, 0));

    buf->xDataLength = total;
    buf->pxEndPoint = ep;
    buf->pxInterface = ep->pxNetworkInterface;
    LogInfo(("mcast_join: IGMPv2 report for %u.%u.%u.%u",
             (unsigned)(group_h >> 24) & 0xff, (unsigned)(group_h >> 16) & 0xff,
             (unsigned)(group_h >> 8) & 0xff, (unsigned)group_h & 0xff));
    send_buffer(buf);
}

/* MLDv1 Multicast Listener Report (ICMPv6 type 131) for the given v6 group. */
static void send_mld_v1_report(NetworkEndPoint_t* ep,
                               const uint8_t group_v6[16]) {
    /* eth(14) + ipv6(40) + hop-by-hop(8) + ICMPv6 MLD(24) = 86 */
    const size_t ip_len = 40, ext_len = 8, mld_len = 24;
    const size_t total = 14 + ip_len + ext_len + mld_len;
    size_t alloc_sz = total < MIN_FRAME ? MIN_FRAME : total;
    NetworkBufferDescriptor_t* buf =
        pxGetNetworkBufferWithDescriptor(alloc_sz, 0);
    if (!buf) {
        LogWarn(("mcast_join: no buffer for MLD"));
        return;
    }
    memset(buf->pucEthernetBuffer, 0, alloc_sz);

    uint8_t* p = buf->pucEthernetBuffer;

    /* Ethernet: dst 33:33:GG:GG:GG:GG (low 32 bits of group) */
    p[0] = 0x33;
    p[1] = 0x33;
    memcpy(p + 2, group_v6 + 12, 4);
    memcpy(p + 6, ep->xMACAddress.ucBytes, 6);
    put_u16(p + 12, 0x86ddU);

    /* IPv6 header */
    uint8_t* ip6 = p + 14;
    ip6[0] = 0x60; /* version 6 */
    ip6[1] = ip6[2] = ip6[3] = 0;
    put_u16(ip6 + 4, (uint16_t)(ext_len + mld_len)); /* payload length */
    ip6[6] = 0; /* next header = Hop-by-Hop options */
    ip6[7] = 1; /* hop limit */
    /* Source: link-local address of endpoint */
    memcpy(ip6 + 8, ep->ipv6_settings.xIPAddress.ucBytes, 16);
    /* Destination: group address */
    memcpy(ip6 + 24, group_v6, 16);

    /* Hop-by-Hop options (8 bytes total): NH=ICMPv6 (58), HdrExtLen=0
     * Options: Router Alert (type=5, len=2, value=0=MLD), PadN(1,0). */
    uint8_t* ext = ip6 + ip_len;
    ext[0] = 58; /* next header: ICMPv6 */
    ext[1] = 0;  /* length in 8-octet units beyond first 8 */
    ext[2] = 0x05;
    ext[3] = 0x02;
    ext[4] = 0x00;
    ext[5] = 0x00; /* Router Alert MLD */
    ext[6] = 0x01;
    ext[7] = 0x00; /* PadN (type 1, len 0) */

    /* ICMPv6 MLD Listener Report (type 131) */
    uint8_t* mld = ext + ext_len;
    mld[0] = 131;        /* type: MLDv1 Listener Report */
    mld[1] = 0;          /* code */
    put_u16(mld + 2, 0); /* checksum (filled below) */
    put_u16(mld + 4, 0); /* max response delay */
    put_u16(mld + 6, 0); /* reserved */
    memcpy(mld + 8, group_v6, 16);

    /* ICMPv6 checksum: pseudo-header (src,dst,upper-layer length,
     * next-header=58) + ICMPv6 message. */
    uint32_t sum = 0;
    for (size_t i = 0; i < 16; i += 2) sum += (ip6[8 + i] << 8) | ip6[9 + i];
    for (size_t i = 0; i < 16; i += 2) sum += (ip6[24 + i] << 8) | ip6[25 + i];
    sum += (uint32_t)mld_len;
    sum += 58U;
    put_u16(mld + 2, inet_chksum(mld, mld_len, sum));

    buf->xDataLength = total;
    buf->pxEndPoint = ep;
    buf->pxInterface = ep->pxNetworkInterface;
    LogInfo(("mcast_join: MLDv1 report for ff02::fb"));
    send_buffer(buf);
}

void mdns_mcast_join_all(void) {
    static const uint32_t mdns_v4_group_n =
        0xfb0000e0U; /* 224.0.0.251 little-endian byte pattern E0.00.00.FB */
    /* On MW300 (ARM little-endian) FreeRTOS_inet_addr returns network-byte-
     * order (big-endian) packed into a uint32_t. 224.0.0.251 → 0xfb0000e0. */
    static const uint8_t mdns_v6_group[16] = {0xff, 0x02, 0, 0, 0, 0, 0, 0,
                                              0,    0,    0, 0, 0, 0, 0, 0xfb};

    for (NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(NULL); ep != NULL;
         ep = FreeRTOS_NextEndPoint(NULL, ep)) {
        if (!ep->bits.bEndPointUp) continue;
        if (!ep->bits.bIPv6) {
            if (ep->ipv4_settings.ulIPAddress != 0U) {
                send_igmp_v2_report(ep, mdns_v4_group_n);
            }
        } else {
            /* Only send MLD on a link-local source — RFC 3590 / RFC 2710. */
            const uint8_t* a = ep->ipv6_settings.xIPAddress.ucBytes;
            if (a[0] == 0xfe && (a[1] & 0xc0) == 0x80) {
                send_mld_v1_report(ep, mdns_v6_group);
            }
        }
    }
}
