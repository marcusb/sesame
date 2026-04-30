#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "FreeRTOS_DNS_Globals.h"
#include "FreeRTOS_DNS_Parser.h"
#include "FreeRTOS_IP.h"
#include "matter_mdns.h"
#include "matter_test_utils.h"
#include "unity.h"

/* Forward declare to satisfy compiler if needed */
struct freertos_addrinfo;

static NetworkEndPoint_t xEndPoint;
static bool bHeapInit = false;

void setUp(void) {
    if (!bHeapInit) {
        xNetworkBuffersInitialise();
        bHeapInit = true;
    }
    matter_test_setup();
    matter_mdns_init();

    memset(&xEndPoint, 0, sizeof(xEndPoint));
    xEndPoint.ipv4_settings.ulIPAddress = FreeRTOS_inet_addr("10.0.2.15");
    xEndPoint.bits.bIPv6 = pdFALSE;
}

void tearDown(void) { matter_test_teardown(); }

/* Mock network transmission to avoid asserting in vReturnEthernetFrame */
void __wrap_vReturnEthernetFrame(NetworkBufferDescriptor_t* pxNetworkBuffer,
                                 BaseType_t xReleaseAfterSend) {
    (void)pxNetworkBuffer;
    (void)xReleaseAfterSend;
    /* Do nothing, successfully "transmitted" */
}

/* Helper to execute a query using a real network buffer */
static uint32_t execute_query(const uint8_t* query, size_t query_len) {
    NetworkBufferDescriptor_t* pxNetBuf =
        pxGetNetworkBufferWithDescriptor(1500, 0);
    TEST_ASSERT_NOT_NULL(pxNetBuf);

    pxNetBuf->pxEndPoint = &xEndPoint;
    uint8_t* ucEthBuffer = pxNetBuf->pucEthernetBuffer;
    memset(ucEthBuffer, 0, 1500);

    /* Initialize Headers to avoid assertion failures */
    ucEthBuffer[-6] =
        0x40; /* IPv4 type tag used by pxUDPPayloadBuffer_to_NetworkBuffer */

    EthernetHeader_t* pxEthHeader = (EthernetHeader_t*)ucEthBuffer;
    pxEthHeader->usFrameType = ipIPv4_FRAME_TYPE;

    IPHeader_t* pxIPHeader = (IPHeader_t*)(ucEthBuffer + ipSIZE_OF_ETH_HEADER);
    pxIPHeader->ucVersionHeaderLength = 0x45; /* IPv4, 5 words */
    pxIPHeader->ulSourceIPAddress = FreeRTOS_inet_addr("10.0.2.2");
    pxIPHeader->ulDestinationIPAddress = FreeRTOS_inet_addr("224.0.0.251");
    pxIPHeader->ucProtocol = ipPROTOCOL_UDP;

    UDPHeader_t* pxUDPHeader =
        (UDPHeader_t*)(ucEthBuffer + ipSIZE_OF_ETH_HEADER +
                       ipSIZE_OF_IPv4_HEADER);
    pxUDPHeader->usSourcePort = FreeRTOS_htons(5353);
    pxUDPHeader->usDestinationPort = FreeRTOS_htons(5353);

    /* Payload goes after UDP header */
    uint8_t* payload = ucEthBuffer + ipSIZE_OF_ETH_HEADER +
                       ipSIZE_OF_IPv4_HEADER + ipSIZE_OF_UDP_HEADER;
    memcpy(payload, query, query_len);
    pxNetBuf->xDataLength = ipSIZE_OF_ETH_HEADER + ipSIZE_OF_IPv4_HEADER +
                            ipSIZE_OF_UDP_HEADER + query_len;

    uint32_t result =
        DNS_ParseDNSReply(payload, query_len, NULL, pdFALSE, 5353);

    /* Since DNS_ParseDNSReply might NOT free the original buffer if it didn't
     * duplicate, wait, the responder typically duplicates. But if it didn't, we
     * should free it. Actually, let's just free it. Wait, the real stack
     * manages it. DNS_ParseDNSReply does NOT free the incoming buffer. It only
     * frees the duplicated one. */
    vReleaseNetworkBufferAndDescriptor(pxNetBuf);
    return result;
}

void test_responder_matches_matterc_query(void) {
    /* Register a service first */
    matter_mdns_add_service("_matterc", "_udp", 5540, "\x07SII=500", "INST123",
                            "HOST");

    /* Construct a raw mDNS query for _matterc._udp.local
     * Transaction ID: 0x1234
     * Flags: 0x0000 (Standard query)
     * Questions: 1
     * Name: \x08 _matterc \x04 _udp \x05 local \x00
     * Type: PTR (0x000C)
     * Class: IN (0x0001)
     */
    uint8_t query[] = {
        0x12, 0x34, /* ID */
        0x00, 0x00, /* Flags */
        0x00, 0x01, /* Questions */
        0x00, 0x00, /* Answers */
        0x00, 0x00, /* Authority */
        0x00, 0x00, /* Additional */
        0x08, '_',  'm',  'a', 't', 't', 'e', 'r', 'c',  0x04, '_',  'u',
        'd',  'p',  0x05, 'l', 'o', 'c', 'a', 'l', 0x00, 0x00, 0x0C, /* Type PTR
                                                                      */
        0x00, 0x01 /* Class IN */
    };

    uint32_t result = execute_query(query, sizeof(query));
    TEST_ASSERT_EQUAL(
        0, result); /* It returns 0 for incoming queries it handles */
}

void test_responder_matches_matterc_query_with_unicast_bit(void) {
    matter_mdns_add_service("_matterc", "_udp", 5540, "\x07SII=500", "INST123",
                            "HOST");

    /* mDNS Class 0x8001 = IN + Unicast Response bit */
    uint8_t query[] = {
        0x12, 0x34, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x08, '_',  'm',  'a',  't',  't',  'e',  'r',
        'c',  0x04, '_',  'u',  'd',  'p',  0x05, 'l',  'o',  'c',
        'a',  'l',  0x00, 0x00, 0x0C, 0x80, 0x01 /* Class IN, QU bit set */
    };

    execute_query(query, sizeof(query));
}

void test_responder_matches_srv_query(void) {
    matter_mdns_add_service("_matterc", "_udp", 5540, "\x07SII=500", "INST123",
                            "HOST");

    /* Query for INST123._matterc._udp.local SRV */
    uint8_t query[] = {
        0x12, 0x34, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x07, 'I',  'N',  'S',  'T',  '1',  '2',  '3',  0x08, '_',
        'm',  'a',  't',  't',  'e',  'r',  'c',  0x04, '_',  'u',  'd',
        'p',  0x05, 'l',  'o',  'c',  'a',  'l',  0x00, 0x00, 0x21, /* Type SRV
                                                                       (33) */
        0x00, 0x01 /* Class IN */
    };

    uint32_t result = execute_query(query, sizeof(query));
    TEST_ASSERT_EQUAL(0, result);
}

void test_responder_matches_txt_query(void) {
    matter_mdns_add_service("_matterc", "_udp", 5540, "\x07SII=500", "INST123",
                            "HOST");

    /* Query for INST123._matterc._udp.local TXT */
    uint8_t query[] = {
        0x12, 0x34, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x07, 'I',  'N',  'S',  'T',  '1',  '2',  '3',  0x08, '_',
        'm',  'a',  't',  't',  'e',  'r',  'c',  0x04, '_',  'u',  'd',
        'p',  0x05, 'l',  'o',  'c',  'a',  'l',  0x00, 0x00, 0x10, /* Type TXT
                                                                       (16) */
        0x00, 0x01 /* Class IN */
    };

    uint32_t result = execute_query(query, sizeof(query));
    TEST_ASSERT_EQUAL(0, result);
}

void test_responder_matches_a_query(void) {
    matter_mdns_add_hostname("HOST", NULL, NULL);

    /* Query for HOST.local A */
    uint8_t query[] = {
        0x12, 0x34, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x04, 'H',  'O',  'S',  'T',  0x05,
        'l',  'o',  'c',  'a',  'l',  0x00, 0x00, 0x01, /* Type A (1) */
        0x00, 0x01                                      /* Class IN */
    };

    uint32_t result = execute_query(query, sizeof(query));
    TEST_ASSERT_EQUAL(0, result);
}

void test_responder_includes_additional_records(void) {
    matter_mdns_add_service("_matterc", "_udp", 5540, "\x07SII=500", "INST123",
                            "HOST");
    matter_mdns_add_hostname("HOST", NULL, NULL);

    /* Query for _matterc._udp.local PTR */
    uint8_t query[] = {0x12, 0x34, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x08, '_',  'm',  'a',
                       't',  't',  'e',  'r',  'c',  0x04, '_',  'u',
                       'd',  'p',  0x05, 'l',  'o',  'c',  'a',  'l',
                       0x00, 0x00, 0x0C, 0x00, 0x01};

    /* This will call xApplicationDNSRecordQueryHook_Multi and mark the PTR
     * record as answer */
    execute_query(query, sizeof(query));

    /* Manually call the hook */
    extern void xApplicationDNSRecordsMatchedHook(void);
    xApplicationDNSRecordsMatchedHook();

    /* Verify records in s_view (we use matter_mdns_get_view so we don't reset
     * them) */
    DNSRecord_t* recs = NULL;
    UBaseType_t n = matter_mdns_get_view(&recs);

    int answers = 0;
    int additionals = 0;
    for (UBaseType_t i = 0; i < n; i++) {
        if (recs[i].uxServeRecord == 3) answers++;
        if (recs[i].uxServeRecord == 1) additionals++;
    }

    /* Expect 1 answer (PTR) and at least SRV, TXT, A, AAAA as additionals */
    TEST_ASSERT_EQUAL_INT(1, answers);
    TEST_ASSERT_GREATER_OR_EQUAL_INT(4, additionals);
}

void run_tests(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_responder_matches_matterc_query);
    RUN_TEST(test_responder_matches_matterc_query_with_unicast_bit);
    RUN_TEST(test_responder_matches_srv_query);
    RUN_TEST(test_responder_matches_txt_query);
    RUN_TEST(test_responder_matches_a_query);
    RUN_TEST(test_responder_includes_additional_records);
}
