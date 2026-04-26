#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "FreeRTOS_DNS_Globals.h"
#include "board_support.h"
#include "logging.h"
#include "matter_mdns.h"
#include "task.h"
#include "unity.h"

void setUp(void) { matter_mdns_init(); }

void tearDown(void) {}

static UBaseType_t count_records_by_type(uint16_t type) {
    DNSRecord_t* recs = NULL;
    UBaseType_t n = matter_mdns_snapshot(&recs);
    UBaseType_t count = 0;
    for (UBaseType_t i = 0; i < n; i++) {
        if (recs[i].usRecordType == type) count++;
    }
    return count;
}

static DNSRecord_t* find_record(uint16_t type, const char* name) {
    DNSRecord_t* recs = NULL;
    UBaseType_t n = matter_mdns_snapshot(&recs);
    for (UBaseType_t i = 0; i < n; i++) {
        if (recs[i].usRecordType == type && recs[i].pcName &&
            strcmp(recs[i].pcName, name) == 0) {
            return &recs[i];
        }
    }
    return NULL;
}

void test_add_hostname_registers_both_a_and_aaaa(void) {
    matter_mdns_add_hostname("FOO", NULL, NULL);

    DNSRecord_t* a = find_record(dnsTYPE_A_HOST, "FOO.local");
    DNSRecord_t* aaaa = find_record(dnsTYPE_AAAA_HOST, "FOO.local");

    TEST_ASSERT_NOT_NULL_MESSAGE(a, "expected A record for FOO.local");
    TEST_ASSERT_NOT_NULL_MESSAGE(aaaa, "expected AAAA record for FOO.local");
}

void test_add_hostname_idempotent(void) {
    matter_mdns_add_hostname("BAR", NULL, NULL);
    UBaseType_t a1 = count_records_by_type(dnsTYPE_A_HOST);
    UBaseType_t aaaa1 = count_records_by_type(dnsTYPE_AAAA_HOST);

    matter_mdns_add_hostname("BAR", NULL, NULL);
    UBaseType_t a2 = count_records_by_type(dnsTYPE_A_HOST);
    UBaseType_t aaaa2 = count_records_by_type(dnsTYPE_AAAA_HOST);

    TEST_ASSERT_EQUAL_UINT(a1, a2);
    TEST_ASSERT_EQUAL_UINT(aaaa1, aaaa2);
    TEST_ASSERT_EQUAL_UINT(1, a1);
    TEST_ASSERT_EQUAL_UINT(1, aaaa1);
}

void test_add_service_registers_ptr_srv_txt(void) {
    matter_mdns_add_service("_matterc", "_udp", 5540, "\x07SII=500\x04CM=1",
                            "INST123", "HOST");

    /* Expect: services-PTR, instance-PTR, SRV, TXT */
    DNSRecord_t* services_ptr =
        find_record(dnsTYPE_PTR, "_services._dns-sd._udp.local");
    DNSRecord_t* instance_ptr = find_record(dnsTYPE_PTR, "_matterc._udp.local");
    DNSRecord_t* srv = find_record(dnsTYPE_SRV, "INST123._matterc._udp.local");
    DNSRecord_t* txt = find_record(dnsTYPE_TXT, "INST123._matterc._udp.local");

    TEST_ASSERT_NOT_NULL_MESSAGE(services_ptr, "expected _services PTR");
    TEST_ASSERT_NOT_NULL_MESSAGE(instance_ptr, "expected _matterc PTR");
    TEST_ASSERT_NOT_NULL_MESSAGE(srv, "expected SRV record");
    TEST_ASSERT_NOT_NULL_MESSAGE(txt, "expected TXT record");

    TEST_ASSERT_EQUAL_STRING("_matterc._udp.local",
                             services_ptr->xData.pcPtrRecord);
    TEST_ASSERT_EQUAL_STRING("INST123._matterc._udp.local",
                             instance_ptr->xData.pcPtrRecord);
    TEST_ASSERT_EQUAL_STRING("HOST.local", srv->xData.xSrvRecord.pcTarget);
    TEST_ASSERT_EQUAL_UINT16(5540, srv->xData.xSrvRecord.usPort);
}

void test_add_service_txt_is_pascal_string_format(void) {
    /* Pascal-string DNS-SD format: <len><chunk><len><chunk>... */
    const char* txt_in = "\x07SII=500\x04CM=1";
    matter_mdns_add_service("_matterc", "_udp", 5540, txt_in, "INST", "HOST");

    DNSRecord_t* txt = find_record(dnsTYPE_TXT, "INST._matterc._udp.local");
    TEST_ASSERT_NOT_NULL(txt);
    /* TXT data must round-trip exactly — no extra wrapping. */
    TEST_ASSERT_EQUAL_MEMORY(txt_in, txt->xData.pcTxtRecord, strlen(txt_in));
    TEST_ASSERT_EQUAL_UINT8(0, txt->xData.pcTxtRecord[strlen(txt_in)]);
}

void test_add_subtype_registers_ptr(void) {
    matter_mdns_add_service("_matterc", "_udp", 5540, "", "INST", "HOST");
    matter_mdns_add_subtype("_matterc", "_udp", "INST", "HOST", "_S10");

    DNSRecord_t* sub =
        find_record(dnsTYPE_PTR, "_S10._sub._matterc._udp.local");
    TEST_ASSERT_NOT_NULL_MESSAGE(sub, "expected subtype PTR");
    TEST_ASSERT_EQUAL_STRING("INST._matterc._udp.local",
                             sub->xData.pcPtrRecord);
}

void test_remove_service_removes_records(void) {
    matter_mdns_add_service("_matterc", "_udp", 5540, "", "INST", "HOST");

    UBaseType_t before = count_records_by_type(dnsTYPE_SRV);
    TEST_ASSERT_GREATER_THAN_UINT(0, before);

    matter_mdns_remove_service("_matterc", "_udp", "INST", "HOST");

    TEST_ASSERT_NULL(find_record(dnsTYPE_SRV, "INST._matterc._udp.local"));
    TEST_ASSERT_NULL(find_record(dnsTYPE_TXT, "INST._matterc._udp.local"));
}

void run_tests(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_add_hostname_registers_both_a_and_aaaa);
    RUN_TEST(test_add_hostname_idempotent);
    RUN_TEST(test_add_service_registers_ptr_srv_txt);
    RUN_TEST(test_add_service_txt_is_pascal_string_format);
    RUN_TEST(test_add_subtype_registers_ptr);
    RUN_TEST(test_remove_service_removes_records);
}
