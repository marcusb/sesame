#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/ecp.h"
#include "mbedtls/entropy.h"
#include "task.h"
#include "unity.h"

void setUp(void) {}
void tearDown(void) {}

void test_mbedtls_ec_p256_mul_raw(void) {
    printf("\n--- Starting RAW mbedTLS EC P256 mul baseline test ---\n");
    printf("Free heap: %d\n", (int)xPortGetFreeHeapSize());

    mbedtls_ecp_group grp;
    mbedtls_ecp_point Q;
    mbedtls_mpi d;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;

    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_point_init(&Q);
    mbedtls_mpi_init(&d);
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&ctr_drbg);

    /* Valid RNG is mandatory in mbedTLS 3.x */
    int ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                    NULL, 0);
    TEST_ASSERT_EQUAL(0, ret);

    ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1);
    TEST_ASSERT_EQUAL(0, ret);

    unsigned char scalar_buf[32] = {0x02}; /* Start with small scalar */
    mbedtls_mpi_read_binary(&d, scalar_buf, sizeof(scalar_buf));

    printf("Calling mbedtls_ecp_mul (G*2)...\n");
    TickType_t start = xTaskGetTickCount();
    ret = mbedtls_ecp_mul(&grp, &Q, &d, &grp.G, mbedtls_ctr_drbg_random,
                          &ctr_drbg);
    TickType_t end = xTaskGetTickCount();
    printf("mbedtls_ecp_mul returned %d, took %d ms\n", ret,
           (int)((end - start) * portTICK_PERIOD_MS));
    TEST_ASSERT_EQUAL(0, ret);

    /* Now try full 256-bit scalar */
    printf("Calling mbedtls_ecp_mul (full 256-bit)...\n");
    for (int i = 0; i < 32; i++) scalar_buf[i] = (unsigned char)i;
    mbedtls_mpi_read_binary(&d, scalar_buf, sizeof(scalar_buf));

    start = xTaskGetTickCount();
    ret = mbedtls_ecp_mul(&grp, &Q, &d, &grp.G, mbedtls_ctr_drbg_random,
                          &ctr_drbg);
    end = xTaskGetTickCount();
    printf("mbedtls_ecp_mul (full) returned %d, took %d ms\n", ret,
           (int)((end - start) * portTICK_PERIOD_MS));
    TEST_ASSERT_EQUAL(0, ret);

    mbedtls_ecp_group_free(&grp);
    mbedtls_ecp_point_free(&Q);
    mbedtls_mpi_free(&d);
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);

    printf("--- EC RAW mul test PASSED ---\n");
}

extern void test_board_init(void);

static void run_tests_task(void* params) {
    (void)params;
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();
    RUN_TEST(test_mbedtls_ec_p256_mul_raw);
    int result = UNITY_END();
    printf("\r\nTEST_RESULT:%d\r\n", result);
    vTaskDelete(NULL);
}

int main(void) {
    test_board_init();
    xTaskCreate(run_tests_task, "tests", 8192, NULL, tskIDLE_PRIORITY + 1,
                NULL);
    vTaskStartScheduler();
    return 0;
}
