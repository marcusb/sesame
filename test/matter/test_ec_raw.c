#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "board_support.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/ecp.h"
#include "mbedtls/entropy.h"
#include "task.h"
#include "unity.h"

void test_mbedtls_ec_p256_mul_raw(void) {
    mbedtls_ecp_group grp;
    mbedtls_ecp_point Q;
    mbedtls_mpi d;

    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_point_init(&Q);
    mbedtls_mpi_init(&d);

    int ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1);
    TEST_ASSERT_EQUAL(0, ret);

    unsigned char scalar_buf[32] = {0x02}; /* Start with small scalar */
    mbedtls_mpi_read_binary(&d, scalar_buf, sizeof(scalar_buf));

    ret = mbedtls_ecp_mul(&grp, &Q, &d, &grp.G, mbedtls_ctr_drbg_random,
                          &ctr_drbg);
    TEST_ASSERT_EQUAL(0, ret);

    /* Now try full 256-bit scalar */
    for (int i = 0; i < 32; i++) scalar_buf[i] = (unsigned char)i;
    mbedtls_mpi_read_binary(&d, scalar_buf, sizeof(scalar_buf));

    ret = mbedtls_ecp_mul(&grp, &Q, &d, &grp.G, mbedtls_ctr_drbg_random,
                          &ctr_drbg);
    TEST_ASSERT_EQUAL(0, ret);

    mbedtls_ecp_group_free(&grp);
    mbedtls_ecp_point_free(&Q);
    mbedtls_mpi_free(&d);
}

void run_tests(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_mbedtls_ec_p256_mul_raw);
}
