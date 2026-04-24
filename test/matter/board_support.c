#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* mw320 board */
#include "be_mapping.h"
#include "board.h"
#include "board_support.h"
#include "clock_config.h"
#include "entropy_poll.h"
#include "fsl_debug_console.h"
#include "ksdk_mbedtls.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "pin_mux.h"

/* Unity */
#include "unity.h"

/* Global symbols referenced by modules under test */
QueueHandle_t ctrl_queue;
void* psm_hnd = (void*)0x12345678;
mbedtls_ctr_drbg_context ctr_drbg;

/* ---- Unity output ---- */
void unity_putchar(char c) {
    if (c == '\n') {
        PUTCHAR('\r');
    }
    PUTCHAR(c);
}

void unity_flush(void) { DbgConsole_Flush(); }

/* ---- FreeRTOS application hooks ---- */
void vApplicationIdleHook(void) {}
void vApplicationMallocFailedHook(void) {
    printf("MALLOC FAILED!\n");
    for (;;);
}
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    printf("STACK OVERFLOW in task %s!\n", pcTaskName);
    (void)xTask;
    (void)pcTaskName;
    for (;;);
}
void vApplicationTickHook(void) {}

/* ---- newlib wrappers ---- */
void* __wrap_sbrk(int incr) {
    (void)incr;
    return (void*)-1;
}

/* ---- Heap setup (mirrors main.c) ---- */
extern unsigned __HeapBase, __HeapLimit, __HeapBase_sram0, __HeapLimit_sram0;

void setup_heap(void) {
    HeapRegion_t xHeapRegions[3];
    size_t i = 0;

    /* Prioritize larger SRAM1 region for better contiguous allocation */
    if ((unsigned)&__HeapLimit > (unsigned)&__HeapBase) {
        xHeapRegions[i++] =
            (HeapRegion_t){(uint8_t*)&__HeapBase,
                           (unsigned)&__HeapLimit - (unsigned)&__HeapBase};
    }

    if ((unsigned)&__HeapLimit_sram0 > (unsigned)&__HeapBase_sram0) {
        xHeapRegions[i++] = (HeapRegion_t){
            (uint8_t*)&__HeapBase_sram0,
            (unsigned)&__HeapLimit_sram0 - (unsigned)&__HeapBase_sram0};
    }

    xHeapRegions[i] = (HeapRegion_t){NULL, 0};
    vPortDefineHeapRegions(xHeapRegions);
}

void test_board_init(void) {
    board_init_pins();
    init_boot_clocks();
    init_debug_console();
    setup_heap();

    /* Initialize KSDK mbedTLS entropy with a dummy hash */
    uint8_t dummy_hash[32];
    for (int i = 0; i < 32; i++) dummy_hash[i] = (uint8_t)i;
    mbedtls_hardware_init_hash(dummy_hash, 32);

    static mbedtls_entropy_context entropy;
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&ctr_drbg);
    int ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                    NULL, 0);
    if (ret != 0) {
        printf("mbedtls_ctr_drbg_seed failed: %d\n", ret);
        for (;;);
    }
}

/* ---- PSM Stub ---- */
static uint8_t psm_buf[1024];
static size_t psm_len = 0;
static int psm_error = 0;

void stub_psm_set_data(const uint8_t* data, size_t len) {
    if (data == NULL) {
        psm_len = 0;
    } else {
        if (len > sizeof(psm_buf)) len = sizeof(psm_buf);
        memcpy(psm_buf, data, len);
        psm_len = len;
    }
    psm_error = 0;
}

void stub_psm_get_written(const uint8_t** out, size_t* out_len) {
    *out = psm_buf;
    *out_len = psm_len;
}

void stub_psm_set_error(int err) {
    psm_error = err;
    if (err) psm_len = 0;
}

int psm_get_variable(void* phandle, const char* variable, void* value,
                     uint32_t max_len) {
    (void)phandle;
    (void)variable;
    if (psm_error) return psm_error;
    if (psm_len == 0) return -1;
    uint32_t copy = (psm_len < max_len) ? (uint32_t)psm_len : max_len;
    memcpy(value, psm_buf, copy);
    return (int)copy;
}

int psm_set_variable(void* phandle, const char* variable, const void* value,
                     uint32_t len) {
    (void)phandle;
    (void)variable;
    if (psm_error) return psm_error;
    if (len > sizeof(psm_buf)) len = sizeof(psm_buf);
    memcpy(psm_buf, value, len);
    psm_len = len;
    return 0;
}

int psm_object_delete(void* phandle, const char* variable) {
    (void)phandle;
    (void)variable;
    psm_len = 0;
    return 0;
}

/* ---- OS stubs for wlcmgr ---- */
typedef void* os_rw_lock_t;
int os_rwlock_create(os_rw_lock_t* lock, const char* mutex_name,
                     const char* lock_name) {
    return 0;
}
int os_rwlock_read_lock(os_rw_lock_t* lock, unsigned int wait_time) {
    return 0;
}
int os_rwlock_read_unlock(os_rw_lock_t* lock) { return 0; }
int os_rwlock_write_lock(os_rw_lock_t* lock, unsigned int wait_time) {
    return 0;
}
void os_rwlock_write_unlock(os_rw_lock_t* lock) {}
void os_rwlock_delete(os_rw_lock_t* lock) {}
