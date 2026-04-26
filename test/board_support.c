#include "board_support.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board.h"
#include "clock_config.h"
#include "ksdk_mbedtls.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "pin_mux.h"
#include "queue.h"
#include "task.h"

/* Global symbols referenced by modules under test */
QueueHandle_t ctrl_queue;
void* psm_hnd = NULL;
mbedtls_ctr_drbg_context ctr_drbg;

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

/* ---- FreeRTOS application hooks ---- */

__attribute__((weak)) void vApplicationIdleHook(void) {}

__attribute__((weak)) void vApplicationTickHook(void) {}

void vApplicationMallocFailedHook(void) {
    printf("MALLOC FAILED!\n");
    configASSERT(0);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    printf("STACK OVERFLOW in task %s!\n", pcTaskName);
    (void)xTask;
    (void)pcTaskName;
    configASSERT(0);
}

/* ---- newlib wrappers ---- */
void* __wrap_sbrk(int incr) {
    (void)incr;
    return (void*)-1;
}

/* ---- Board initialization ---- */

static void init_mbedtls_random(void) {
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
        configASSERT(0);
    }
}

void board_init(void) {
    board_init_pins();
    init_boot_clocks();

    /* Enable line buffering on stdout to speed up semihosting output */
    setvbuf(stdout, NULL, _IOLBF, 256);

    setup_heap();
    init_mbedtls_random();
}
