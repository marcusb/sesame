#include "board_support.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board.h"
#include "clock_config.h"
#include "ksdk_mbedtls.h"
#include "mbedtls/entropy.h"
#include "pin_mux.h"
#include "queue.h"
#include "task.h"

/* Global symbols referenced by modules under test */
QueueHandle_t ctrl_queue;
void* psm_hnd = NULL;

/* ---- Heap setup (mirrors main.c) ---- */

extern unsigned __HeapBase, __HeapLimit, __HeapBase_sram0, __HeapLimit_sram0;

void setup_heap(void) {
    HeapRegion_t xHeapRegions[3];
    size_t i = 0;

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
    exit(1);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    printf("STACK OVERFLOW in task %s!\n", pcTaskName);
    exit(1);
}

void* __wrap_sbrk(int incr) {
    (void)incr;
    return (void*)-1;
}

void board_init(void) {
    /* Minimal init for QEMU debugging */
    setvbuf(stdout, NULL, _IOLBF, 256);

    /* Initialize dummy entropy for tests */
    uint8_t dummy_hash[32] = {0x42};
    mbedtls_hardware_init_hash(dummy_hash, sizeof(dummy_hash));

    setup_heap();
}
