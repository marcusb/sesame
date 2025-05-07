#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"

// from heap_5.c
typedef struct A_BLOCK_LINK {
    struct A_BLOCK_LINK
        *pxNextFreeBlock; /*<< The next free block in the list. */
    size_t xBlockSize;    /*<< The size of the free block. */
} BlockLink_t;

static const size_t xHeapStructSize =
    (sizeof(BlockLink_t) + ((size_t)(portBYTE_ALIGNMENT - 1))) &
    ~((size_t)portBYTE_ALIGNMENT_MASK);

#define heapBITS_PER_BYTE ((size_t)8)
static size_t xBlockAllocatedBit =
    ((size_t)1) << ((sizeof(size_t) * heapBITS_PER_BYTE) - 1);

// this implementation assumes we are using heap_5
static void *pvPortReAlloc(void *p, size_t size) {
    void *q = NULL;
    if (p == NULL || size > 0) {
        q = pvPortMalloc(size);
        if (p != NULL && q != NULL) {
            BlockLink_t *block =
                (BlockLink_t *)((uint8_t *)p - xHeapStructSize);
            configASSERT((block->xBlockSize & xBlockAllocatedBit) != 0);
            configASSERT(block->pxNextFreeBlock == NULL);
            size_t block_size = block->xBlockSize &= ~xBlockAllocatedBit;
            memcpy(q, p, min(size, block_size));
        }
    }
    if (p != NULL) {
        vPortFree(p);
    }
    return q;
}

void *__wrap_malloc(size_t size) { return pvPortMalloc(size); }

void __wrap_free(void *ptr) { vPortFree(ptr); }

void *__wrap_calloc(size_t nmemb, size_t size) {
    void *ptr = pvPortMalloc(size);
    if (ptr) {
        memset(ptr, 0x00, size);
    }
    return ptr;
}

void *__wrap_realloc(void *ptr, size_t size) {
    return pvPortReAlloc(ptr, size);
}
