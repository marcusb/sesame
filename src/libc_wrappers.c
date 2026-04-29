#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "debug_console.h"
#ifdef USE_BACKTRACE
#include "backtrace.h"
#endif

// from heap_5.c
typedef struct A_BLOCK_LINK {
    struct A_BLOCK_LINK*
        pxNextFreeBlock; /*<< The next free block in the list. */
    size_t xBlockSize;   /*<< The size of the free block. */
} BlockLink_t;

static const size_t xHeapStructSize =
    (sizeof(BlockLink_t) + ((size_t)(portBYTE_ALIGNMENT - 1))) &
    ~((size_t)portBYTE_ALIGNMENT_MASK);

#define heapBITS_PER_BYTE ((size_t)8)
static size_t xBlockAllocatedBit =
    ((size_t)1) << ((sizeof(size_t) * heapBITS_PER_BYTE) - 1);

// this implementation assumes we are using heap_5
static void* pvPortReAlloc(void* p, size_t size) {
    void* q = NULL;
    if (p == NULL || size > 0) {
        q = pvPortMalloc(size);
        if (p != NULL && q != NULL) {
            BlockLink_t* block = (BlockLink_t*)((uint8_t*)p - xHeapStructSize);
            configASSERT((block->xBlockSize & xBlockAllocatedBit) != 0);
            configASSERT(block->pxNextFreeBlock == NULL);
            /* Read the allocated size without modifying the metadata —
             * vPortFree() below re-checks the allocated bit and asserts if
             * it has been cleared. */
            size_t block_size = block->xBlockSize & ~xBlockAllocatedBit;
            memcpy(q, p, min(size, block_size));
        }
    }
    if (p != NULL) {
        vPortFree(p);
    }
    return q;
}

void* __wrap_malloc(size_t size) { return pvPortMalloc(size); }

void __wrap_free(void* ptr) { vPortFree(ptr); }

void* __wrap_calloc(size_t nmemb, size_t size) {
    size_t total = nmemb * size;
    void* ptr = pvPortMalloc(total);
    if (ptr) {
        memset(ptr, 0x00, total);
    }
    return ptr;
}

void* __wrap_realloc(void* ptr, size_t size) {
    return pvPortReAlloc(ptr, size);
}

#if SDK_DEBUGCONSOLE || defined(QEMU)
int __wrap_printf(const char* format, ...) {
    va_list ap;
    va_start(ap, format);
#if defined(QEMU)
    extern int __real_vprintf(const char* format, va_list ap);
    int ret = vprintf(format, ap);
#else
    char buf[256];
    int ret = vsnprintf(buf, sizeof(buf), format, ap);
    DbgConsole_Printf("%s", buf);
#endif
    va_end(ap);
    return ret;
}

int __wrap_fprintf(FILE* stream, const char* format, ...) {
    va_list ap;
    va_start(ap, format);
#if defined(QEMU)
    int ret = vfprintf(stream, format, ap);
#else
    (void)stream;
    char buf[256];
    int ret = vsnprintf(buf, sizeof(buf), format, ap);
    DbgConsole_Printf("%s", buf);
#endif
    va_end(ap);
    return ret;
}

__attribute__((noreturn)) void _exit(int status) {
    (void)status;
    portDISABLE_INTERRUPTS();
    for (;;) {
#if defined(QEMU)
        __asm__ volatile("bkpt #0");
#else
        __BKPT(0);
#endif
    }
}

__attribute__((noreturn)) void __assert_func(const char* file, int line,
                                             const char* func,
                                             const char* failedexpr) {
    PRINTF("\r\n***ASSERT ERROR: %s in function %s %s:%d\r\n", failedexpr, func,
           file, line);
#ifdef USE_BACKTRACE
    static backtrace_t bt[20];
    register uint32_t pc, sp;
    __asm__ volatile("mov %0, pc" : "=r"(pc));
    __asm__ volatile("mov %0, sp" : "=r"(sp));
    backtrace_frame_t frame;
    frame.sp = sp;
    frame.fp = (uint32_t)__builtin_frame_address(0);
    frame.lr = (uint32_t)__builtin_return_address(0);
    frame.pc = pc;
    int count = _backtrace_unwind(bt, 20, &frame);
    for (int i = 0; i < count; ++i) {
        PRINTF("%s@%p\r\n", bt[i].name, bt[i].address);
    }
#endif
    portDISABLE_INTERRUPTS();
    for (;;) {
#if defined(QEMU)
        __asm__ volatile("bkpt #0");
#else
        __BKPT(0);
#endif
    }
}
#endif
