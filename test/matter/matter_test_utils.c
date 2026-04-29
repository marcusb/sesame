#include "matter_test_utils.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board_support.h"
#include "semphr.h"
#include "unity.h"

extern bvm* g_matter_vm;
extern SemaphoreHandle_t g_matter_vm_lock;

extern int be_call_ctype_func(bvm* vm, const void* definition);
extern int be_load_crypto_module(bvm* vm);

void matter_test_setup(void) {
    g_matter_vm = be_vm_new();
    if (!g_matter_vm) {
        printf("FAILED to create Berry VM (RAM starvation?)\n");
        return;
    }
    be_set_ctype_func_handler(g_matter_vm, be_call_ctype_func);
    be_load_crypto_module(g_matter_vm);

    if (!g_matter_vm_lock) {
        static StaticSemaphore_t lock_buf;
        g_matter_vm_lock = xSemaphoreCreateRecursiveMutexStatic(&lock_buf);
    }
}

void matter_test_teardown(void) {
    be_vm_delete(g_matter_vm);
    g_matter_vm = NULL;
}

void be_assert_success(const char* code) {
    int res = be_dostring(g_matter_vm, code);
    if (res != 0) {
        const char* err = be_tostring(g_matter_vm, -1);
        printf("\r\nBERRY ERROR: %s\r\n", err ? err : "(null)");
        be_dumpexcept(g_matter_vm);
    }
    TEST_ASSERT_EQUAL_INT(0, res);
}
