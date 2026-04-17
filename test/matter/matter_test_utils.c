#include "matter_test_utils.h"

#include <stdio.h>
#include <string.h>

#include "unity.h"

bvm* vm;

extern int be_call_ctype_func(bvm* vm, const void* definition);
extern int be_load_crypto_module(bvm* vm);

void matter_test_setup(void) {
    vm = be_vm_new();
    be_set_ctype_func_handler(vm, be_call_ctype_func);
    be_load_crypto_module(vm);
}

void matter_test_teardown(void) {
    be_vm_delete(vm);
    vm = NULL;
}

void be_assert_success(const char* code) {
    int res = be_dostring(vm, code);
    if (res != 0) {
        const char* err = be_tostring(vm, -1);
        printf("\r\nBERRY ERROR: %s\r\n", err ? err : "(null)");
        be_dumpexcept(vm);
    }
    TEST_ASSERT_EQUAL_INT(0, res);
}
