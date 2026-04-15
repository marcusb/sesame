#include <stdio.h>
#include <string.h>

#include "app_logging.h"
#include "berry.h"
#include "fsl_debug_console.h"
#include "unity.h"

extern bclass_array be_class_table;

static bvm* vm;

void test_matter_setup(void) { vm = be_vm_new(); }

void test_matter_teardown(void) { be_vm_delete(vm); }

void test_matter_crypto_random(void) {
    int res = be_dostring(
        vm, "import crypto; var b = crypto.random(16); assert(b.size() == 16)");
    TEST_ASSERT_EQUAL_INT(0, res);
}

void test_matter_crypto_sha256(void) {
    int res = be_dostring(
        vm,
        "import crypto; "
        "var b = crypto.SHA256(bytes().fromstring('hello')); "
        "print('SHA256(hello) =', b.tohex()); "
        "assert(b.tohex() == "
        "'2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824')");
    if (res != 0) {
        const char* err = be_tostring(vm, -1);
        PRINTF("\r\nBERRY ERROR: %s\r\n", err ? err : "(null)");
        be_dumpexcept(vm);
    }
    TEST_ASSERT_EQUAL_INT(0, res);
}

void test_matter_module_import(void) {
    int res = be_dostring(vm, "import matter");
    TEST_ASSERT_EQUAL_INT(0, res);
}

void run_test_matter(void) {
    test_matter_setup();
    RUN_TEST(test_matter_crypto_sha256);
    RUN_TEST(test_matter_module_import);
    test_matter_teardown();
}
