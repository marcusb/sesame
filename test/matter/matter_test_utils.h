#pragma once
#include "berry.h"

extern bvm* vm;

/* Initialize VM and register common handlers (like ctype_func) */
void matter_test_setup(void);

/* Clean up VM */
void matter_test_teardown(void);

/* Execute Berry code and assert success (res == 0) */
void be_assert_success(const char* code);
