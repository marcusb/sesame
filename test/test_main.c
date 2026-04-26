#include "unity.h"

/* Forward declarations of test suites */
void run_tests_string_util(void);
void run_tests_util(void);
void run_tests_logging(void);
void run_tests_config_manager(void);
void run_tests_pic_uart(void);

void run_tests(void) {
    run_tests_string_util();
    run_tests_util();
    run_tests_logging();
    run_tests_config_manager();
    run_tests_pic_uart();
}
