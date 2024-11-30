#include "backtrace.h"
#include "critical_error.h"
#include "wmstdio.h"

#define BACKTRACE_DEPTH 20

static backtrace_t backtrace[BACKTRACE_DEPTH];

__attribute__((used)) void critical_error(int crit_errno, void *data) {
    ll_printf("Critical error number: %d (caller addr: %x)\r\n", crit_errno,
              __builtin_return_address(0));
    ll_printf("Description: %s\r\n", critical_error_msg(crit_errno));

    int count = backtrace_unwind(backtrace, BACKTRACE_DEPTH);
    ll_printf("backtrace ret %d\r\n", count);
    for (int i = 0; i < count; ++i) {
        ll_printf("%p - %s@%p\r\n", backtrace[i].address, backtrace[i].name,
                  backtrace[i].address);
    }
}
