#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_logging.h"
#include "harness.h"
#include "task.h"

extern void matter_init(void);

static void start_matter(void) {
    LogInfo(("Starting Matter stack from harness callback"));
    matter_init();
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    harness_init();
    harness_on_network_up(start_matter);
    harness_run();

    return 0;
}
