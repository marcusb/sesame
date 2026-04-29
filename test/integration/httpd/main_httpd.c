#include <stdlib.h>

#include "FreeRTOS.h"
#include "harness.h"
#include "task.h"

extern void httpd_task(void* params);

static void start_sut(void) {
    xTaskCreate(httpd_task, "HTTPD", 4096, NULL, tskIDLE_PRIORITY + 2, NULL);
}

int main(void) {
    setup_heap();
    harness_init();
    harness_on_network_up(start_sut);
    harness_run();
    return 0;
}
