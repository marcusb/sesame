#include <string.h>

#include "FreeRTOS.h"
#include "app_logging.h"
#undef LOG_INFO
#undef LOG_ERROR
#define LOG_INFO(fmt, ...) LogInfo((fmt, ##__VA_ARGS__))
#define LOG_ERROR(fmt, ...) LogError((fmt, ##__VA_ARGS__))
#include "be_vm.h"
#include "berry.h"
#include "queue.h"
#include "task.h"

static void matter_task(void* pvParameters) {
    (void)pvParameters;
    LOG_INFO("starting matter task");

    bvm* vm = be_vm_new();
    if (!vm) {
        LOG_ERROR("failed to create berry vm");
        vTaskDelete(NULL);
        return;
    }

    /* Load and run matter bootstrap. */
    if (be_dostring(vm, "import matter") != 0) {
        LOG_ERROR("failed to import matter: %s", be_tostring(vm, -1));
    }

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        be_getglobal(vm, "tasmota");
        if (!be_isnil(vm, -1)) {
            be_getmember(vm, -1, "fast_loop");
            if (be_isfunction(vm, -1)) {
                if (be_pcall(vm, 0) != 0) {
                    be_pop(vm, 1);
                }
            }
            be_pop(vm, 1);
        }
        be_pop(vm, 1);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
    }
}

void matter_init(void) {
    xTaskCreate(matter_task, "matter", 8192, NULL, tskIDLE_PRIORITY + 1, NULL);
}
