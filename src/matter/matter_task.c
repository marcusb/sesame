#include <string.h>

#include "FreeRTOS.h"
#include "app_logging.h"
#undef LOG_INFO
#undef LOG_ERROR
#define LOG_INFO(fmt, ...) LogInfo((fmt, ##__VA_ARGS__))
#define LOG_ERROR(fmt, ...) LogError((fmt, ##__VA_ARGS__))
#include "be_vm.h"
#include "berry.h"
#include "matter_mdns.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

extern void matter_tasmota_tick(bvm* vm);
extern void matter_tasmota_notify_network_up(bvm* vm);

/* Shared between the matter task and (host-test) cmd handlers. */
bvm* g_matter_vm;
SemaphoreHandle_t g_matter_vm_lock;
static StaticSemaphore_t g_matter_vm_lock_buf;

static void matter_task(void* pvParameters) {
    (void)pvParameters;
    LOG_INFO("starting matter task");

    bvm* vm = be_vm_new();
    if (!vm) {
        LOG_ERROR("failed to create berry vm");
        vTaskDelete(NULL);
        return;
    }
    g_matter_vm = vm;

    extern int be_load_crypto_module(bvm * vm);
    be_load_crypto_module(vm);

    /* Initialize registry globals used by the tasmota shim. */
    if (be_dostring(vm,
                    "_matter_fast_cbs = []\n"
                    "_matter_fast_cbs_once = []\n"
                    "_matter_net_cbs = []\n"
                    "_matter_drivers = []\n") != 0) {
        LOG_ERROR("failed to init matter globals: %s", be_tostring(vm, -1));
    }

    if (be_dostring(vm, "import matter") != 0) {
        LOG_ERROR("failed to import matter: %s", be_tostring(vm, -1));
    }

    /* Task is started from the network-up handler, so fire the callbacks now.
     */
    matter_tasmota_notify_network_up(vm);

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        xSemaphoreTakeRecursive(g_matter_vm_lock, portMAX_DELAY);
        matter_tasmota_tick(vm);
        xSemaphoreGiveRecursive(g_matter_vm_lock);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
    }
}

void matter_init(void) {
    matter_mdns_init();
    g_matter_vm_lock =
        xSemaphoreCreateRecursiveMutexStatic(&g_matter_vm_lock_buf);
    xTaskCreate(matter_task, "matter", 8192, NULL, tskIDLE_PRIORITY + 1, NULL);
}
