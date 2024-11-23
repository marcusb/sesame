#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "NetworkInterface.h"
#include "mw300_rd_net.h"
#include "task.h"

// wmsdk
#include <board.h>

#include "boot_flags.h"
#include "cli.h"
#include "flash.h"
#include "iot_crypto.h"
#include "iot_logging_task.h"
#include "iot_wifi.h"
#include "lowlevel_drivers.h"
#include "mbedtls/entropy.h"
#include "mdev_gpio.h"
#include "mdev_pinmux.h"
#include "mdev_rtc.h"
#include "partition.h"
#include "psm-v2.h"
#include "pwrmgr.h"
#include "queue.h"
#include "wifi.h"
#include "wmstdio.h"
#include "wmtime.h"

// Application
#include "app_logging.h"
#include "controller.h"
#include "httpd.h"
#include "leds.h"
#include "mqtt.h"
#include "network.h"
#include "ota.h"
#include "pic_uart.h"

#define BTN_WIFI GPIO_22
#define BTN_OTA GPIO_23

/* Logging Task Defines. */
#define mainLOGGING_MESSAGE_QUEUE_LENGTH (64)

psm_hnd_t psm_hnd;
flash_desc_t fl;
static QueueHandle_t ctrl_queue;
static QueueHandle_t ota_queue;
static QueueHandle_t pic_queue;
static QueueHandle_t nm_queue;

// IP configuration to use when DHCP does not succeed
static uint8_t mac_addr[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
static const uint8_t default_ip_addr[4] = {192, 168, 4, 1};
static const uint8_t netmask[4] = {255, 255, 255, 0};
static const uint8_t gateway_addr[4] = {0};
static const uint8_t dns_server_addr[4] = {0};

static const char HOSTNAME[] = "sesame";

void wm_printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    wmprintf(format, args);
    va_end(args);
}

static void gpio_cb(int pin, void *data) {
    if (pin == BTN_WIFI) {
        ctrl_msg_t msg = {CTRL_MSG_WIFI_BUTTON};
        xQueueSendToBackFromISR(ctrl_queue, &msg, NULL);
    } else if (pin == BTN_OTA) {
        ctrl_msg_t msg = {CTRL_MSG_OTA_BUTTON};
        xQueueSendToBackFromISR(ctrl_queue, &msg, NULL);
    }
}

static void init_gpio_input(mdev_t *gpio_dev, int pin) {
    gpio_drv_setdir(gpio_dev, pin, GPIO_INPUT);
    int res =
        gpio_drv_set_cb(gpio_dev, pin, GPIO_INT_FALLING_EDGE, NULL, gpio_cb);
    if (res != WM_SUCCESS) {
        LogError(("failed to regiser GPIO ISR"));
    }
}

static void init_gpio() {
    gpio_drv_init();
    PMU_VbatBrndetConfig_Type vbat_brndet_cfg = {PMU_VBAT_BRNDET_TRIGVOLT_7,
                                                 PMU_VBAT_BRNDET_HYST_3,
                                                 PMU_VBAT_BRNDET_FILT_0};
    PMU_ConfigVbatBrndet(&vbat_brndet_cfg);

    mdev_t *gpio_dev = gpio_drv_open("MDEV_GPIO");
    if (gpio_dev == NULL) {
        configPRINT("gpio_drv_open failed\r\n");
        return;
    }

    pinmux_drv_init();
    mdev_t *pinmux_dev = pinmux_drv_open("MDEV_PINMUX");
    if (pinmux_dev == NULL) {
        configPRINT("pinux_drv_open failed\r\n");
        return;
    }

    pinmux_drv_setfunc(pinmux_dev, GPIO_0, GPIO0_GPIO0);
    gpio_drv_setdir(gpio_dev, GPIO_0, GPIO_OUTPUT);
    gpio_drv_write(gpio_dev, GPIO_0, GPIO_IO_LOW);

    // set_PIC16LF15_SIGNAL_LOW
    // (GPIO) pin 46 set to RUN (0)
    pinmux_drv_setfunc(pinmux_dev, GPIO_46, GPIO46_GPIO46);
    gpio_drv_setdir(gpio_dev, GPIO_46, GPIO_OUTPUT);
    gpio_drv_write(gpio_dev, GPIO_46, GPIO_IO_LOW);

    pinmux_drv_setfunc(pinmux_dev, GPIO_48, GPIO48_GPIO48);
    gpio_drv_setdir(gpio_dev, GPIO_48, GPIO_OUTPUT);
    gpio_drv_write(gpio_dev, GPIO_48, GPIO_IO_HIGH);

    // set_PIC16LF15_RESET_RELEASE
    // (GPIO) pin 1 set to RESET (1) ???
    pinmux_drv_setfunc(pinmux_dev, GPIO_1, GPIO1_GPIO1);
    gpio_drv_setdir(gpio_dev, GPIO_1, GPIO_OUTPUT);
    gpio_drv_write(gpio_dev, GPIO_1, GPIO_IO_LOW);

    pinmux_drv_setfunc(pinmux_dev, GPIO_49, GPIO49_GPIO49);
    gpio_drv_setdir(gpio_dev, GPIO_49, GPIO_OUTPUT);
    gpio_drv_write(gpio_dev, GPIO_49, GPIO_IO_HIGH);
    vTaskDelay(800);

    // set up buttons
    pinmux_drv_setfunc(pinmux_dev, BTN_WIFI, GPIO22_GPIO22);
    init_gpio_input(gpio_dev, BTN_WIFI);
    pinmux_drv_setfunc(pinmux_dev, BTN_OTA, GPIO23_GPIO23);
    init_gpio_input(gpio_dev, BTN_OTA);

    pinmux_drv_close(pinmux_dev);
    gpio_drv_close(gpio_dev);
}

/**
 * @brief Initialize board functions that do not require FreeRTOS
 */
static void platform_init(void) {
    wmstdio_init(UART0_ID, 0);
    boot_report_flags();

    wifi_set_packet_retry_count(3);

    int ret = part_get_desc_from_id(FC_COMP_PSM, &fl);
    if (ret != WM_SUCCESS) {
        configPRINT("Unable to get flash desc from id\r\n");
    }
    ret = psm_module_init(&fl, &psm_hnd, NULL);
    if (ret != 0) {
        configPRINT("Failed to initialize psm module\r\n");
    }
    CRYPTO_Init();
    init_gpio();
    flash_drv_init();

    wmtime_init();
    pm_init();

    cli_init();
    pm_cli_init();
    wmtime_cli_init();
    pm_mcu_cli_init();
}

/**
 * @brief Application runtime entry point.
 */
int main(void) {
    platform_init();

    /* Create tasks that are not dependent on the Wi-Fi being initialized. */
    struct freertos_sockaddr udp_log_addr = {
        sizeof(struct freertos_sockaddr), FREERTOS_AF_INET,
        FreeRTOS_htons(5050), FreeRTOS_inet_addr("172.16.1.10")};
    xLoggingTaskInitialize(512, tskIDLE_PRIORITY,
                           mainLOGGING_MESSAGE_QUEUE_LENGTH, &udp_log_addr);

    static NetworkInterface_t sta_iface;
    static NetworkEndPoint_t sta_endpoint;
    pxMW300_FillInterfaceDescriptor(BSS_TYPE_STA, &sta_iface);
    FreeRTOS_FillEndPoint(&sta_iface, &sta_endpoint, default_ip_addr, netmask,
                          gateway_addr, dns_server_addr, mac_addr);
    sta_endpoint.bits.bWantDHCP = pdTRUE;

    static NetworkInterface_t uap_iface;
    static NetworkEndPoint_t uap_endpoint;
    pxMW300_FillInterfaceDescriptor(BSS_TYPE_UAP, &uap_iface);
    FreeRTOS_FillEndPoint(&uap_iface, &uap_endpoint, default_ip_addr, netmask,
                          gateway_addr, dns_server_addr, mac_addr);

    int res = FreeRTOS_IPInit_Multi();
    configASSERT(res);

    ctrl_queue = xQueueCreate(8, sizeof(ctrl_msg_t));
    configASSERT(ctrl_queue);

    ota_queue = xQueueCreate(5, sizeof(ota_cmd_t));
    configASSERT(ota_queue);
    xTaskCreate(ota_task, "OTA", 1024, ota_queue, tskIDLE_PRIORITY, NULL);

    pic_queue = xQueueCreate(5, sizeof(pic_cmd_t));
    configASSERT(pic_queue);
    pic_uart_task_params_t pic_task_params = {ctrl_queue, pic_queue};
    xTaskCreate(pic_uart_task, "PIC Comm", 512, &pic_task_params,
                tskIDLE_PRIORITY + 3, NULL);

    xTaskCreate(led_task, "LED Ctrl", 512, NULL, tskIDLE_PRIORITY, NULL);

    nm_queue = xQueueCreate(5, sizeof(nm_cmd_t));
    configASSERT(nm_queue);
    xTaskCreate(network_manager_task, "NetMgr", 1024, nm_queue,
                tskIDLE_PRIORITY, NULL);

    ctrl_msg_t ctrl_msg;
    for (;;) {
        if (xQueueReceive(ctrl_queue, &ctrl_msg, portMAX_DELAY) == pdPASS) {
            switch (ctrl_msg.type) {
                case CTRL_MSG_OTA_UPGRADE: {
                    ota_cmd_t cmd = OTA_CMD_UPGRADE;
                    xQueueSendToBack(ota_queue, &cmd, 0);
                    break;
                }
                case CTRL_MSG_DOOR_CONTROL: {
                    pic_cmd_t cmd =
                        ctrl_msg.msg.door_control.command == DOOR_CMD_OPEN
                            ? PIC_CMD_OPEN
                            : PIC_CMD_CLOSE;
                    xQueueSendToBack(pic_queue, &cmd, 0);
                    break;
                }
                case CTRL_MSG_DOOR_STATE_UPDATE:
                    publish_state(&ctrl_msg.msg.door_state);
                    break;
                case CTRL_MSG_WIFI_BUTTON: {
                    nm_cmd_t cmd = NM_CMD_AP_MODE;
                    LogDebug(("WiFi button pressed"));
                    xQueueSendToBack(nm_queue, &cmd, 0);
                    break;
                }
                case CTRL_MSG_OTA_BUTTON:
                    LogDebug(("OTA button pressed"));
                    break;

                default:
                    LogError(("unknown message type %d", ctrl_msg.type));
            }
        }
    }

    // not reached
    return 0;
}

void vApplicationIPNetworkEventHook_Multi(eIPCallbackEvent_t event,
                                          NetworkEndPoint_t *endpoint) {
    static bool tasks_created = false;

    LogDebug(("network event: %s", event == eNetworkUp ? "UP" : ""));

    if (event == eNetworkUp) {
        print_ip_config(endpoint);

        notify_dhcp_configured();
        if (!tasks_created) {
            // Create the tasks that use the TCP/IP stack if they have not
            // already been created.
            tasks_created = true;
            xTaskCreate(httpd_task, "HTTPd", 512, ctrl_queue, tskIDLE_PRIORITY,
                        NULL);
            xTaskCreate(mqtt_task, "MQTT", 1024, ctrl_queue,
                        tskIDLE_PRIORITY + 5, NULL);
        }
    }
}

/**
 * @brief User defined assertion call. This function is plugged into
 * configASSERT. See FreeRTOSConfig.h to define configASSERT to something
 * different.
 */
void vAssertCalled(const char *pcFile, uint32_t ulLine) {
    const uint32_t ulLongSleep = 1000UL;
    volatile uint32_t ulBlockVariable = 0UL;
    volatile char *pcFileName = (volatile char *)pcFile;
    volatile uint32_t ulLineNumber = ulLine;

    (void)pcFileName;
    (void)ulLineNumber;

    wmprintf("vAssertCalled %s, %ld\n", pcFile, (long)ulLine);
    wmstdio_flush();

    /* Setting ulBlockVariable to a non-zero value in the debugger will allow
     * this function to be exited. */
    taskDISABLE_INTERRUPTS();
    {
        while (ulBlockVariable == 0UL) {
            vTaskDelay(pdMS_TO_TICKS(ulLongSleep));
        }
    }
    taskENABLE_INTERRUPTS();
}

const char *pcApplicationHostnameHook() { return HOSTNAME; }

/**
 * @brief Warn user if pvPortMalloc fails.
 *
 * Called if a call to pvPortMalloc() fails because there is insufficient
 * free memory available in the FreeRTOS heap.  pvPortMalloc() is called
 * internally by FreeRTOS API functions that create tasks, queues, software
 * timers, and semaphores.  The size of the FreeRTOS heap is set by the
 * configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
 *
 */
void vApplicationMallocFailedHook() {
    configPRINT_STRING(("ERROR: Malloc failed to allocate memory\r\n"));
    taskDISABLE_INTERRUPTS();

    /* Loop forever */
    for (;;) {
    }
}

/**
 * @brief Loop forever if stack overflow is detected.
 *
 * If configCHECK_FOR_STACK_OVERFLOW is set to 1,
 * this hook provides a location for applications to
 * define a response to a stack overflow.
 *
 * Use this hook to help identify that a stack overflow
 * has occurred.
 *
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    wmprintf("ERROR: stack overflow in task %s\r\n", pcTaskName);
    portDISABLE_INTERRUPTS();

    /* Unused Parameters */
    (void)xTask;
    (void)pcTaskName;

    /* Loop forever */
    for (;;) {
    }
}

#undef xTimerGenericCommand
BaseType_t xTimerGenericCommand(TimerHandle_t timer,
                                const BaseType_t command_id,
                                const TickType_t opt_value,
                                BaseType_t *const higher_prio_task_woken,
                                const TickType_t ticks_to_wait) {
    return xTimerGenericCommandFromTask(timer, command_id, opt_value,
                                        higher_prio_task_woken, ticks_to_wait);
}

void abort(void) {
    configASSERT(false);
    while (true) {
    }
}

/**
 * @brief Implements libc calloc semantics using the FreeRTOS heap
 */
void *pvCalloc(size_t xNumElements, size_t xSize) {
    void *pvNew = pvPortMalloc(xNumElements * xSize);

    if (NULL != pvNew) {
        memset(pvNew, 0, xNumElements * xSize);
    }

    return pvNew;
}

void reboot() {
    wifi_deinit();
    LogInfo(("rebooting"));
    pm_reboot_soc();
}