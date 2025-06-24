#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_logging.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "NetworkInterface.h"
#include "mw300_rd_net.h"
#include "queue.h"
#include "task.h"

// mw320
#include "boot_flags.h"
#include "cli.h"
#include "fsl_aes.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_wdt.h"
#include "ksdk_mbedtls.h"
#include "mflash_drv.h"
#include "partition.h"
#include "psm-v2.h"
#include "wifi.h"

// Application
#include "board.h"
#include "config_manager.h"
#include "controller.h"
#include "dhcp.h"
#include "gpio.h"
#include "httpd.h"
#include "leds.h"
#include "logging.h"
#include "mqtt.h"
#include "network.h"
#include "ota.h"
#include "pic_uart.h"
#include "pin_mux.h"
#include "syslog.h"
#include "time_util.h"

/* Logging Task Defines. */
#define mainLOGGING_MESSAGE_QUEUE_LENGTH (16)

// the global controller event queue, all tasks write to it
QueueHandle_t ctrl_queue;
psm_hnd_t psm_hnd;

static QueueHandle_t ota_queue;
static QueueHandle_t pic_queue;
static QueueHandle_t nm_queue;

// IP configuration to use when DHCP does not succeed
static uint8_t mac_addr[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
static const uint8_t default_ip_addr[4] = {192, 168, 4, 1};
static const uint8_t netmask[4] = {255, 255, 255, 0};
static const uint8_t gateway_addr[4] = {0};
static const uint8_t dns_server_addr[4] = {0};

static NetworkInterface_t sta_iface;
static NetworkInterface_t uap_iface;

#if 0
static void hardfault() {
    uint64_t x = *(uint64_t *)(0x20CDCDCD);
    PRINTF("%lu", x);
}
#endif

static void log_console(const log_msg_t *log) {
    static const char levels[] = "-EWID";
    uint8_t level = log->level;
    if (level > LOG_LEVEL_LAST || level < 0) {
        level = LOG_NONE;
    }
    PRINTF("%lu %lu %c[%s] %s\r\n", log->msg_id, log->ticks, levels[level],
           log->task_name, log->msg);
}

extern unsigned __HeapBase, __HeapLimit, __HeapBase_sram0, __HeapLimit_sram0;
static void setup_heap() {
    HeapRegion_t xHeapRegions[] = {
        {(uint8_t *)&__HeapBase_sram0,
         (unsigned)&__HeapLimit_sram0 - (unsigned)&__HeapBase_sram0},
        {(uint8_t *)&__HeapBase,
         (unsigned)&__HeapLimit - (unsigned)&__HeapBase},
        {NULL, 0}};

    vPortDefineHeapRegions(xHeapRegions);
}

typedef struct {
    int gpio;
    ctrl_msg_type_t msg_type;
} button_def_t;

static const button_def_t buttons[] = {
    {BOARD_SW_WIFI_PIN, CTRL_MSG_WIFI_BUTTON},
    {BOARD_SW_OTA_PIN, CTRL_MSG_OTA_BUTTON},
    {-1}};

void GPIO_IRQHandler(void) {
    int pin;
    for (int i = 0; (pin = buttons[i].gpio) != -1; i++) {
        uint32_t port = GPIO_PORT(pin);
        uint32_t mask = 1UL << GPIO_PORT_PIN(pin);
        if (GPIO_PortGetInterruptFlags(GPIO, port) & mask) {
            GPIO_PortClearInterruptFlags(GPIO, port, mask);
            ctrl_msg_t msg = {buttons[i].msg_type};
            xQueueSendToBackFromISR(ctrl_queue, &msg, NULL);
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

static void init_gpio_input(int pin) {
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput,
        0,
    };
    GPIO_PinSetInterruptConfig(GPIO, pin, kGPIO_InterruptFallingEdge);
    GPIO_PortEnableInterrupts(GPIO, GPIO_PORT(pin), 1UL << GPIO_PORT_PIN(pin));
    GPIO_PinInit(GPIO, pin, &sw_config);
}

static void init_gpio() {
    EnableIRQ(GPIO_IRQn);
    // lower interrupt priority so we can call FreeRTOS syscalls from ISR
    NVIC_SetPriority(GPIO_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    gpio_set_output(0);
    gpio_clear(0);

    // set_PIC16LF15_SIGNAL_LOW
    // GPIO46 set to RUN (0)
    gpio_set_output(46);
    gpio_clear(46);

    gpio_set_output(48);
    gpio_set(48);

    // set_PIC16LF15_RESET_RELEASE
    // GPIO pin 1 set to RESET (1)
    gpio_set_output(1);
    gpio_clear(1);

    gpio_set_output(49);
    gpio_set(49);

    int pin;
    for (int i = 0; (pin = buttons[i].gpio) != -1; i++) {
        init_gpio_input(pin);
    }
}

static void psm_init() {
    static flash_desc_t fl;
    int ret = part_get_desc_from_id(FC_COMP_PSM, &fl);
    if (ret != WM_SUCCESS) {
        LogError(("Unable to get flash desc from id"));
    }

    ret = psm_module_init(&fl, &psm_hnd, NULL);
    if (ret != 0) {
        LogError(("Failed to initialize psm module\r\n"));
    }
}

static void report_boot_flags(void) {
    PRINTF("Boot Flags: 0x%x\r\n", g_boot_flags);
    PRINTF(" - Partition Table: %d\r\n",
           !(!(g_boot_flags & BOOT_PARTITION_TABLE_MASK)));
    PRINTF(" - Firmware Partition: %d\r\n", g_boot_flags & BOOT_PARTITION_MASK);
    if (g_boot_flags & BOOT_MAIN_FIRMWARE_BAD_CRC) {
        PRINTF(" - Backup firmware due to CRC error in main firmware\r\n");
    }
    PRINTF("Boot Info:\r\n");
    PRINTF(" - Chip revision id: 0x%x\r\n", SYS_CTL->REV_ID);
    PRINTF("Reset Cause Register: 0x%x\r\n", boot_reset_cause());
    if (boot_reset_cause() & (1 << 5)) {
        PRINTF(" - Watchdog reset bit is set\r\n");
    };
}

static void init_watchdog() {
    wdt_config_t config;
    WDT_GetDefaultConfig(&config);
    config.timeoutValue = kWDT_TimeoutVal2ToThePowerOf29;
    config.timeoutMode = kWDT_ModeTimeoutInterrupt;
    config.enableWDT = true;
    WDT_Init(WDT, &config);
    EnableIRQ(WDT_IRQn);
    WDT_DisableInterrupt(WDT);
}

/**
 * @brief Initialize board functions that do not require FreeRTOS
 */
static void board_init(void) {
    boot_init();
    report_boot_flags();
    init_gpio();
    mflash_drv_init();
    part_init();

    // pm_init();
    setup_rtc();

    // initialize watchdog, unless debugger is connected
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
        PRINTF("debugger connected, not starting watchdog\r\n");
    } else {
        init_watchdog();
    }
}

static void create_tasks() {
    ctrl_queue = xQueueCreate(8, sizeof(ctrl_msg_t));
    configASSERT(ctrl_queue);

    ota_queue = xQueueCreate(5, sizeof(ota_msg_t));
    configASSERT(ota_queue);
    xTaskCreate(ota_task, "OTA", 1024, ota_queue, tskIDLE_PRIORITY, NULL);

    pic_queue = xQueueCreate(5, sizeof(pic_cmd_t));
    configASSERT(pic_queue);
    xTaskCreate(pic_uart_task, "PIC", 512, pic_queue, tskIDLE_PRIORITY + 1,
                NULL);

    xTaskCreate(led_task, "LED", 512, NULL, tskIDLE_PRIORITY, NULL);

    nm_queue = xQueueCreate(5, sizeof(nm_msg_t));
    configASSERT(nm_queue);
    xTaskCreate(network_manager_task, "NetMgr", 512, nm_queue, tskIDLE_PRIORITY,
                NULL);

    xTaskCreate(httpd_task, "HTTPd", 512, NULL, tskIDLE_PRIORITY, NULL);
}

void reboot() {
    wifi_deinit();
    LogInfo(("rebooting"));
    NVIC_SystemReset();
    // pm_reboot_soc();
}

static void main_task(void *param) {
    board_init();

    register_log_backend(log_console);
    register_log_backend(log_syslog);
    init_logging(512, tskIDLE_PRIORITY, mainLOGGING_MESSAGE_QUEUE_LENGTH);

    check_ota_test_image();
    psm_init();
    load_config();

    static NetworkEndPoint_t sta_endpoint;
    mw300_new_netif_desc(BSS_TYPE_STA, &sta_iface);
    FreeRTOS_FillEndPoint(&sta_iface, &sta_endpoint, default_ip_addr, netmask,
                          gateway_addr, dns_server_addr, mac_addr);
    sta_endpoint.bits.bWantDHCP = pdTRUE;

    static NetworkEndPoint_t uap_endpoint;
    mw300_new_netif_desc(BSS_TYPE_UAP, &uap_iface);
    FreeRTOS_FillEndPoint(&uap_iface, &uap_endpoint, default_ip_addr, netmask,
                          gateway_addr, dns_server_addr, mac_addr);

    int res = FreeRTOS_IPInit_Multi();
    configASSERT(res);

    create_tasks();
    ctrl_msg_t ctrl_msg;
    for (;;) {
        WDT_Refresh(WDT);
        if (xQueueReceive(ctrl_queue, &ctrl_msg, 1000) == pdPASS) {
            switch (ctrl_msg.type) {
                case CTRL_MSG_OTA_UPGRADE: {
                    ota_msg_t ota_msg = {OTA_CMD_UPGRADE,
                                         .msg = {ctrl_msg.msg.ota_upgrade}};
                    xQueueSendToBack(ota_queue, &ota_msg, 0);
                    break;
                }

                case CTRL_MSG_OTA_PROMOTE: {
                    ota_msg_t ota_msg = {OTA_CMD_PROMOTE};
                    xQueueSendToBack(ota_queue, &ota_msg, 0);
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
                    nm_msg_t cmd = {NM_CMD_AP_MODE};
                    LogDebug(("WiFi button pressed"));
                    xQueueSendToBack(nm_queue, &cmd, 0);
                    break;
                }

                case CTRL_MSG_OTA_BUTTON:
                    LogDebug(("OTA button pressed"));
                    break;

                case CTRL_MSG_WIFI_CONFIG: {
                    app_config.network_config = ctrl_msg.msg.network_cfg;
                    app_config.has_network_config = true;
                    save_config();
                    vTaskDelay(500);
                    reboot();
                    break;
                }

                case CTRL_MSG_MQTT_CONFIG: {
                    app_config.mqtt_config = ctrl_msg.msg.mqtt_cfg;
                    app_config.has_mqtt_config = true;
                    save_config();
                    reboot();
                    break;
                }

                case CTRL_MSG_LOGGING_CONFIG: {
                    app_config.logging_config = ctrl_msg.msg.logging_cfg;
                    app_config.has_logging_config = true;
                    save_config();
                    reboot();
                    break;
                }

                default:
                    LogError(("unknown message type %d", ctrl_msg.type));
            }
        }
    }
}

int main(void) {
    board_init_pins();
    init_boot_clocks();
    init_debug_console();

    AES_Init(AES);

    uint8_t hash[32];
    uint32_t len = sizeof(hash);
    BOARD_GetHash(hash, &len);
    configASSERT(len > 0U);
    mbedtls_hardware_init_hash(hash, len);

    setup_heap();
    if (xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE + 896, NULL,
                    tskIDLE_PRIORITY + 3, NULL) != pdPASS) {
        PRINTF("main task creation failed\r\n");
    }
    vTaskStartScheduler();
    return 0;
}

static void print_ip_config(NetworkEndPoint_t *endpoint) {
    if (endpoint != NULL) {
        char ip_addr_s[16];
        char netmask_s[16];
        char gateway_s[16];
        char dns_server_s[16];
        FreeRTOS_inet_ntop4(&endpoint->ipv4_settings.ulIPAddress, ip_addr_s,
                            sizeof(ip_addr_s));
        FreeRTOS_inet_ntop4(&endpoint->ipv4_settings.ulNetMask, netmask_s,
                            sizeof(netmask_s));
        FreeRTOS_inet_ntop4(&endpoint->ipv4_settings.ulGatewayAddress,
                            gateway_s, sizeof(gateway_s));
        FreeRTOS_inet_ntop4(&endpoint->ipv4_settings.ulDNSServerAddresses[0],
                            dns_server_s, sizeof(dns_server_s));
        LogInfo(("%s: IPv4 %s/%s, gw %s, dns %s",
                 endpoint->pxNetworkInterface->pcName, ip_addr_s, netmask_s,
                 gateway_s, dns_server_s));
    }
}

bool is_sta_network_up() { return sta_iface.pxEndPoint->bits.bEndPointUp; }

void vApplicationIPNetworkEventHook_Multi(eIPCallbackEvent_t event,
                                          NetworkEndPoint_t *endpoint) {
    static bool tasks_created = false;

    if (event == eNetworkUp) {
        LogInfo(("%s: interface up", endpoint->pxNetworkInterface->pcName));
        print_ip_config(endpoint);

        if (endpoint->pxNetworkInterface == &sta_iface) {
            notify_dhcp_configured();
            if (!tasks_created) {
                // Create the tasks that use the TCP/IP stack if they have
                // not already been created.
                tasks_created = true;
                configure_logging(&app_config.logging_config.syslog_config);
                xTaskCreate(mqtt_task, "MQTT", 512, &app_config.mqtt_config,
                            tskIDLE_PRIORITY, NULL);
            }
        } else if (endpoint->pxNetworkInterface == &uap_iface) {
            static dhcp_task_params_t dhcp_params;
            dhcp_params.endpoint = endpoint;
            xTaskCreate(dhcpd_task, "DHCPd", 512, &dhcp_params,
                        tskIDLE_PRIORITY, NULL);
        }
    }
}

/**
 * @brief User defined assertion call. This function is plugged into
 * configASSERT. See FreeRTOSConfig.h to define configASSERT to something
 * different.
 */
void vAssertCalled(const char *file, uint32_t line) {
    volatile uint32_t ulBlockVariable = 0UL;

    PRINTF("assert in %s:%lu\r\n", file, line);

    /* Setting ulBlockVariable to a non-zero value in the debugger will
     * allow this function to be exited. */
    taskDISABLE_INTERRUPTS();
    {
        while (ulBlockVariable == 0UL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    taskENABLE_INTERRUPTS();
}

/**
 * @brief Warn user if pvPortMalloc fails.
 *
 * Called if a call to pvPortMalloc() fails because there is insufficient
 * free memory available in the FreeRTOS heap.
 */
void vApplicationMallocFailedHook() {
    PRINTF(("ERROR: Malloc failed to allocate memory\r\n"));
    taskDISABLE_INTERRUPTS();
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
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *task_name) {
    PRINTF("ERROR: stack overflow in task %s\r\n", task_name);
    portDISABLE_INTERRUPTS();
    for (;;) {
    }
}
