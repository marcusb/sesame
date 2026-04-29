#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_ND.h"
#include "NetworkInterface.h"
#include "app_logging.h"
#include "queue.h"
#include "task.h"

// mw320
#include "boot_flags.h"
#include "debug_console.h"
#include "fsl_aes.h"
#include "fsl_gpio.h"
#include "fsl_power.h"
#include "fsl_wdt.h"
#include "ksdk_mbedtls.h"
#include "mflash_drv.h"
#include "partition.h"
#include "psm.h"
#include "wifi.h"

// Application
#include "board.h"
#include "config_manager.h"
#include "controller.h"
#include "dhcp.h"
#include "gpio.h"
#include "leds.h"
#include "mw300_rd_net.h"
#include "network.h"
#include "ota.h"
#include "pic_uart.h"
#include "pin_mux.h"
#include "time_util.h"

extern QueueHandle_t ctrl_queue;
extern QueueHandle_t ota_queue;
extern QueueHandle_t pic_queue;
extern QueueHandle_t nm_queue;
extern psm_hnd_t psm_hnd;

NetworkInterface_t sta_iface;
NetworkInterface_t uap_iface;

static SemaphoreHandle_t aes_mutex;

static const uint8_t default_ip_addr[4] = {192, 168, 4, 1};
static const uint8_t netmask[4] = {255, 255, 255, 0};
static const uint8_t gateway_addr[4] = {0};
static const uint8_t dns_server_addr[4] = {0};

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

static status_t aes_lock(void) {
    if (pdTRUE == xSemaphoreTakeRecursive(aes_mutex, portMAX_DELAY)) {
        return kStatus_Success;
    } else {
        return kStatus_Fail;
    }
}

static void aes_unlock(void) { xSemaphoreGiveRecursive(aes_mutex); }

static void aes_init(void) {
    aes_mutex = xSemaphoreCreateRecursiveMutex();
    assert(aes_mutex != NULL);
    AES_Init(AES);
    AES_SetLockFunc(aes_lock, aes_unlock);
}

void board_init(void) {
    boot_init();
    report_boot_flags();
    init_gpio();
    mflash_drv_init();
    part_init();
    aes_init();
    setup_rtc();
    int wifi_res = init_wifi_driver();
    assert(wifi_res == WM_SUCCESS);
    (void)wifi_res;

    // initialize watchdog, unless debugger is connected
    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
        PRINTF("debugger connected, not starting watchdog\r\n");
    } else {
        init_watchdog();
    }
}

void configure_netif() {
    wifi_mac_addr_t mac_addr;
    const uint8_t* hwaddr = (const uint8_t*)mac_addr.mac;

    int ret = wifi_get_device_mac_addr(&mac_addr);
    assert(ret == WM_SUCCESS);

    static NetworkEndPoint_t eps[3];
    mw300_new_netif_desc(BSS_TYPE_STA, &sta_iface);
    FreeRTOS_FillEndPoint(&sta_iface, &eps[0], default_ip_addr, netmask,
                          gateway_addr, dns_server_addr, hwaddr);
    eps[0].bits.bWantDHCP = pdTRUE;

#if ipconfigUSE_IPv6
    // local IPv6 address chosen randomly
    IPv6_Address_t local_prefix = {{0xfe, 0x80}};
    IPv6_Address_t local_addr;
    IPv6_Address_t gateway_addr_ip6 = {
        {0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};
    FreeRTOS_CreateIPv6Address(&local_addr, &local_prefix, 64, pdTRUE);
    FreeRTOS_FillEndPoint_IPv6(&sta_iface, &eps[1], &local_addr, &local_prefix,
                               64, &gateway_addr_ip6, NULL, hwaddr);

    // ULA address configured via SLAAC and RA.
    // Form the EUI64 address from MAC for SLAAC.
    IPv6_Address_t eui64_addr = {{0xfe, 0x80, 0, 0, 0, 0, 0, 0,
                                  hwaddr[0] | 0x02, hwaddr[1], hwaddr[2], 0xff,
                                  0xfe, hwaddr[3], hwaddr[4], hwaddr[5]}};
    FreeRTOS_FillEndPoint_IPv6(&sta_iface, &eps[2], &eui64_addr, &local_prefix,
                               64, &gateway_addr_ip6, NULL, hwaddr);
#endif
#if ipconfigUSE_IPv6
    eps[2].bits.bWantRA = pdTRUE;
#endif

    static NetworkEndPoint_t uap_endpoint;
    mw300_new_netif_desc(BSS_TYPE_UAP, &uap_iface);
    FreeRTOS_FillEndPoint(&uap_iface, &uap_endpoint, default_ip_addr, netmask,
                          gateway_addr, dns_server_addr, hwaddr);

    FreeRTOS_SetDNSIPPreference(xPreferenceIPv4);
    int res = FreeRTOS_IPInit_Multi();
    assert(res);
}

void create_board_tasks() {
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
}

void reboot() {
    wifi_deinit();
    LogInfo(("rebooting"));
    NVIC_SystemReset();
}

extern void main_task(void* param);
extern void setup_heap();

extern unsigned __HeapBase, __HeapLimit, __HeapBase_sram0, __HeapLimit_sram0;
void setup_heap() {
    HeapRegion_t xHeapRegions[] = {
        {(uint8_t*)&__HeapBase_sram0,
         (unsigned)&__HeapLimit_sram0 - (unsigned)&__HeapBase_sram0},
        {(uint8_t*)&__HeapBase, (unsigned)&__HeapLimit - (unsigned)&__HeapBase},
        {NULL, 0}};

    vPortDefineHeapRegions(xHeapRegions);
}

int main(void) {
    board_init_pins();
    init_boot_clocks();
    init_debug_console();

    CLOCK_EnableXtal32K(kCLOCK_Osc32k_Internal);
    CLOCK_AttachClk(kXTAL32K_to_RTC);

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

void notify_board_dhcp_configured() { notify_dhcp_configured(); }

void notify_board_ipv6_addr_change() { notify_ipv6_addr_change(); }

bool is_sta_iface(NetworkInterface_t* iface) { return iface == &sta_iface; }

bool is_uap_iface(NetworkInterface_t* iface) { return iface == &uap_iface; }
