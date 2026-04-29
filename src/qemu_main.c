#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_ND.h"
#include "NetworkInterface.h"
#include "app_logging.h"
#include "queue.h"
#include "task.h"

// mw320 (stubs or generic)
#include "debug_console.h"
#include "psm.h"

// Application
#include "config_manager.h"
#include "controller.h"
#include "time_util.h"

extern QueueHandle_t ctrl_queue;
extern QueueHandle_t ota_queue;
extern QueueHandle_t pic_queue;
extern QueueHandle_t nm_queue;
extern psm_hnd_t psm_hnd;

static const uint8_t default_ip_addr[4] = {10, 0, 2, 15};
static const uint8_t netmask[4] = {255, 255, 255, 0};
static const uint8_t gateway_addr[4] = {10, 0, 2, 2};
static const uint8_t dns_server_addr[4] = {10, 0, 2, 3};
static const uint8_t hwaddr[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

static NetworkInterface_t eth_iface;

void board_init(void) {
    // No-op for QEMU
}

void configure_netif() {
    static NetworkEndPoint_t eps[3];

    // In QEMU build, FREERTOS_PLUS_TCP_NETWORK_IF will be MPS2_AN385
    // which provides pxMPS2_FillInterfaceDescriptor
    extern NetworkInterface_t* pxMPS2_FillInterfaceDescriptor(
        BaseType_t xEMACIndex, NetworkInterface_t * pxInterface);
    pxMPS2_FillInterfaceDescriptor(0, &eth_iface);

    // Set Ethernet IRQ (13) priority to something safe for FreeRTOS
    // configMAX_SYSCALL_INTERRUPT_PRIORITY is 0xBF
    // NVIC Priority Registers start at 0xE000E400.
    // IRQ 13 is the 14th interrupt. IPR register index = 13 / 4 = 3.
    // Offset in register = (13 % 4) * 8 = 8.
    // We'll set it to 0xE0 (low priority).
    *(volatile uint32_t*)(0xE000E400 + 3 * 4) = (0xE0 << 8);

    FreeRTOS_FillEndPoint(&eth_iface, &eps[0], default_ip_addr, netmask,
                          gateway_addr, dns_server_addr, hwaddr);
    eps[0].bits.bWantDHCP = pdTRUE;

#if ipconfigUSE_IPv6
    IPv6_Address_t local_prefix = {{0xfe, 0x80}};
    IPv6_Address_t local_addr;
    IPv6_Address_t gateway_addr_ip6 = {
        {0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};
    FreeRTOS_CreateIPv6Address(&local_addr, &local_prefix, 64, pdTRUE);
    FreeRTOS_FillEndPoint_IPv6(&eth_iface, &eps[1], &local_addr, &local_prefix,
                               64, &gateway_addr_ip6, NULL, hwaddr);

    IPv6_Address_t eui64_addr = {{0xfe, 0x80, 0, 0, 0, 0, 0, 0,
                                  hwaddr[0] | 0x02, hwaddr[1], hwaddr[2], 0xff,
                                  0xfe, hwaddr[3], hwaddr[4], hwaddr[5]}};
    FreeRTOS_FillEndPoint_IPv6(&eth_iface, &eps[2], &eui64_addr, &local_prefix,
                               64, &gateway_addr_ip6, NULL, hwaddr);
    eps[2].bits.bWantRA = pdTRUE;
#endif

    FreeRTOS_SetDNSIPPreference(xPreferenceIPv4);
    int res = FreeRTOS_IPInit_Multi();
    assert(res);
}

void create_board_tasks() {
    // No board tasks in QEMU
}

#include "fsl_common.h"

void reboot() {
    LogInfo(("QEMU reboot (NVIC_SystemReset)"));
    NVIC_SystemReset();
}

extern void main_task(void* param);
extern void setup_heap();

extern unsigned __HeapBase, __HeapLimit;
void setup_heap() {
    // Simplified heap for QEMU
    HeapRegion_t xHeapRegions[] = {
        {(uint8_t*)&__HeapBase, (unsigned)&__HeapLimit - (unsigned)&__HeapBase},
        {NULL, 0}};

    vPortDefineHeapRegions(xHeapRegions);
}

uint32_t SystemCoreClock = 25000000;  // 25MHz standard for MPS2

void vApplicationIdleHook() {}

extern void mbedtls_hardware_init_hash(uint8_t* entropy, size_t len);

int main(void) {
    // In QEMU, stdout goes to host via semihosting automatically with
    // --oslib=semihost
    setup_heap();

    /* QEMU has no hardware entropy source. Seed the mbedtls entropy pool
     * with a deterministic placeholder so internal_entropy_poll() answers
     * with non-zero data — without this, ctr_drbg_random() returns
     * NO_SOURCE, crypto.random() yields nil to Berry, and Matter's
     * generate_random_passcode() spins forever. The PRNG is still seeded
     * in CTR_DRBG; this just gates startup. */
    static uint8_t qemu_seed_entropy[32] = {
        0xa1, 0x5e, 0xc0, 0xde, 0xde, 0xad, 0xbe, 0xef, 0x00, 0x11, 0x22,
        0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
        0xee, 0xff, 0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe,
    };
    mbedtls_hardware_init_hash(qemu_seed_entropy, sizeof(qemu_seed_entropy));

    if (xTaskCreate(main_task, "main", configMINIMAL_STACK_SIZE + 896, NULL,
                    tskIDLE_PRIORITY + 3, NULL) != pdPASS) {
        printf("main task creation failed\n");
    }
    vTaskStartScheduler();
    return 0;
}

void notify_board_dhcp_configured() {
    // No-op or generic
}

void notify_board_ipv6_addr_change() {
    // No-op or generic
}

bool is_sta_iface(NetworkInterface_t* iface) { return iface == &eth_iface; }

bool is_uap_iface(NetworkInterface_t* iface) { return false; }
