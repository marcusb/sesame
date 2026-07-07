#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "88MW320.h"
#include "fsl_clock.h"
#include "fsl_pinmux.h"
#include "fsl_power.h"
#include "pin_mux.h"

/**
 * Early reset hook: runs BEFORE any RAM access or flash XIP.
 * Only disables the watchdog timer. Boot2 already configured the flash
 * controller correctly (CMD_TYPE=7, FastReadQuadIOContinuous = XIP mode).
 * Do NOT touch FCCR here — changing CMD_TYPE to 6 would disable XIP and
 * make __start unreachable from flash.
 */
__attribute__((naked)) void soc_early_reset_hook(void)
{
    __asm volatile (
        // Disable watchdog (WDT base 0x48040000)
        "ldr r0, =0x48040000\n"
        "movs r1, #0\n"
        "str r1, [r0]\n"
        "bx lr\n"
    );
}

// RAM build: flash base is at SRAM1 (0x20000000), skip flash controller ops
#define IS_RAM_BUILD (!DT_NODE_EXISTS(DT_CHOSEN(zephyr_flash)))

#define BOARD_BOOTCLOCKRUN_CORE_CLOCK 200000000U

// Diagnostic: write here before each init stage. Check with debugger if hang.
volatile uint32_t __attribute__((used)) boot_diag
    __attribute__((section(".data"))) = 0;

#define BOOT_DIAG_STAGE(x) (boot_diag = (x))
#define BOOT_DIAG_PMU_PAD 0x01
#define BOOT_DIAG_FLASH_DEINIT 0x02
#define BOOT_DIAG_REFCLK_SYS 0x03
#define BOOT_DIAG_RC32M 0x04
#define BOOT_DIAG_REFCLK_OSC 0x05
#define BOOT_DIAG_SFLL 0x06
#define BOOT_DIAG_SYSCLK 0x07
#define BOOT_DIAG_FLASH_INIT 0x08
#define BOOT_DIAG_PINMUX 0x09
#define BOOT_DIAG_DONE 0x0A

// Timeout for hardware ready loops (in iterations). Set diag to 0x80+stage on
// timeout.
#define BOOT_TIMEOUT 0xFFFFFFFF
#define BOOT_TIMEOUT_FLAG 0x80

static clock_sfll_config_t sfll_config = {
    .sfllSrc = kCLOCK_SFllSrcMainXtal, /* XTAL clock */
    .refDiv = 0x60U,
    .fbDiv = 0xFAU,
    .kvco = 0x1U,
    .postDiv = 0x0U};

void deinit_flashc(void) {
    uint32_t reg = FLASHC->FCCR;
    uint32_t timeout;

    /* Disable cache */
    reg &= ~FLASHC_FCCR_CACHE_EN_MASK;
    FLASHC->FCCR = reg;
    /* Set CMD_TYPE to exit continuous read mode */
    reg = (reg & ~FLASHC_FCCR_CMD_TYPE_MASK) | FLASHC_FCCR_CMD_TYPE(0xCU);
    FLASHC->FCCR = reg;
    /* Wait exit done */
    timeout = BOOT_TIMEOUT;
    while ((FLASHC->FCSR & FLASHC_FCSR_CONT_RD_MD_EXIT_DONE_MASK) == 0U) {
        if (--timeout == 0) {
            boot_diag = BOOT_TIMEOUT_FLAG | BOOT_DIAG_FLASH_DEINIT;
            while (1) {
            }
        }
    }
    /* Clear exit done flag */
    FLASHC->FCSR = FLASHC_FCSR_CONT_RD_MD_EXIT_DONE_MASK;
    /* Set pad mux to QSPI */
    FLASHC->FCCR &= ~FLASHC_FCCR_FLASHC_PAD_EN_MASK;
}

void init_flashc(void) {
    uint32_t reg = FLASHC->FCCR;

    /* Set pad mux to FLASHC */
    reg |= FLASHC_FCCR_FLASHC_PAD_EN_MASK;
    FLASHC->FCCR = reg;

    /* Set CMD_TYPE to continuous read mode (0x7 = FastReadQuadIOContinuous = XIP) */
    reg = (reg & ~FLASHC_FCCR_CMD_TYPE_MASK) | FLASHC_FCCR_CMD_TYPE(0x7U);
    FLASHC->FCCR = reg;
    /* Enable cache */
    reg |= FLASHC_FCCR_CACHE_EN_MASK;
    FLASHC->FCCR = reg;
}

static void init_boot_clocks(void) {
    POWER_PowerOnVddioPad(kPOWER_VddIoAon);
    POWER_PowerOnVddioPad(kPOWER_VddIo0);
    POWER_PowerOnVddioPad(kPOWER_VddIo1);
    POWER_PowerOnVddioPad(kPOWER_VddIo2);
    POWER_PowerOnVddioPad(kPOWER_VddIo3);
    /* Do not touch flash controller here in XIP. 
       deinit_flashc() exits continuous read mode, causing a hardfault 
       when the CPU fetches the next instruction from flash. */

#if !DT_NODE_EXISTS(DT_CHOSEN(zephyr_flash))
    /* Enable RC32M. */
    BOOT_DIAG_STAGE(BOOT_DIAG_RC32M);
    CLOCK_EnableClock(kCLOCK_Rc32m);
    CLOCK_EnableRC32M(false);

    /* Enable UART0 clock gate (bit 4 of PERI_CLK_EN) */
    CLOCK_EnableClock(kCLOCK_Uart0);

    /* Set the PMU clock divider to 1 */
    CLOCK_SetClkDiv(kCLOCK_DivPmu, 1U);

    /* Set external Xtal frequency to clock driver */
    CLOCK_SetMainXtalFreq(CLK_MAINXTAL_CLK);

    if ((PMU->CLK_SRC & PMU_CLK_SRC_SYS_CLK_SEL_MASK) != kCLOCK_SysClkSrcSFll) {
        /* Enable System OSC 38.4M. */
        BOOT_DIAG_STAGE(BOOT_DIAG_REFCLK_OSC);
        CLOCK_EnableRefClk(kCLOCK_RefClk_SYS);

        /* Initialize SFLL to 200M. */
        BOOT_DIAG_STAGE(BOOT_DIAG_SFLL);
        CLOCK_InitSFll(&sfll_config);

        /* Set dividers */
        CLOCK_SetClkDiv(kCLOCK_DivApb0, 2U);
        CLOCK_SetClkDiv(kCLOCK_DivApb1, 2U);
        CLOCK_SetClkDiv(kCLOCK_DivPmu, 4U);

        /* Switch system clock source to SFLL before RC32M calibration */
        CLOCK_SetSysClkSource(kCLOCK_SysClkSrcSFll);
    }

    CLOCK_SetClkDiv(kCLOCK_DivQspi, 4U);

    /* Calibrate RC32M */
    CLOCK_CalibrateRC32M(true, 0U);
    /* RC32M enabled, controller clock can be gated. */
    CLOCK_DisableClock(kCLOCK_Rc32m);

    /* Reset the PMU clock divider to 1 */
    CLOCK_SetClkDiv(kCLOCK_DivPmu, 1U);

    /* Restart flash controller (needed for XIP and mflash operations) */
    if (IS_RAM_BUILD) {
        init_flashc();
    }

    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
    BOOT_DIAG_STAGE(0x0B);  // DIAG: clocks done
#endif
}

void board_init_pins(void) {
#if !DT_NODE_EXISTS(DT_CHOSEN(zephyr_flash))
    PINMUX_PinMuxSet(BOARD_UART0_TX_PIN,
                     BOARD_UART0_TX_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_UART0_RX_PIN,
                     BOARD_UART0_RX_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_UART1_TX_PIN,
                     BOARD_UART1_TX_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_UART1_RX_PIN,
                     BOARD_UART1_RX_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);

    PINMUX_PinMuxSet(0, PINMUX_GPIO0_GPIO0 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(46, PINMUX_GPIO46_GPIO46 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(48, PINMUX_GPIO48_GPIO48 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(1, PINMUX_GPIO1_GPIO1 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(49, PINMUX_GPIO49_GPIO49 | PINMUX_MODE_DEFAULT);

    PINMUX_PinMuxSet(BOARD_XTAL32K_IN_PIN,
                     BOARD_XTAL32K_IN_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(BOARD_XTAL32K_OUT_PIN,
                     BOARD_XTAL32K_OUT_PIN_FUNCTION_ID | PINMUX_MODE_DEFAULT);
#endif
}

void init_debug_console(void) {
}

static int nxp_88mw320_init(void) {
    BOOT_DIAG_STAGE(0x01);  // DIAG: SOC init entry
    WDT->WDT_CR = 0;        // Disable watchdog again (belt and suspenders)
    board_init_pins();
    BOOT_DIAG_STAGE(BOOT_DIAG_PINMUX);
    init_boot_clocks();
    BOOT_DIAG_STAGE(0x0B);  // Clocks done
    init_debug_console();
    BOOT_DIAG_STAGE(0x0C);  // UART clock done

#if !DT_NODE_EXISTS(DT_CHOSEN(zephyr_flash))
    CLOCK_EnableXtal32K(kCLOCK_Osc32k_Internal);
    CLOCK_AttachClk(kXTAL32K_to_RTC);
#endif

    BOOT_DIAG_STAGE(BOOT_DIAG_DONE);
    return 0;
}

SYS_INIT(nxp_88mw320_init, PRE_KERNEL_1, 0);
