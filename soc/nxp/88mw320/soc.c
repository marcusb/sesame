#include <zephyr/init.h>

#include "fsl_clock.h"
#include "fsl_power.h"

#ifndef CONFIG_XIP
#define IS_RAM_BUILD 1
#else
#define IS_RAM_BUILD 0
#endif

#define BOARD_BOOTCLOCKRUN_CORE_CLOCK 200000000U

#if IS_RAM_BUILD
static clock_sfll_config_t sfll_config = {
    .sfllSrc = kCLOCK_SFllSrcMainXtal, /* XTAL clock */
    .refDiv = 0x60U,
    .fbDiv = 0xFAU,
    .kvco = 0x1U,
    .postDiv = 0x0U};

static void deinit_flashc(void) {
    uint32_t reg = FLASHC->FCCR;

    /* Disable cache */
    reg &= ~FLASHC_FCCR_CACHE_EN_MASK;
    FLASHC->FCCR = reg;
    /* Set CMD_TYPE to exit continuous read mode */
    reg = (reg & ~FLASHC_FCCR_CMD_TYPE_MASK) | FLASHC_FCCR_CMD_TYPE(0xCU);
    FLASHC->FCCR = reg;
    /* Wait exit done */
    while ((FLASHC->FCSR & FLASHC_FCSR_CONT_RD_MD_EXIT_DONE_MASK) == 0U) {
    }
    /* Clear exit done flag */
    FLASHC->FCSR = FLASHC_FCSR_CONT_RD_MD_EXIT_DONE_MASK;
    /* Set pad mux to QSPI */
    FLASHC->FCCR &= ~FLASHC_FCCR_FLASHC_PAD_EN_MASK;
}

static void init_flashc(void) {
    uint32_t reg = FLASHC->FCCR;

    /* Set pad mux to FLASHC */
    reg |= FLASHC_FCCR_FLASHC_PAD_EN_MASK;
    FLASHC->FCCR = reg;

    /* Set CMD_TYPE to continuous read mode */
    reg = (reg & ~FLASHC_FCCR_CMD_TYPE_MASK) | FLASHC_FCCR_CMD_TYPE(0x6U);
    FLASHC->FCCR = reg;
    /* Enable cache */
    reg |= FLASHC_FCCR_CACHE_EN_MASK;
    FLASHC->FCCR = reg;
}
#endif

static void init_boot_clocks(void) {
    POWER_PowerOnVddioPad(kPOWER_VddIoAon);
    POWER_PowerOnVddioPad(kPOWER_VddIo0);
    POWER_PowerOnVddioPad(kPOWER_VddIo1);
    POWER_PowerOnVddioPad(kPOWER_VddIo2);
    POWER_PowerOnVddioPad(kPOWER_VddIo3);

    /* Wait for VDDIO to be ready */
    volatile uint32_t loop = 0x2000;
    while (loop--) {
        __NOP();
    }

    /* Enable GPIO and UART0 clocks */
    CLOCK_EnableClock(kCLOCK_Gpio);
    CLOCK_EnableClock(kCLOCK_Uart0);

#if IS_RAM_BUILD
    /* Enable RC32M. */
    CLOCK_EnableClock(kCLOCK_Rc32m);
    CLOCK_EnableRC32M(false);

    /* Switch to RC32M before changing SFLL (if boot2 left it at SFLL) */
    CLOCK_SetSysClkSource(kCLOCK_SysClkSrcRC32M_1);

    /* Set the PMU clock divider to 1 */
    CLOCK_SetClkDiv(kCLOCK_DivPmu, 1U);

    /* Set external Xtal frequency to clock driver */
    CLOCK_SetMainXtalFreq(CLK_MAINXTAL_CLK);

    /* Enable System OSC 38.4M. */
    CLOCK_EnableRefClk(kCLOCK_RefClk_SYS);

    /* Initialize SFLL to 200M. */
    CLOCK_InitSFll(&sfll_config);

    /* Set dividers */
    CLOCK_SetClkDiv(kCLOCK_DivApb0, 2U);
    CLOCK_SetClkDiv(kCLOCK_DivApb1, 2U);
    CLOCK_SetClkDiv(kCLOCK_DivPmu, 4U);

    /* Switch system clock source to SFLL before RC32M calibration */
    CLOCK_SetSysClkSource(kCLOCK_SysClkSrcSFll);

    CLOCK_SetClkDiv(kCLOCK_DivQspi, 4U);

    /* Calibrate RC32M */
    CLOCK_CalibrateRC32M(true, 0U);
    /* RC32M enabled, controller clock can be gated. */
    CLOCK_DisableClock(kCLOCK_Rc32m);

    /* Reset the PMU clock divider to 1 */
    CLOCK_SetClkDiv(kCLOCK_DivPmu, 1U);

    /* Restart flash controller (needed for XIP and mflash operations) */
    init_flashc();
#else
    /* For XIP builds, boot2 has already configured the clocks.
     * We just need to inform the clock driver of the XTAL frequency
     * so that CLOCK_GetSysClkFreq() returns the correct value. */
    CLOCK_SetMainXtalFreq(CLK_MAINXTAL_CLK);
#endif

    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
}

static int nxp_88mw320_init(void) {
    init_boot_clocks();

    return 0;
}

SYS_INIT(nxp_88mw320_init, PRE_KERNEL_1, 0);

void sys_arch_reboot(int type) {
    ARG_UNUSED(type);

    __disable_irq();

    /* Switch system clock to RC32M before power-cycling WLAN.
     * MAINXTAL is inside the WLAN power domain, so powering off WLAN
     * kills the SFLL input clock. */
    CLOCK_EnableClock(kCLOCK_Rc32m);
    CLOCK_EnableRC32M(false);
    CLOCK_SetSysClkSource(kCLOCK_SysClkSrcRC32M_1);

    /* Assert WLAN power down to reset radio state machine */
    PMU->WLAN_CTRL = 0;

    for (volatile int i = 0; i < 10000; i++) {
        __NOP();
    }

    /* Power WLAN back on and request REFCLK_SYS (equivalent to chip_fixup mwb
     * 0x480a0118 3) */
    PMU->WLAN_CTRL = PMU_WLAN_CTRL_PD_MASK | PMU_WLAN_CTRL_REFCLK_SYS_REQ_MASK;

    /* Wait for REFCLK_SYS to be ready before resetting CPU */
    while ((PMU->WLAN_CTRL & PMU_WLAN_CTRL_REFCLK_SYS_RDY_MASK) == 0U) {
        __NOP();
    }

    NVIC_SystemReset();
}
