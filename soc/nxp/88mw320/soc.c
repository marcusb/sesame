#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include "fsl_power.h"
#include "fsl_clock.h"
#include "fsl_pinmux.h"
#include "88MW320.h"



// RAM build: flash base is at SRAM1 (0x20000000), skip flash controller ops
#ifndef CONFIG_XIP
#define IS_RAM_BUILD 1
#else
#define IS_RAM_BUILD 0
#endif

#define BOARD_BOOTCLOCKRUN_CORE_CLOCK 200000000U

// Timeout for hardware ready loops (in iterations).
#define BOOT_TIMEOUT 0xFFFFFFFF

static clock_sfll_config_t sfll_config = {
    .sfllSrc = kCLOCK_SFllSrcMainXtal, /* XTAL clock */
    .refDiv = 0x60U,
    .fbDiv = 0xFAU,
    .kvco = 0x1U,
    .postDiv = 0x0U};

static void deinit_flashc(void) {
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
            while (1) { }
        }
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

static void init_boot_clocks(void) {
    uint32_t timeout;

    /* Disable watchdog timer immediately */
    WDT->WDT_CR = 0;

    /* Power on VDDIO pads */
    PMU->IO_PAD_PWR_CFG |= PMU_IO_PAD_PWR_CFG_GPIO_AON_PDB_MASK;

    /* Both pad regulator and IO domain powered on for VddIo0..3 */
    /* PDB bits 0..3, LOW_VDDB bits 12..15 */
    PMU->IO_PAD_PWR_CFG |= ((1UL << 12) | (1UL << 0));
    PMU->IO_PAD_PWR_CFG |= ((1UL << 13) | (1UL << 1));
    PMU->IO_PAD_PWR_CFG |= ((1UL << 14) | (1UL << 2));
    PMU->IO_PAD_PWR_CFG |= ((1UL << 15) | (1UL << 3));

    /* Disable watchdog clock (bit 23, active-low: 1=disabled) */
    PMU->PERI_CLK_EN |= (1UL << 23);

    /* Enable GPIO and UART0 clocks */
    CLOCK_EnableClock(kCLOCK_Gpio);
    CLOCK_EnableClock(kCLOCK_Uart0);

    /* Stop flash controller before changing clock (XIP only, skip for RAM build) */
    if (!IS_RAM_BUILD) {
        deinit_flashc();
    }

    /* Enable ref clock SYS */
    PMU->WLAN_CTRL |= PMU_WLAN_CTRL_PD_MASK;
    PMU->WLAN_CTRL |= (1U << PMU_WLAN_CTRL_REFCLK_SYS_REQ_SHIFT);
    timeout = BOOT_TIMEOUT;
    while ((PMU->WLAN_CTRL & (1U << (PMU_WLAN_CTRL_REFCLK_SYS_REQ_SHIFT + 3U))) == 0U) {
        if (--timeout == 0) {
            while (1) { }
        }
    }

    /* Enable RC32M. */
    CLOCK_EnableClock(kCLOCK_Rc32m);
    CLOCK_EnableRC32M(false);

    /* Set the PMU clock divider to 1 */
    CLOCK_SetClkDiv(kCLOCK_DivPmu, 1U);

    /* Set external Xtal frequency to clock driver */
    g_mainXtalFreq = CLK_MAINXTAL_CLK;

    /* Enable System OSC 38.4M. */
    CLOCK_EnableRefClk(kCLOCK_RefClk_SYS);

    /* Initialize SFLL to 200M. */
    CLOCK_InitSFll(&sfll_config);

    /* Set dividers */
    CLOCK_SetClkDiv(kCLOCK_DivApb0, 2U);
    CLOCK_SetClkDiv(kCLOCK_DivApb1, 2U);
    CLOCK_SetClkDiv(kCLOCK_DivPmu, 4U);

    /* Set UART fast clock divider to 1 (nom=1, denom=1) */
    PMU->UART_FAST_CLK_DIV = (1U << PMU_UART_FAST_CLK_DIV_NOMINATOR_SHIFT) | 1U;

    /* Select UART0 Fast Clock */
    PMU->UART_CLK_SEL |= PMU_UART_CLK_SEL_UART0_CLK_SEL_MASK;

    /* Switch system clock source to SFLL before RC32M calibration */
    CLOCK_SetSysClkSource(kCLOCK_SysClkSrcSFll);

    CLOCK_SetClkDiv(kCLOCK_DivQspi, 4U);

    /* RC32M enabled, controller clock can be gated. */
    CLOCK_DisableClock(kCLOCK_Rc32m);

    /* Re-enable flash controller (XIP only, skip for RAM build) */
    if (!IS_RAM_BUILD) {
        init_flashc();
    }

    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
}

static int nxp_88mw320_init(void)
{
    // Disable watchdog timer immediately
    WDT->WDT_CR = 0;
    init_boot_clocks();

    return 0;
}

SYS_INIT(nxp_88mw320_init, PRE_KERNEL_1, 0);
