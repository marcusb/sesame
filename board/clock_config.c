#include "clock_config.h"

#include "board.h"
#include "fsl_clock.h"
#include "fsl_power.h"

#define BOARD_XTAL_SYS_CLK_HZ 384000000U
#define BOARD_BOOTCLOCKRUN_CORE_CLOCK 200000000U

extern uint32_t SystemCoreClock;

static const clock_sfll_config_t sfll_config = {
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

void init_boot_clocks(void) {
    POWER_PowerOnVddioPad(kPOWER_VddIoAon);
    POWER_PowerOnVddioPad(kPOWER_VddIo0);
    POWER_PowerOnVddioPad(kPOWER_VddIo1);
    POWER_PowerOnVddioPad(kPOWER_VddIo2);
    POWER_PowerOnVddioPad(kPOWER_VddIo3);

    if (BOARD_IS_XIP) {
        /* Stop flash controller before changing clock */
        deinit_flashc();
    }
    /* Enable RC32M. */
    CLOCK_EnableClock(kCLOCK_Rc32m);
    CLOCK_EnableRC32M(false);

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

    if (BOARD_IS_XIP) {
        /* Restart flash controller for XIP */
        init_flashc();
    }

    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
}
