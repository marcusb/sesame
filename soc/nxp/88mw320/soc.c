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
#define IS_RAM_BUILD (!DT_NODE_EXISTS(DT_CHOSEN(zephyr_flash)))

#define BOARD_BOOTCLOCKRUN_CORE_CLOCK 200000000U

// Diagnostic: write here before each init stage. Check with debugger if hang.
volatile uint32_t __attribute__((used)) boot_diag __attribute__((section(".data"))) = 0;

#define BOOT_DIAG_STAGE(x) (boot_diag = (x))
#define BOOT_DIAG_PMU_PAD    0x01
#define BOOT_DIAG_FLASH_DEINIT 0x02
#define BOOT_DIAG_REFCLK_SYS   0x03
#define BOOT_DIAG_RC32M        0x04
#define BOOT_DIAG_REFCLK_OSC   0x05
#define BOOT_DIAG_SFLL         0x06
#define BOOT_DIAG_SYSCLK       0x07
#define BOOT_DIAG_FLASH_INIT   0x08
#define BOOT_DIAG_PINMUX       0x09
#define BOOT_DIAG_DONE         0x0A

// Timeout for hardware ready loops (in iterations). Set diag to 0x80+stage on timeout.
#define BOOT_TIMEOUT 0xFFFFFFFF
#define BOOT_TIMEOUT_FLAG 0x80

static clock_sfll_config_t sfll_config = {
    .sfllSrc = kCLOCK_SFllSrcMainXtal, /* XTAL clock */
    .refDiv = 0x60U,
    .fbDiv = 0xFAU,
    .kvco = 0x1U,
    .postDiv = 0x0U};

__ramfunc static void deinit_flashc(void) {
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
            while (1) { }
        }
    }
    /* Clear exit done flag */
    FLASHC->FCSR = FLASHC_FCSR_CONT_RD_MD_EXIT_DONE_MASK;
    /* Set pad mux to QSPI */
    FLASHC->FCCR &= ~FLASHC_FCCR_FLASHC_PAD_EN_MASK;
}

__ramfunc static void init_flashc(void) {
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

__ramfunc static void ram_CLOCK_EnableClock(clock_ip_name_t name)
{
    uint32_t pos   = CLK_OP_POS(name);
    uint32_t mask  = CLK_OP_MASK(name);
    uint32_t value = CLK_OP_VALUE(name);

    CLK_OP_REG(name) = (CLK_OP_REG(name) & ~(mask << pos)) | (value << pos);
}
__ramfunc static void ram_CLOCK_DisableClock(clock_ip_name_t name)
{
    uint32_t pos   = CLK_OP_POS(name);
    uint32_t mask  = CLK_OP_MASK(name);
    uint32_t value = (~CLK_OP_VALUE(name)) & mask;

    CLK_OP_REG(name) = (CLK_OP_REG(name) & ~(mask << pos)) | (value << pos);
}
__ramfunc static void ram_CLOCK_SetClkDiv(clock_div_name_t name, uint32_t divider)
{
    uint32_t pos   = CLK_OP_POS(name);
    uint32_t mask  = CLK_OP_MASK(name);
    uint32_t value = CLK_OP_VALUE(name);

    if (value == 0U)
    {
        CLK_OP_REG(name) = (CLK_OP_REG(name) & ~(mask << pos)) | (divider << pos);
    }
    else
    {
        switch (name)
        {
            case kCLOCK_DivUartFast:
            case kCLOCK_DivUartSlow:
                CLK_OP_REG(name) = divider;
                break;
            case kCLOCK_DivWdt:
                CLK_OP_REG(name) &= ~(PMU_PERI2_CLK_DIV_WDT_CLK_DIV_5_3_MASK | PMU_PERI2_CLK_DIV_WDT_CLK_DIV_2_2_MASK |
                                      PMU_PERI2_CLK_DIV_WDT_CLK_DIV_1_0_MASK);
                CLK_OP_REG(name) |= ((divider >> 3U) << PMU_PERI2_CLK_DIV_WDT_CLK_DIV_5_3_SHIFT) |
                                    (((divider >> 2U) & 1U) << PMU_PERI2_CLK_DIV_WDT_CLK_DIV_2_2_SHIFT) |
                                    ((divider & 3U) << PMU_PERI2_CLK_DIV_WDT_CLK_DIV_1_0_SHIFT);
                break;
            case kCLOCK_DivGpt3:
                CLK_OP_REG(name) &=
                    ~(PMU_PERI2_CLK_DIV_GPT3_CLK_DIV_5_3_MASK | PMU_PERI2_CLK_DIV_GPT3_CLK_DIV_2_0_MASK);
                CLK_OP_REG(name) |= ((divider >> 3U) << PMU_PERI2_CLK_DIV_GPT3_CLK_DIV_5_3_SHIFT) |
                                    ((divider & 7U) << PMU_PERI2_CLK_DIV_GPT3_CLK_DIV_2_0_SHIFT);
                break;
            default:
                break;
        }
    }
}
__ramfunc static void ram_CLOCK_SetSysClkSource(clock_sys_clk_src_t src)
{
    uint32_t currSrc;
    bool rc32mRdy;

    currSrc  = PMU->CLK_SRC & PMU_CLK_SRC_SYS_CLK_SEL_MASK;
    rc32mRdy = (PMU->CLK_RDY & PMU_CLK_RDY_RC32M_RDY_MASK) != 0U;

    switch (src)
    {
        case kCLOCK_SysClkSrcSFll:
            if ((currSrc == (uint32_t)kCLOCK_SysClkSrcMainXtal) && rc32mRdy)
            {
                PMU->CLK_SRC =
                    (PMU->CLK_SRC & ~PMU_CLK_SRC_SYS_CLK_SEL_MASK) | PMU_CLK_SRC_SYS_CLK_SEL(kCLOCK_SysClkSrcRC32M_3);
                currSrc = (uint32_t)kCLOCK_SysClkSrcRC32M_3;
            }
            if (currSrc == (uint32_t)kCLOCK_SysClkSrcRC32M_3)
            {
                PMU->CLK_SRC =
                    (PMU->CLK_SRC & ~PMU_CLK_SRC_SYS_CLK_SEL_MASK) | PMU_CLK_SRC_SYS_CLK_SEL(kCLOCK_SysClkSrcRC32M_1);
            }
            PMU->CLK_SRC =
                (PMU->CLK_SRC & ~PMU_CLK_SRC_SYS_CLK_SEL_MASK) | PMU_CLK_SRC_SYS_CLK_SEL(kCLOCK_SysClkSrcSFll);
            break;
        case kCLOCK_SysClkSrcRC32M_1:
        case kCLOCK_SysClkSrcRC32M_3:
            if ((currSrc == (uint32_t)kCLOCK_SysClkSrcSFll) || (currSrc == (uint32_t)kCLOCK_SysClkSrcMainXtal))
            {
                PMU->CLK_SRC = (PMU->CLK_SRC & ~PMU_CLK_SRC_SYS_CLK_SEL_MASK) | PMU_CLK_SRC_SYS_CLK_SEL(currSrc + 1U);
            }
            break;
        case kCLOCK_SysClkSrcMainXtal:
            if ((currSrc == (uint32_t)kCLOCK_SysClkSrcSFll) && rc32mRdy)
            {
                PMU->CLK_SRC =
                    (PMU->CLK_SRC & ~PMU_CLK_SRC_SYS_CLK_SEL_MASK) | PMU_CLK_SRC_SYS_CLK_SEL(kCLOCK_SysClkSrcRC32M_1);
                currSrc = (uint32_t)kCLOCK_SysClkSrcRC32M_1;
            }
            if (currSrc == (uint32_t)kCLOCK_SysClkSrcRC32M_1)
            {
                PMU->CLK_SRC =
                    (PMU->CLK_SRC & ~PMU_CLK_SRC_SYS_CLK_SEL_MASK) | PMU_CLK_SRC_SYS_CLK_SEL(kCLOCK_SysClkSrcRC32M_3);
            }
            PMU->CLK_SRC =
                (PMU->CLK_SRC & ~PMU_CLK_SRC_SYS_CLK_SEL_MASK) | PMU_CLK_SRC_SYS_CLK_SEL(kCLOCK_SysClkSrcMainXtal);
            break;
        default:
            break;
    }
}
__ramfunc static void ram_CLOCK_EnableRC32M(bool halfDiv)
{
    uint32_t timeout;
    RC32->CLK = (RC32->CLK & ~RC32_CLK_REF_SEL_MASK) | RC32_CLK_REF_SEL(halfDiv ? 0U : 1U);
    RC32->CTRL &= ~RC32_CTRL_PD_MASK;
    RC32->CTRL |= RC32_CTRL_EN_MASK;
    timeout = BOOT_TIMEOUT;
    while ((PMU->CLK_RDY & PMU_CLK_RDY_RC32M_RDY_MASK) == 0U) {
        if (--timeout == 0) {
            boot_diag = BOOT_TIMEOUT_FLAG | BOOT_DIAG_RC32M;
            while (1) { }
        }
    }
}
__ramfunc static void ram_CLOCK_EnableRefClk(uint32_t refclks)
{
    uint32_t timeout;
    /* Ensure WLAN is powered on */
    PMU->WLAN_CTRL |= PMU_WLAN_CTRL_PD_MASK;
    /* Enable reference clock request */
    PMU->WLAN_CTRL |= refclks;
    /* Wait reference clock ready */
    timeout = BOOT_TIMEOUT;
    while ((PMU->WLAN_CTRL & (refclks << 3U)) != (refclks << 3U)) {
        if (--timeout == 0) {
            boot_diag = BOOT_TIMEOUT_FLAG | BOOT_DIAG_REFCLK_OSC;
            while (1) { }
        }
    }
}
__ramfunc static void ram_CLOCK_InitSFll(const clock_sfll_config_t *config)
{
    uint32_t timeout;
    PMU->SFLL_CTRL0 = (PMU->SFLL_CTRL0 & ~(PMU_SFLL_CTRL0_SFLL_FBDIV_MASK | PMU_SFLL_CTRL0_SFLL_KVCO_MASK |
                                           PMU_SFLL_CTRL0_SFLL_REFCLK_SEL_MASK)) |
                      PMU_SFLL_CTRL0_SFLL_FBDIV(config->fbDiv) | PMU_SFLL_CTRL0_SFLL_KVCO(config->kvco) |
                      PMU_SFLL_CTRL0_SFLL_REFCLK_SEL(config->sfllSrc);
    PMU->SFLL_CTRL1 = (PMU->SFLL_CTRL1 & ~(PMU_SFLL_CTRL1_SFLL_REFDIV_MASK | PMU_SFLL_CTRL1_SFLL_DIV_SEL_MASK)) |
                      PMU_SFLL_CTRL1_SFLL_REFDIV(config->refDiv) | PMU_SFLL_CTRL1_SFLL_DIV_SEL(config->postDiv);

    /* Power up */
    PMU->SFLL_CTRL0 |= PMU_SFLL_CTRL0_SFLL_PU_MASK;

    /* Wait PLL ready */
    timeout = BOOT_TIMEOUT;
    while ((PMU->CLK_RDY & PMU_CLK_RDY_PLL_CLK_RDY_MASK) == 0U) {
        if (--timeout == 0) {
            boot_diag = BOOT_TIMEOUT_FLAG | BOOT_DIAG_SFLL;
            while (1) { }
        }
    }
}
__ramfunc static void init_boot_clocks(void) {
    uint32_t timeout;

    /* Disable watchdog timer immediately */
    WDT->WDT_CR = 0;

    /* Power on VDDIO pads */
    BOOT_DIAG_STAGE(BOOT_DIAG_PMU_PAD);
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
    ram_CLOCK_EnableClock(kCLOCK_Gpio);
    ram_CLOCK_EnableClock(kCLOCK_Uart0);

    /* Stop flash controller before changing clock (XIP only, skip for RAM build) */
    if (!IS_RAM_BUILD) {
        BOOT_DIAG_STAGE(BOOT_DIAG_FLASH_DEINIT);
        deinit_flashc();
    }

    /* Enable ref clock SYS */
    BOOT_DIAG_STAGE(BOOT_DIAG_REFCLK_SYS);
    PMU->WLAN_CTRL |= PMU_WLAN_CTRL_PD_MASK;
    PMU->WLAN_CTRL |= (1U << PMU_WLAN_CTRL_REFCLK_SYS_REQ_SHIFT);
    timeout = BOOT_TIMEOUT;
    while ((PMU->WLAN_CTRL & (1U << (PMU_WLAN_CTRL_REFCLK_SYS_REQ_SHIFT + 3U))) == 0U) {
        if (--timeout == 0) {
            boot_diag = BOOT_TIMEOUT_FLAG | BOOT_DIAG_REFCLK_SYS;
            while (1) { }
        }
    }

    /* Enable RC32M. */
    BOOT_DIAG_STAGE(BOOT_DIAG_RC32M);
    ram_CLOCK_EnableClock(kCLOCK_Rc32m);
    ram_CLOCK_EnableRC32M(false);

    /* Set the PMU clock divider to 1 */
    ram_CLOCK_SetClkDiv(kCLOCK_DivPmu, 1U);

    /* Set external Xtal frequency to clock driver */
    g_mainXtalFreq = CLK_MAINXTAL_CLK;

    /* Enable System OSC 38.4M. */
    BOOT_DIAG_STAGE(BOOT_DIAG_REFCLK_OSC);
    ram_CLOCK_EnableRefClk(kCLOCK_RefClk_SYS);

    /* Initialize SFLL to 200M. */
    BOOT_DIAG_STAGE(BOOT_DIAG_SFLL);
    ram_CLOCK_InitSFll(&sfll_config);

    /* Set dividers */
    ram_CLOCK_SetClkDiv(kCLOCK_DivApb0, 2U);
    ram_CLOCK_SetClkDiv(kCLOCK_DivApb1, 2U);
    ram_CLOCK_SetClkDiv(kCLOCK_DivPmu, 4U);

    /* Set UART fast clock divider to 1 (nom=1, denom=1) */
    PMU->UART_FAST_CLK_DIV = (1U << PMU_UART_FAST_CLK_DIV_NOMINATOR_SHIFT) | 1U;

    /* Select UART0 Fast Clock */
    PMU->UART_CLK_SEL |= PMU_UART_CLK_SEL_UART0_CLK_SEL_MASK;

    /* Switch system clock source to SFLL before RC32M calibration */
    BOOT_DIAG_STAGE(BOOT_DIAG_SYSCLK);
    ram_CLOCK_SetSysClkSource(kCLOCK_SysClkSrcSFll);

    ram_CLOCK_SetClkDiv(kCLOCK_DivQspi, 4U);

    /* RC32M enabled, controller clock can be gated. */
    ram_CLOCK_DisableClock(kCLOCK_Rc32m);

    /* Re-enable flash controller (XIP only, skip for RAM build) */
    if (!IS_RAM_BUILD) {
        BOOT_DIAG_STAGE(BOOT_DIAG_FLASH_INIT);
        init_flashc();
    }

    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
    BOOT_DIAG_STAGE(0x0B); // DIAG: clocks done
}

static int nxp_88mw320_init(void)
{
    BOOT_DIAG_STAGE(0x01); // DIAG: SOC init entry
    // Disable watchdog timer immediately
    WDT->WDT_CR = 0;
    init_boot_clocks();
    BOOT_DIAG_STAGE(BOOT_DIAG_PINMUX);
    CLOCK_EnableClock(kCLOCK_Gpio);
    PINMUX_PinMuxSet(2, PINMUX_GPIO2_UART0_TXD | PINMUX_MODE_DEFAULT);
    CLOCK_EnableClock(kCLOCK_Gpio);
    PINMUX_PinMuxSet(3, PINMUX_GPIO3_UART0_RXD | PINMUX_MODE_DEFAULT);
    CLOCK_AttachClk(kSYS_CLK_to_FAST_UART0);
    CLOCK_EnableClock(kCLOCK_Uart0);

    /* Mux LED pins to GPIO function */
    PINMUX_PinMuxSet(16, PINMUX_GPIO16_GPIO16 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(27, PINMUX_GPIO27_GPIO27 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(40, PINMUX_GPIO40_GPIO40 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(41, PINMUX_GPIO41_GPIO41 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(42, PINMUX_GPIO42_GPIO42 | PINMUX_MODE_DEFAULT);
    PINMUX_PinMuxSet(43, PINMUX_GPIO43_GPIO43 | PINMUX_MODE_DEFAULT);

    BOOT_DIAG_STAGE(BOOT_DIAG_DONE);
    return 0;
}


SYS_INIT(nxp_88mw320_init, PRE_KERNEL_1, 0);
