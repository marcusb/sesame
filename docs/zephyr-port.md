# Zephyr Port Status

## Build System

The project is a Zephyr standalone application that uses the Zephyr build system
via `west`. The app lives inside the Zephyr tree as a external application.

### Structure

```
sesame/
├── CMakeLists.txt              # App-level CMake: sources, includes
├── prj.conf                    # App Kconfig options
├── west.yml                    # West manifest (zephyr module at main)
├── zephyr/
│   ├── CMakeLists.txt          # Adds drivers/ subdirectory
│   ├── Kconfig                 # App-level Kconfig (empty stub)
│   └── module.yml              # Declares board_root, soc_root, dts_root
├── boards/arm/marvell_mw302/   # Board definition
│   ├── board.yml               # Board → SOC mapping (88mw320)
│   ├── Kconfig.marvell_mw302   # BOARD_MARVELL_MW302, BOARD_MARVELL_MW302_GENIE_IDCM
│   ├── Kconfig.defconfig       # Board name defaults
│   ├── CMakeLists.txt          # Empty (no board-specific sources)
│   ├── marvell_mw302_88mw320_cpu0.dts    # Base board DTS
│   └── marvell_mw302_genie_idcm.dts      # Genie IDCM overlay DTS
├── soc/nxp/88mw320/            # SOC definition
│   ├── soc.yml                 # SOC metadata (nxp vendor, 88mw320 family)
│   ├── Kconfig.soc             # SOC_88MW320: Cortex-M4F, FPU, MPU, DWT, SysTick
│   ├── Kconfig.defconfig       # NUM_IRQS=48, SYS_CLOCK_HW_CYCLES_PER_SEC=200MHz
│   ├── CMakeLists.txt          # soc.c source, linker script, code relocation
│   ├── 88mw320.dtsi            # SOC DT: CPU, flash, SRAM0/1, UART0, GPIO0/1
│   ├── soc.c                   # SOC init: clocks, flash controller, pinmux
│   └── soc.h                   # SOC header (empty stub)
├── drivers/                    # Custom Zephyr drivers
│   ├── CMakeLists.txt          # Adds gpio/, serial/, entropy/, wifi/mw320/
│   ├── Kconfig                 # Rsources all driver Kconfigs
│   ├── gpio/gpio_mw320.c       # GPIO driver wrapping fsl_gpio.h
│   ├── serial/uart_mw320.c     # UART driver wrapping fsl_uart.h
│   ├── entropy/entropy_mw320.c # Entropy driver (stub, no DT node)
│   └── wifi/mw320/wifi_mw320.c # WiFi L2 driver (stub)
├── dts/bindings/              # DT bindings
│   ├── gpio/nxp,mw320-gpio.yaml
│   └── serial/nxp,mw320-uart.yaml
├── modules/mw320_sdk/zephyr/   # MW320 SDK as Zephyr module
│   ├── CMakeLists.txt          # SDK driver sources + includes
│   ├── Kconfig                 # Module Kconfig
│   └── module.yml              # Module metadata
├── src/main.c                  # App: printk loop + k_msleep
└── app.overlay                 # Board overlay DTS (LED, flash partition)
```

### Build Command

```sh
west build -b marvell_mw302/88mw320/cpu0 \
  -- -DZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb -DGNUARMEMB_TOOLCHAIN_PATH=/usr
```

Board target format: `board_name/soc_name/cpu_cluster`
- `marvell_mw302` — board
- `88mw320` — SOC
- `cpu0` — CPU cluster

### Module Discovery

`zephyr/module.yml` sets `board_root: .`, `soc_root: .`, `dts_root: .`,
so Zephyr finds the custom board/SOC/DT bindings in the app directory.

`modules/mw320_sdk/zephyr/module.yml` exposes the MW320 SDK drivers as a
Zephyr module. The SDK CMakeLists.txt pulls in `fsl_*.c` driver sources and
relocates `fsl_clock.c` to RAM.

### Memory Layout

| Region     | Address    | Size     | Purpose                              |
|------------|------------|----------|--------------------------------------|
| FLASH      | 0x1f0000c8 | ~8 MB    | Code + RO data (ROM start offset 0xC8)|
| SRAM0      | 0x00100000 | 380 KB   | RAM-relocated code, .data, .bss, heap|
| SRAM1      | 0x20000000 | 128 KB   | Reserved (unused)                    |

RAM-relocated code (`.ram_text_reloc` @ 0x100000):
- All of `soc.c` (via `zephyr_code_relocate(FILES soc.c LOCATION RAM)`)
- All of `fsl_clock.c` (via `zephyr_code_relocate` in mw320_sdk module)

`__ramfunc` functions (`.ramfunc` @ 0x100b28):
- `deinit_flashc`, `init_flashc`, `ram_CLOCK_*` helpers from `soc.c`

### Init Sequence

| Priority        | Init Function                    | Purpose                                  |
|-----------------|----------------------------------|------------------------------------------|
| PRE_KERNEL_1/0  | `nxp_88mw320_init`               | Clocks, flash ctrl, UART pinmux          |
| PRE_KERNEL_1/N  | `gpio_mw320_init` (gpio0, gpio1) | GPIO driver init (no-op)                 |
| PRE_KERNEL_1/N  | `uart_mw320_init`                | UART driver init (clock, pinmux, baud)   |
| PRE_KERNEL_1/N  | `uart_console_init`              | Console → UART binding                   |
| PRE_KERNEL_2    | `sys_clock_driver_init`          | SysTick timer init                       |
| POST_KERNEL     | `enable_logger`                  | Zephyr log subsystem                     |
| POST_KERNEL     | `malloc_prepare`                 | Heap initialization                      |
| APPLICATION     | `boot_banner`                    | Prints Zephyr boot banner                |
| APPLICATION     | `main()`                         | App entry: printk loop                   |

### MW320 SDK Integration

The SDK provides hardware driver wrappers (`fsl_clock.h`, `fsl_uart.h`,
`fsl_gpio.h`, `fsl_pinmux.h`, `fsl_power.h`). These are compiled as a
Zephyr library and linked into the app. Key:
- `fsl_clock.c` is relocated to RAM (clock registers must be accessed from RAM)
- `system_88MW320.c` provides `SystemCoreClock` global and `SystemInit()`
- SDK headers are included via `zephyr_include_directories`

### Device Tree

Compiled DT: `marvell_mw302_88mw320_cpu0.dts` → includes `88mw320.dtsi`
- UART0 @ 0x46040000, status=okay, 115200 baud
- GPIO0 @ 0x46060000, port=0, 32 pins, status=okay
- GPIO1 @ 0x46060000, port=1, 18 pins, status=okay
- Flash @ 0x1f000000, 8 MB with partition table
- SRAM0 @ 0x00100000, 380 KB (chosen as zephyr,sram)

## Current Status

**XIP Boot**: ✅ Working
**RAM Boot**: ✅ Working
**Serial Console**: ✅ Working

### Resolved Issues

#### Flash (XIP) Boot Hang at `UART_GetStatusFlags`
When executing the XIP build, the device would hang indefinitely without printing the Zephyr boot banner. 
Through GDB and hardware register inspection, it was discovered that `UART_GetStatusFlags()` was reading `0x00000000` from the UART `LSR` register and looping forever waiting for the `THRE` (Transmit Holding Register Empty) flag to be set.

**Root Cause:**
The `boot2` bootloader handles all necessary hardware initialization before jumping to the application:
1. Configures the System clock to use the SFLL at 192MHz.
2. Turns on the UART APB clock gates and sets the fractional dividers so the baudrate hits exactly 115200.
3. Configures the GPIO pin muxing for the UART pads.
4. Outputs its own boot logs (proving the UART is active and transmitting).

When Zephyr started up, its `soc.c` (`init_boot_clocks`, `board_init_pins`) and `uart_mw320.c` (`uart_mw320_init`) were unconditionally attempting to re-initialize these components. Re-initializing the clock source, APB dividers, or pin muxing while executing from flash clobbered the state left by `boot2`, effectively disabling the UART APB clock and clearing the `IER`/`FCR` registers. As a result, reads to the UART `LSR` returned `0x00`, and `uart_mw320_poll_out` hung infinitely.

**Fix:**
Hardware re-initialization is now skipped for XIP builds. Conditional blocks (`#if !DT_NODE_EXISTS(DT_CHOSEN(zephyr_flash))`) were added to:
- `init_boot_clocks()` in `soc.c`
- `board_init_pins()` in `soc.c`
- `uart_mw320_init()` in `uart_mw320.c`

For XIP builds, the application inherits the pristine environment created by `boot2`. For RAM builds (`IS_RAM_BUILD` / no `zephyr_flash`), initialization is preserved as `boot2` is bypassed during RAM debugging.
