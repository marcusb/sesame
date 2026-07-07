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

## Known Issues

### Critical (boot hang/crash)

1. **`soc/Kconfig` line 2: `invalid_syntax`** — garbage text that may confuse
   the Kconfig parser.

2. **`prj.conf` duplicates** — `CONFIG_GPIO=y`, `CONFIG_CODE_DATA_RELOCATION=y`,
   and `CONFIG_CODE_DATA_RELOCATION_SRAM=y` each appear twice.

3. **UART driver double-pinmux** — `uart_mw320_init()` calls `PINMUX_PinMuxSet`
   twice: first with SDK constants (matching `soc.c`), then with hardcoded magic
   values (`2 | (1 << 3)`). The second call may corrupt the pinmux config set
   by the SOC init.

4. **`memset` in UART driver without `<string.h>`** — `uart_mw320_init()` calls
   `memset()` but doesn't include `<zephyr/string.h>` or `<string.h>`. This may
   compile by luck (transitive include) but is undefined behavior.

5. **Entropy driver hardcodes `DEVICE_DT_INST_DEFINE(0, ...)`** — No DT node
   exists for `nxp,mw320-entropy`, so the driver should use
   `DT_INST_FOREACH_STATUS_OKAY` or be removed. Currently it's excluded by the
   Kconfig dependency on `DT_HAS_NXP_MW320_ENTROPY_ENABLED`, so it's not built.

6. **Flash layout mismatch** — `board/flash-layout.txt` shows app at 0x30000,
   but DTS partitions place `image-0` at 0x74000. `CONFIG_ROM_START_OFFSET=0xC8`
   is a tiny offset that doesn't match either layout.

### Potential root causes for boot hang

- **Clock init infinite loop**: `init_boot_clocks()` has several `while()` loops
  waiting for hardware ready flags (PLL lock, RC32M ready, ref clock ready,
  flash exit continuous mode). If the hardware state differs from expectations,
  these loops hang forever.

- **Flash controller access from non-RAM code**: After `deinit_flashc()`, only
  `__ramfunc` and RAM-relocated code can execute. The `zephyr_code_relocate`
  directive relocates all of `soc.c` to RAM, so this should be safe, but the
  `PINMUX_PinMuxSet` function remains in flash (0x1f002f04) and is called via
  a veneer from RAM.

- **Vector table address**: VTOR is set to 0x1f000200 (flash). If the bootloader
  loads the app into RAM and doesn't adjust VTOR, the CPU may fault on the first
  exception.

## Debugging Boot Hang

### Diagnostic Variable

`boot_diag` (volatile uint32_t) at RAM address `0x100e14`. Check with GDB:

```
(gdb) print/x *(uint32_t*)0x100e14
```

Values indicate how far boot progressed:

| Value | Stage | Description |
|-------|-------|-------------|
| 0x00 | Uninitialized | Hang before SOC init (Zephyr core issue) |
| 0x01 | PMU_PAD | VDDIO pad power-on |
| 0x02 | FLASH_DEINIT | Flash controller deinit (exit continuous mode) |
| 0x03 | REFCLK_SYS | Reference clock SYS ready wait |
| 0x04 | RC32M | RC32M oscillator ready wait |
| 0x05 | REFCLK_OSC | System OSC 38.4M ready wait |
| 0x06 | SFLL | SFLL PLL lock wait |
| 0x07 | SYSCLK | System clock switch to SFLL |
| 0x08 | FLASH_INIT | Flash controller re-init |
| 0x09 | PINMUX | UART pinmux configuration |
| 0x0A | DONE | SOC init complete |
| 0x80+ | Timeout | Stage (lower 7 bits) timed out in hardware ready loop |

### GDB Debug Session

```bash
# Terminal 1: Start OpenOCD
openocd -s tools/OpenOCD -f tools/OpenOCD/interface/ftdi.cfg -f tools/OpenOCD/openocd.cfg

# Terminal 2: GDB
gdb-multiarch -x tools/OpenOCD/gdbinit build/zephyr/zephyr.elf
(gdb) debug                    # Load to RAM, stop at main()
(gdb) print/x *(uint32_t*)0x100e14   # Check boot_diag
(gdb) info registers           # Check CPU state
(gdb) bt                       # Backtrace
```

### Fixed Issues

The following issues were found and fixed in the latest commit:

1. ~~`soc/Kconfig` line 2: `invalid_syntax`~~ — removed garbage text
2. ~~`prj.conf` duplicates~~ — deduplicated CONFIG entries
3. ~~UART driver double pinmux~~ — removed redundant PINMUX_PinMuxSet calls
4. ~~UART driver missing `<string.h>`~~ — added include for `memset`
5. ~~Infinite loops without timeout~~ — added timeout protection with diagnostic markers
