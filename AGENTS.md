## Project Overview

Sesame is custom firmware for the Genie 1155 garage door opener (MW300 SoC). It restores network control via MQTT and HTTP, replacing the defunct Aladdin Connect app. The firmware:

- Runs on a Marvell MW320 module with ARM Cortex-M4 processor
- Uses Zephyr RTOS
- Provides HTTP, MQTT and Matter APIs for remote control
- Supports over-the-air (OTA) firmware updates with A/B partition scheme

See docs/teardown.md for hardware information.
See docs/development.md for developer instructions.

## Build System

The project uses the standard Zephyr sysbuild system to orchestrate multi-image builds natively for flash variants.

### Sysbuild Setup

Sesame relies on Zephyr's `sysbuild` to coordinate the building of both the primary application (`sesame`) and the bootloader (`mcuboot`) in a single step.

- **`sysbuild.conf`**: Configures the overall sysbuild environment, telling it to build MCUboot alongside the main application.
- **`sysbuild/mcuboot.conf`**: Contains the Kconfig overrides for the MCUboot image (e.g., enabling Direct-XIP, setting partition sizes, configuring logging).
- **`sysbuild/mcuboot.overlay`**: Hardware devicetree overlay for the MCUboot bootloader image (flash partitions, GPIO LED hogs, retention RAM).
- **`sysbuild/native_sim.conf`**: Kconfig overrides for the native simulation domain (`native_sim`).
- **`sysbuild/sesame_test.conf`**: Kconfig overrides for the on-device system test firmware (`sesame_test`).
- **`sysbuild/sesame_test.overlay`**: Devicetree overlay for the on-device system test firmware (`sesame_test`, configuring simulator flash for storage partition).
- **`Kconfig.sysbuild`**: Sysbuild Kconfig definitions, including `BUILD_NATIVE_SIM` (default `y`).
- **`sysbuild.cmake`**: Sysbuild multi-image script adding `native_sim` as an additional domain when `BUILD_NATIVE_SIM` is enabled.
- **`mcuboot_module/`**: Contains a custom CMakeLists file integrated via sysbuild that handles compiling the MW320-specific flash and pinctrl drivers into MCUboot, as well as running the `axf2firmware` post-build tool to convert MCUboot's `.elf` into the required `mcuboot.bin`.

### Boot and flash layout

Flash layout is in boards/arm/marvell_mw302/flash-layout.txt.

The boot loader chain:
* MW320 boot ROM
* boot2 boot loader from the MW320 SDK
* mcuboot
* sesame application

### Prerequisites

**Debian/Ubuntu:**

```sh
apt install cmake ninja-build protobuf-compiler python3-protobuf openocd qemu-system-arm python3-venv
```

Additionally, you will need the [Zephyr SDK](https://github.com/zephyrproject-rtos/sdk-ng/releases) installed (e.g. in `~/zephyr-sdk`).

### Bootstrapping the Project

This project uses a self-contained West workspace topology (T2).

1. **Clone the repository and initialize submodules:**
   ```sh
   git clone https://github.com/marcusb/sesame.git
   cd sesame
   
   # Initialize all submodules recursively
   git submodule update --init --recursive
   ```

2. **Set up the Python Environment & Matter Bootstrap:**
   We build Matter within Zephyr, and they both expect a Python environment. To avoid managing two separate virtual environments and dealing with import errors, we use Matter's Pigweed environment and install Zephyr's tools directly into it.

   ```sh
   # 1. Bootstrap the Matter (CHIP) environment (downloads gn, ninja, zap, and python)
   source third_party/connectedhomeip/scripts/activate.sh
   
   # 2. Install Zephyr's dependencies into the Pigweed environment
   pip install west pyserial protobuf python-path
   pip install -r deps/zephyr/scripts/requirements.txt
   pip install -r bootloader/mcuboot/scripts/requirements.txt
   
   # Python 3.14 compatibility fix for Pigweed's pinned older typing_extensions
   pip install --upgrade typing_extensions
   ```
   *Note: Pigweed treats its environment as ephemeral. If you update Matter and it re-bootstraps, you may need to re-run the `pip install` step if `west` becomes unavailable.*

3. **Initialize and update the West workspace:**
   ```sh
   west init -l .
   west update
   ```
   *Note: Zephyr modules are placed in `./deps/` to prevent clashing with the local `./modules/`.

### Code Generation (ZAP / Matter IDL)

Matter uses ZCL Advanced Platform (ZAP) to define clusters and endpoints. 
The configuration is stored in `src/matter/window-app.zap` and its corresponding IDL format `src/matter/window-app.matter`.

If you need to change cluster configurations (e.g. adding a new endpoint or feature):
1. Modify `window-app.zap` using the ZAP UI:
   ```sh
   third_party/connectedhomeip/.environment/cipd/packages/zap/zap-cli       -z third_party/connectedhomeip/src/app/zap-templates/zcl/zcl.json       src/matter/window-app.zap
   ```
2. Generate the updated `window-app.matter` file from your changes:
   ```sh
   third_party/connectedhomeip/scripts/tools/zap/generate.py       src/matter/window-app.zap       -o src/matter/
   ```
3. Rebuild the project. The build system will automatically invoke `codegen.py` to generate the updated C++ static cluster configurations from `window-app.matter`. Note that sometimes generating `.matter` files adds explicit `handle command xxxResponse` lines; if you get `duplicate case value` errors in `Groups.h` or similar, ensure you remove `handle command xxxResponse` lines from the `.matter` file.

### Build Commands

**Clean build:**

*Ensure you have activated the environment (see Bootstrapping above) before building.*
```sh
source third_party/connectedhomeip/scripts/activate.sh

rm -rf build && west build --sysbuild
```

**Incremental build:**

```sh
ninja -C build                     # Build all variants (hardware)
ninja -C build sesame/zephyr/zephyr.elf   # Build only hardware flash version
```

**Build outputs:**

- `build/sesame/zephyr/zephyr.signed.bin`, `build/mcuboot/zephyr/mcuboot.bin` – Hardware flash versions
- `build/native_sim/zephyr/zephyr.exe` – Native simulation executable for integration tests
- `build/sesame_test/zephyr/zephyr.signed.bin` – on-device system test binary

## Project Architecture

### App framework and management

1. **System startup** (`main.c`): Initializes generic RTOS scheduler and app tasks. Hardware-specific initialization is handled inline via preprocessor guards (e.g. `CONFIG_SOC_88MW320` vs `native_sim`).
2. **Network stack** (`network.c`): Manages WiFi on hardware. `native_sim` uses direct Ethernet initialization.
3. **Configuration** (`config_manager.c`): Reads/writes protobuf config via Zephyr Settings.
4. **Control interfaces**:
   - HTTP server (`httpd.c`) for REST API and device setup
   - MQTT agent (`mqtt.c`) for pub/sub commands and door status

### Key Components

| Component                | File(s)                 | Purpose  |
| ------------------------ | ----------------------- | -------------------- |
| **App startup**          | `main.c`                | Initializes generic app tasks, Zephyr OS                             |
| **Network Manager**      | `network.c`     | WiFi state machine, IP configuration (DHCP), hardware only   |
| **HTTP Server**          | `httpd.c`               | Receives config and OTA requests via REST, protobuf payloads                |
| **MQTT**                 | `mqtt.c`                | MQTT pub/sub, topic structure, reconnection logic                 |
| **Config Manager**       | `config_manager.c`      | Read/write AppConfig (network, MQTT, logging) stored in Settings         |
| **OTA**                  | `ota.c`, `ota_client.c` | Firmware download, partition management, hardware only              |
| **LEDs & Buttons**       | `leds.c`      | Status indicators, user input handling, hardware only               |
| **PIC comms**            | `pic_uart.c`            | Serial I/O with the PIC16 for door control and status, hardware only|
| **Logging**              | `sesame_syslog.c` | syslog facility                               |
| **Board-specific files** | `boards/arm/marvell_mw302/*`               | Flash layout, board config, ld scripts                             |

### Task Hierarchy (Zephyr)

- **Main thread** – Initialization, runs in idle; calls application task starts
- **System workqueue** – Handles delayed works (e.g. WiFi reconnect)
- **MQTT Agent thread** – Maintains MQTT connection, publishes door status
- **OTA thread** – Handles firmware download and flashing
- **HTTP server** runs in its own Zephyr socket thread

### Data Flow Example: Open Door via MQTT

```
1. MQTT message arrives → MQTT received
2. MQTT listener publishes to ctrl_queue
3. Main task consumes ctrl_queue → enqueues command on pic_queue
4. pic_uart actuates door relay, status updated
5. MQTT publishes new state to broker
```

### Memory & Storage

- **Heap**: Two regions (SRAM0 for small allocations, main SRAM for larger)
- **Flash partitions** (managed by PSM):
  - Partition table, boot2, mcuboot, WiFi firmware (via OpenOCD at first flash)
  - image-0 app firmware partition
  - image-1 app firmware partition (alternate active/passive for OTA)
  - storage partition (AppConfig and Matter fabric state)

### Configuration Format

Configuration is **protobuf-encoded**, not JSON. See `proto/app_config.proto` for schema:

- `NetworkConfig` – hostname, SSID, WiFi security, password
- `MqttConfig` – broker host/port, credentials, topic prefix, TLS flag
- `LoggingConfig` – syslog target (optional)

Example setup via curl (from README):

```sh
echo 'hostname: "sesame", ssid: "MY_WIFI", security: 2, password: "pass"' \
  | protoc --encode=NetworkConfig proto/app_config.proto \
  | curl --data-binary @- -H content-type:application/protobuf \
  'http://192.168.4.1/cfg/network'
```

## Development Workflow

### Development Loop (Rapid Iteration)

**Two-terminal workflow** for fast iteration:

**Terminal 1** – Build, flash and reboot:
```sh
ninja -C build sesame/zephyr/zephyr.elf && ./tools/OpenOCD/flashprog.py --image-0 build/sesame/zephyr/zephyr.signed.bin -r
```

**Terminal 2** – Monitor serial output:
```sh
./tools/monitor.py
```

Typical feature development cycle:
1. Understand affected modules and check proto definitions
2. Implement code changes
3. Re-run Terminal 1 command
4. Check Terminal 2 for output (device will reboot automatically)
5. Iterate until feature works

### Final Validation (Before Commit)

Before committing changes, run unit, integration and system tests.

**1. Run Unit & Integration Tests in native_sim:**

Build and run the tests:
```sh
# unit tests (Twister)
./run_tests.sh unit
# integration tests (native_sim host)
./run_tests.sh integration
# system tests (on-device sesame_test binary)
./run_tests.sh system /dev/ttyUSB0
```

Or run all the tests with:
```
./run_tests.sh all /dev/ttyUSB0
```

**IMPORTANT:** Never use `-l boards/arm/marvell_mw302/flash-layout.txt` with `flashprog.py` during development.
The `-l` flag erases and re-partitions the entire flash, including Boot2 and WiFi firmware.
Use `--mcuboot ... --image-0 ...` alone to flash only the bootloader and application partition.
The `-l` flag is only needed for initial device provisioning (first-time install).

### Logs & Debugging

**Log output** – All operations logged via `LOG_*` macros (see `include/app_logging.h`):
- `LOG_ERROR(msg)`, `LOG_WARN(msg)`, `LOG_INFO(msg)`, `LOG_DEBUG(msg)`
- Sent to console, circular buffer, and optional syslog
- Includes task name and timestamp

**Stack traces and Core Dumps**

If a kernel panic occurs, Zephyr will dump a hex core block. See `docs/development.md` under "Debugging Kernel Panics" for instructions on how to parse this into a C++ stack trace using GDB.

**One-shot flash + capture** – `tools/flash_and_monitor.sh` starts `monitor.py` in the background, runs `flashprog.py --mcuboot build/mcuboot/zephyr/mcuboot.bin --image-0 build/sesame/zephyr/zephyr.signed.bin -r`, and writes the serial output to a log file. Useful for grabbing the reset-through-steady-state window in a single step:
```sh
tools/flash_and_monitor.sh [timeout_sec] [logfile]   # defaults: 60 /tmp/sesame_matter_dbg.log
```

### Device Flashing (first-time install only)

```sh
./tools/OpenOCD/flashprog.py -l boards/arm/marvell_mw302/flash-layout.txt \
  --boot2 mw320_sdk/mw320_matter_flash/Matter/boot2.bin \
  --wififw mw320_sdk/mw320_matter_flash/Matter/mw32x_uapsta_W14.88.36.p172.bin \
  --mcuboot build/mcuboot/zephyr/mcuboot.bin \
  --image-0 build/sesame/zephyr/zephyr.signed.bin -r
```

(Requires Tigard or similar JTAG board connected to J7 on iDCM board.)

### MCUboot and OTA Strategy

**Architecture & Hardware Translation:**
Sesame uses a **Direct-XIP** (Execute In Place) OTA strategy, leveraging the hardware capabilities of the MW320 flash controller (FLASHC).
Instead of copying the active image into a single primary slot, the bootloader (MCUboot) natively executes the image directly from whichever slot it resides in (`slot0` or `slot1`).
- The application is linked to a fixed virtual memory address (`0x1F000000`) with `CONFIG_FLASH_LOAD_OFFSET=0` (configured via a devicetree `linker_partition` at offset `0x0`).
- MCUboot determines the physical offset of the active slot (`0x30000` for Slot 0, `0x1A0000` for Slot 1), and writes this offset directly into the flash controller's `FAOFFR` register before jumping to the application.
- The flash controller automatically translates all virtual instruction fetches by adding the `FAOFFR` offset in hardware, allowing true position-independent execution without complex linker scripts.

**Build and Configuration:**
The Zephyr sysbuild framework coordinates building both the application and the MCUboot bootloader.
- The application flash build produces `build/sesame/zephyr/zephyr.signed.bin`.
- The MCUboot build produces `build/mcuboot/zephyr/mcuboot.bin` (via the custom `axf2firmware` tool post-build step in `mcuboot_module/CMakeLists.txt`).
- MCUboot configuration is managed in `sysbuild/mcuboot.conf`, which enables `CONFIG_BOOT_DIRECT_XIP=y` and `CONFIG_BOOT_DIRECT_XIP_REVERT=y`.

**Slot Detection Logic:**
Because the application is always linked at `0x1F000000`, the active slot cannot be determined via link address. Instead, `my_boot_fetch_active_slot()` in `src/ota.c` determines the active slot by explicitly querying the flash controller:
```c
if (FLASHC->FAOFFR > DT_REG_ADDR(DT_NODELABEL(slot0_partition))) {
    return 1; // Slot 1
}
return 0; // Slot 0
```
This guarantees the downloaded firmware update is safely written to the *inactive* slot, preventing the running application from overwriting itself.

**OTA Update Flow:**
1. Device downloads the firmware image (e.g., via the `/fwupgrade` POST endpoint).
2. The image is written to the inactive slot using Zephyr's flash map API.
3. Upon completion, the inactive slot is marked as `pending` and the device reboots.
4. MCUboot verifies the signature of the new image and boots it in **test mode** (Direct-XIP Revert mode) from the secondary slot.
5. If the new image boots successfully and the user verifies it, a POST to `/promote` sets the `image_ok` flag, confirming the update.
6. If the device crashes or reboots *before* confirmation, MCUboot detects the failure, erases the faulty image, and automatically rolls back to the previous slot.

**Matter Debugging Notes:**
- Zephyr networking features must be correctly configured to allow the Matter Minimal mDNS responder to function. Specifically, `CONFIG_NET_CONTEXT_RECV_PKTINFO=y` is required; without it, `IPV6_PKTINFO` or `IPV6_RECVPKTINFO` sockopt calls fail (error 109 `ENOPROTOOPT`), and `Minimal mDNS` drops incoming queries.
- Do NOT use `CONFIG_CHIP_ENABLE_PAIRING_AUTOSTART=y` for this device since we do not use BLE for commissioning. This flag starts the mDNS server immediately at boot *before* the WiFi interface connects, causing `Minimal mDNS` to bind to a down interface. Instead, wait for the network to be `UP` and manually open the commissioning window via `chip::Server::GetInstance().GetCommissioningWindowManager().OpenBasicCommissioningWindow()`.

## Dependencies

### External Libraries (via Zephyr / FetchContent)

| Library           | Purpose                                   |
| ----------------- | ----------------------------------------- |
| Zephyr RTOS       | Kernel, scheduler, threads, queues        |
| Zephyr Net Stack  | IPv4/IPv6 TCP/IP stack                    |
| nanopb            | Protobuf encoder/decoder                  |
| mbedTLS           | Crypto (AES, TLS), uses custom config     |

## File Organization

```
sesame/
├── src/                 # Application C source
│   ├── main.c          # Entry point, Zephyr init
│   ├── mqtt.c          # MQTT agent (thread + publish)
│   ├── httpd.c         # HTTP server callbacks
│   ├── network.c # WiFi & TCP/IP state machine
│   ├── config_manager.c  # Flash config I/O
│   ├── ota.c, ota_client.c # OTA logic
│   └── ...
├── include/            # Public headers
│   ├── controller.h    # Common data structues for controller queue
│   ├── mqtt.h, httpd.h, network.h, etc.
├── boards/             # Zephyr board definitions
├── proto/              # Protobuf definitions
│   ├── api.proto       # OTA request/response
│   └── app_config.proto # Configuration schema
├── CMakeLists.txt      # Main build configuration
└── mw320_sdk/          # (submodule) Hardware drivers, WiFi firmware, bootloader
```

## Coding Conventions

- **C99 standard** – See `target_compile_options` in CMakeLists.txt
- **Logging** – Use `LOG_ERROR`, `LOG_WARN`, `LOG_INFO`, `LOG_DEBUG` macros (defined in `app_logging.h`)
- **Task communication** – Via Zephyr message queues (see `main.c` for queue definitions)
- **Protobuf messages** – Use nanopb for encode/decode; see `proto/` for schemas and `.options` for size hints
- **Error handling** – Check return codes from WiFi/MQTT/HTTP APIs; many return status enums
- **Comments** – Omit obvious comments; clarify non-obvious logic

### Code style

Follow Google C/C++ code style for Sesame code.
Exceptions:
- Use snake_case for variable and function names.

Python scripts formatted with black, isort, flake8.

For library code, follow the style of the library (eg Zephyr)

Place `#include`s at the top, never in between functions.
Prefer including function declarations from headers instead of one-off `extern`s.
Always use braces with control flow statements like if, else, for, while, never
elide the braces even for simple single-line statements.

## Tools & IDE Support

- **Language server**: `.ccls` configured for ARM includes and cross-compilation flags
- **Formatting**: `.clang-format` – Run `clang-format -i src/*.c include/*.h` to format
- **Linker map**: Examine `build/sesame.map` for memory usage and symbol sizes
- **Compile DB**: Generate with `-DCMAKE_EXPORT_COMPILE_COMMANDS=TRUE` for IDE integration

## Serial Terminal & Testing

**Setup:**
```sh
pip install pyserial
```

### Monitoring Serial Output

The project provides a robust serial monitor that handles reconnections and timestamping.

**Method 1: Monitor-only**
```bash
./tools/monitor.py
```

**Method 2: Monitor After Flash**
Monitor an already flashed (XIP) application:
```bash
./tools/run_on_device.sh build/zephyr/zephyr.elf --no-load
```

### Rebooting and Serial Capture

To reboot the device and capture the initial boot sequence (critical for debugging boot crashes):

```bash
./tools/monitor.py & sleep 2 && ./tools/OpenOCD/flashprog.py -r && wait
```

## GDB Debugging

Debugging is performed via JTAG using OpenOCD and GDB.

### Setup

1. **Start OpenOCD** in a separate terminal:
```bash
openocd -s tools/OpenOCD -f tools/OpenOCD/interface/ftdi.cfg -f tools/OpenOCD/openocd.cfg
```

2. **Launch GDB** using the provided initialization script:
```bash
gdb-multiarch -x tools/OpenOCD/gdbinit build/zephyr/zephyr.elf
```

### Common Commands

The `gdbinit` script provides helper functions for common tasks:

- `debug` – Resets the board, loads the application into RAM, and stops at `main()`.
- `xip-debug` – Resets the board and stops at the Flash (XIP) application `main()`.

### Manual Debugging (Batch Mode)

To run automated GDB traces (e.g., in CI or for specific bug hunts):
```bash
gdb-multiarch -batch -x tools/OpenOCD/gdbinit build/zephyr/zephyr.elf \
  -ex "xip-debug" \
  -ex "thbreak mbedtls_ecp_mul" \
  -ex "continue" \
  -ex "bt" \
  -ex "quit"
```
*(Note: Use `thbreak` for hardware breakpoints when debugging code running from Flash.)*

### Committing Code

**Always before committing, build and run tests.**
