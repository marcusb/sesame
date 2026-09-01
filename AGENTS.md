## Project Overview

Sesame is custom firmware for the Genie 1155 garage door opener (MW300 SoC). It restores network control via MQTT and HTTP, replacing the defunct Aladdin Connect app. The firmware:

- Runs on a Marvell MW320 module with ARM Cortex-M4 processor
- Communicates locally only (no external network dependency)
- Uses FreeRTOS for task management
- Provides HTTP API and MQTT agent for remote control
- Supports over-the-air (OTA) firmware updates with A/B partition scheme

See docs/teardown.md for hardware information.

## Build System

The project uses the standard Zephyr sysbuild system to orchestrate multi-image builds natively for flash variants.

### Sysbuild Setup

Sesame relies on Zephyr's `sysbuild` to coordinate the building of both the primary application (`sesame`) and the bootloader (`mcuboot`) in a single step.

- **`sysbuild.conf`**: Configures the overall sysbuild environment, telling it to build MCUboot alongside the main application.
- **`sysbuild/mcuboot.conf`**: Contains the Kconfig overrides for the MCUboot image (e.g., enabling Direct-XIP, setting partition sizes, configuring logging).
- **`mcuboot_module/`**: Contains a custom CMakeLists file integrated via sysbuild that handles compiling the MW320-specific flash and pinctrl drivers into MCUboot, as well as running the `axf2firmware` post-build tool to convert MCUboot's `.elf` into the required `mcuboot.bin`.

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
   We build Matter within Zephyr, and they both expect a Python environment. To avoid managing two separate virtual environments and dealing with import errors, we use Matter's Pigweed environment as the sole environment and install Zephyr's tools directly into it.

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
   Ensure `ZEPHYR_BASE` is *not* exported in your shell.
   ```sh
   west init -l .
   west update
   ```
   *Note: Zephyr modules are placed in `./deps/` to prevent clashing with the local `./modules/mw320_sdk`.*

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

rm -rf build && ZEPHYR_BASE=$PWD/deps/zephyr west build --sysbuild
```

**Incremental build:**

```sh
ninja -C build                     # Build all variants (hardware)
ninja -C build sesame/zephyr/zephyr.elf   # Build only hardware flash version
```

**Iterating on QEMU-only sources (e.g. `test/integration/**`):**

The QEMU artifacts are produced by an `ExternalProject_Add(qemu_variants ...)`
that drives a separate inner build in `build/qemu-build/`. The *outer*
`ninja -C build qemu_variants` step always reports success because of
`BUILD_ALWAYS TRUE`, but only re-invokes the inner ninja — it does not show
the inner build's compile lines if nothing changed.

When changing a source that only the QEMU build consumes (anything under
`test/integration/`, `src/qemu/`, etc.), run the inner ninja directly so the
build lines are visible and you can confirm the binary actually rebuilt:

```sh
ninja -C build/qemu-build test/integration/<module>/<module>_it-qemu.axf
cp build/qemu-build/test/integration/<module>/<module>_it-qemu.axf \
   build/test/integration/<module>/<module>_it-qemu.axf
```

**Verifying you ran the binary you just built — not a stale log:**

When debugging by re-running QEMU and grepping a log file, always confirm:
1. `stat -c '%Y %n' /path/to/binary /path/to/log` — the log mtime must be
   *after* the binary mtime.
2. The QEMU process exited cleanly: capture `$?` after `kill $QEMU_PID;
   wait $QEMU_PID` and check `pgrep -af qemu-system` is empty before
   the next run.

Most subtle "the change had no effect" bugs in this project are not stale
binaries — they are stale log files left behind when a `pkill`/`kill` chain
in a shell one-liner aborted before the new QEMU launch. Treat the log
contents as suspect until you have proven its mtime is fresh.

**Build outputs:**

- `build/sesame/zephyr/zephyr.signed.bin`, `build/mcuboot/zephyr/mcuboot.bin` – Hardware flash versions
- `test/sesame_tests.axf` – Hardware test suite
- `test/sesame_tests-qemu.axf` – QEMU test suite

## Project Architecture

### App framework and management

1. **System startup** (`main.c`): Initializes generic RTOS scheduler and app tasks. Hardware-specific initialization is handled in `board_main.c` (physical device) or `qemu_main.c` (QEMU).
2. **Network stack** (`network.c`): Manages WiFi on hardware. QEMU uses direct Ethernet initialization in `qemu_main.c`.
3. **Configuration** (`config_manager.c`): Reads/writes protobuf config via `psm.h` abstraction.
4. **Control interfaces**:
   - HTTP server (`httpd.c`) for REST API and device setup
   - MQTT agent (`mqtt.c`) for pub/sub commands and door status

### Key Components

| Component                | File(s)                 | Purpose  |
| ------------------------ | ----------------------- | -------------------- |
| **App startup**          | `main.c`                | Initializes generic app tasks, Zephyr OS                             |
| **Board Entry**          | `board_main.c`          | Hardware-specific init, starts WiFi network manager                 |
| **Network Manager**      | `network.c`     | WiFi state machine, IP configuration (DHCP), hardware only   |
| **HTTP Server**          | `httpd.c`               | Receives config and OTA requests via REST, protobuf payloads                |
| **MQTT**                 | `mqtt.c`                | MQTT agent for pub/sub, topic structure, reconnection logic                 |
| **Config Manager**       | `config_manager.c`      | Read/write AppConfig (network, MQTT, logging) stored in PSM         |
| **OTA**                  | `ota.c`, `ota_client.c` | Firmware download, partition management, hardware only              |
| **LEDs & Buttons**       | `leds.c`      | Status indicators, user input handling, hardware only               |
| **PIC comms**            | `pic_uart.c`            | Serial I/O with the PIC16 for door control and status, hardware only|
| **Logging**              | `logging.c`, `sesame_syslog.c` | Circular buffer logs, syslog facility                               |
| **Board-specific files** | `board/*`               | Flash layout, board config, ld scripts                             |
| **QEMU Stubs**           | `qemu_stubs.c`          | Mocked peripherals for QEMU emulation                               |
| **QEMU PSM**             | `qemu_psm.c`            | Persistent storage via semihosting file I/O                         |

### Task Hierarchy (Zephyr)

- **Main thread** – Initialization, runs in idle; calls application task starts
- **System workqueue** – Handles delayed works (e.g. WiFi reconnect)
- **MQTT Agent thread** – Maintains MQTT connection, publishes door status
- **OTA thread** – Handles firmware download and flashing
- **HTTP server** runs in its own Zephyr socket thread

### Data Flow Example: Open Door via MQTT

```
1. MQTT message arrives → coreMQTT processes
2. MQTT agent task calls callback → publishes to ctrl_queue
3. Main task consumes ctrl_queue → enqueues command on pic_queue
4. pic_uart_task actuates door relay, status updated
5. MQTT publishes new state to broker
```

### Memory & Storage

- **Heap**: Two regions (SRAM0 for small allocations, main SRAM for larger)
- **Flash partitions** (managed by PSM):
  - Partition table, bootloader, WiFi firmware (via OpenOCD at first flash)
  - Primary firmware partition (active boot)
  - Secondary firmware partition (OTA staging)
  - PSM config partition (NetworkConfig, MqttConfig, LoggingConfig)

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

**Setup** – Install pyserial:
```sh
pip install pyserial
```

**Two-terminal workflow** for fast iteration (no device reboot between builds):

**Terminal 1** – Build and flash:
```sh
ninja -C build sesame/zephyr/zephyr.elf && ./tools/OpenOCD/flashprog.py --mcuboot build/mcuboot/zephyr/mcuboot.bin --image-0 build/sesame/zephyr/zephyr.signed.bin -r
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

**Automated testing with pyserial** – Claude can help write test scripts:
```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
time.sleep(1)

# Read boot output
output = ser.read_until(b'ready', timeout=5).decode(errors='ignore')
assert 'WiFi' in output, "WiFi init failed"

ser.write(b'test command\r\n')
response = ser.read_until(b'\n', timeout=2)
print(f"Response: {response.decode()}")

ser.close()
```

### Final Validation (Before Commit)

Before committing changes, ensure that all unit tests pass in both QEMU and on physical hardware.

**1. Run Unit & Integration Tests in QEMU:**
Build and run the full test suite in the emulator using CTest:
```sh
ninja -C build test
```

**2. Run On-device Unit Tests:**
```sh
ninja -C build sesame_tests && ./tools/run_on_device.sh build/test/sesame_tests.axf
```

**3. Flash and Full System Test:**
Build, flash, and test through full reboot cycle:
```bash
ninja -C build sesame/zephyr/zephyr.elf && \
./tools/OpenOCD/flashprog.py --mcuboot build/mcuboot/zephyr/mcuboot.bin --image-0 build/sesame/zephyr/zephyr.signed.bin -r && \
./tools/monitor.py
```

- Builds full flash binary
- Flashes permanently
- Device reboots automatically (`-r` option)
- Verify output and behavior persist through reboot

**IMPORTANT:** Never use `-l board/flash-layout.txt` with `flashprog.py` during development.
The `-l` flag erases and re-partitions the entire flash, including Boot2 and WiFi firmware.
Use `--mcuboot ... --image-0 ...` alone to flash only the bootloader and application partition.
The `-l` flag is only needed for initial device provisioning (first-time install).

### Logs & Debugging

**Log output** – All operations logged via `LOG_*` macros (see `include/app_logging.h`):
- `LOG_ERROR(msg)`, `LOG_WARN(msg)`, `LOG_INFO(msg)`, `LOG_DEBUG(msg)`
- Sent to console, circular buffer, and optional syslog
- Includes task name and timestamp

**Capture logs to file** – Via tee:
```sh
./tools/monitor.py | tee output.log
```

**Stack traces** – Compiled with `USE_BACKTRACE=ON` by default; on crash, backtrace printed to console.

### Unit Tests

On-device unit tests use the [Unity](https://github.com/ThrowTheSwitch/Unity) framework.
Tests can be run either on the physical ARM Cortex-M4 target via JTAG flash, or in QEMU.

**Hardware (JTAG) Build & Run:**
```sh
ninja -C build sesame_tests
./tools/run_on_device.sh build/test/sesame_tests.axf
```

**QEMU (Emulator) Build & Run:**
The build system automatically configures CTest to run tests in QEMU.
```sh
ninja -C build test
```
Individual test binaries (e.g., `test/sesame_tests-qemu.axf`, `test/matter_sesame_tests-qemu.axf`) can also be run manually via `qemu-system-arm`.

Each test prints immediately as it executes:
```
test/test_string_util.c:14:test_strtcpy_zero_dsize:PASS
test/test_string_util.c:27:test_strtcpy_normal:PASS
...
25 Tests 0 Failures 0 Ignored
OK
TEST_RESULT:0
```

`TEST_RESULT:0` = all passed. The test sources are in `test/`.

### Integration Testing

**Test boot sequence** – Verify device starts and connects:
```python
#!/usr/bin/env python3
import serial
import time
import sys

def test_boot(port='/dev/ttyUSB0', timeout=15):
    """Wait for device to boot and print ready message."""
    ser = serial.Serial(port, 115200, timeout=2)
    time.sleep(1)

    try:
        # Watch for boot logs
        start = time.time()
        boot_logs = ""
        while time.time() - start < timeout:
            chunk = ser.read(512)
            if chunk:
                boot_logs += chunk.decode(errors='ignore')
                print(chunk.decode(errors='ignore'), end='', flush=True)

        if 'WiFi' in boot_logs and 'ready' in boot_logs:
            print("\n✓ Boot successful")
            return True
        else:
            print("\n✗ Boot incomplete")
            return False
    finally:
        ser.close()

if __name__ == '__main__':
    sys.exit(0 if test_boot() else 1)
```

**Run after flash:**
```sh
ninja -C build sesame/zephyr/zephyr.elf && \
./tools/OpenOCD/flashprog.py --mcuboot build/mcuboot/zephyr/mcuboot.bin --image-0 build/sesame/zephyr/zephyr.signed.bin -r && \
python3 test_boot.py
```

**One-shot flash + capture** – `tools/flash_and_monitor.sh` starts `monitor.py` in the background, runs `flashprog.py --mcuboot build/mcuboot/zephyr/mcuboot.bin --image-0 build/sesame/zephyr/zephyr.signed.bin -r`, and writes the serial output to a log file. Useful for grabbing the reset-through-steady-state window in a single step:
```sh
tools/flash_and_monitor.sh [timeout_sec] [logfile]   # defaults: 60 /tmp/sesame_matter_dbg.log
```

### Device Flashing (first-time install only)

```sh
./tools/OpenOCD/flashprog.py -l board/flash_layout.txt \
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

**Automated OTA test with pyserial:**
```python
import serial
import time
import subprocess

# Monitor serial, wait for "test image running" message
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=10)
time.sleep(2)

output = ser.read_until(b'OTA LED blinks blue', timeout=30).decode(errors='ignore')
if 'Diagnostic' in output:
    print("✓ Test image booted successfully")
    # Promote to primary
    subprocess.run(['curl', '-v', 'http://sesame/promote'])
else:
    print("✗ Test image failed, rolling back")
    # Let device reboot naturally to rollback

ser.close()
```

## Dependencies

### External Libraries (via Zephyr / FetchContent)

| Library           | Purpose                                   |
| ----------------- | ----------------------------------------- |
| Zephyr RTOS       | Kernel, scheduler, threads, queues        |
| Zephyr Net Stack  | IPv4/IPv6 TCP/IP stack                    |
| coreMQTT          | MQTT client                               |
| coreMQTT-Agent    | MQTT task wrapper for thread-safety       |
| coreHTTP          | HTTP/1.1 client & server                  |
| nanopb            | Protobuf encoder/decoder                  |
| mbedTLS           | Crypto (AES, TLS), uses custom config     |
| backoff_algorithm | Exponential backoff for reconnection      |

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
├── board/              # Board support
│   ├── board.c
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

For library code, follow the style of the library (eg FreeRTOS)

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

**Always before committing, build and verify the XIP build.**
