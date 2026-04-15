## Project Overview

Sesame is custom firmware for the Genie 1155 garage door opener (MW300 SoC). It restores network control via MQTT and HTTP, replacing the defunct Aladdin Connect app. The firmware:

- Runs on a Marvell MW320 module with ARM Cortex-M4 processor
- Communicates locally only (no external network dependency)
- Uses FreeRTOS for task management
- Provides HTTP API and MQTT agent for remote control
- Supports over-the-air (OTA) firmware updates with A/B partition scheme

## Board Hardware

See @docs/teardown.md for hardware information.

## Build System

The project uses CMake with two-stage builds: a native build produces the `axf2firmware` tool, then a cross-compile builds the ARM firmware.

### Prerequisites

**Debian/Ubuntu:**

```sh
apt install cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi \
    binutils-arm-none-eabi protobuf-compiler python3-protobuf openocd
```

### Build Commands

**Clean build:**

```sh
rm -rf build-native build && mkdir build-native build
cd build-native && cmake -G Ninja .. && ninja && cd ..
cd build && cmake --no-warn-unused-cli -DCMAKE_BUILD_TYPE:STRING=Debug \
  -DUSE_BACKTRACE=ON -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE \
  -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -G Ninja \
  -Daxf2firmware_DIR=~/src/sesame/build-native .. && ninja
```

**Incremental build:**

```sh
ninja -C build                     # Build flash version
ninja -C build sesame_ram.axf      # Build RAM-bootable version
```

**Build outputs:**

- `sesame.axf` – ELF (flash version, for debugging)
- `sesame.bin` – Binary firmware image (for permanent flash)
- `sesame_ram.axf` – ELF (RAM version, loads non-permanently via ramload.py)
- `sesame.map`, `sesame_ram.map` – Linker map files (symbol addresses, memory layout)

## Project Architecture

### App framework and management

1. **System startup** (`main.c`): Initializes heap, flash/config storage (PSM), logging, FreeRTOS scheduler
2. **Network stack** (`network_manager.c`): Manages WiFi (STA mode or access point), DHCP, IPv4/IPv6
3. **Configuration** (`config_manager.c`): Reads/writes protobuf config from flash partitions
4. **Control interfaces**:
   - HTTP server (`httpd.c`) for REST API and device setup
   - MQTT agent (`mqtt.c`) for pub/sub commands and door status

### Key Components

| Component                | File(s)                 | Purpose  |
| ------------------------ | ----------------------- | -------------------- |
| **App startup**          | `main.c`                | Initializes board, logging, FreeRTOS                                        |
| **Network Manager**      | `network_manager.c`     | WiFi state machine, IP configuration (DHCP/static), connects tasks          |
| **HTTP Server**          | `httpd.c`               | Receives config and OTA requests via REST, protobuf payloads                |
| **MQTT**                 | `mqtt.c`                | MQTT agent for pub/sub, topic structure, reconnection logic                 |
| **Config Manager**       | `config_manager.c`      | Read/write AppConfig (network, MQTT, logging) stored in flash PSM partition |
| **OTA**                  | `ota.c`, `ota_client.c` | Firmware download, partition management, A/B boot scheme                    |
| **LEDs & Buttons**       | `leds.c`, `gpio.c`      | Status indicators, user input handling                                      |
| **PIC comms**            | `pic_uart.c`            | Serial I/O with the PIC16 for door control and status                       |
| **Logging**              | `logging.c`, `syslog.c` | Circular buffer logs, syslog facility                                       |
| **Board-specific files** | `board/*`               | Flash layout, board config, ld scripts                                      |

### Task Hierarchy (FreeRTOS)

- **Main task** – Initialization, runs in idle; calls application task starts
- **Network Manager task** – Manages WiFi and TCP/IP stack events
- **MQTT Agent task** – Maintains MQTT connection, publishes door status
- **OTA task** – Handles firmware download and flashing
- **HTTP server** runs in callback (coreHTTP, event-driven within network stack)

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

**Terminal 1** – Build and load to RAM:
```sh
ninja -C build sesame_ram.axf && ./tools/OpenOCD/ramload.py build/sesame_ram.axf
```

**Terminal 2** – Monitor serial output:
```sh
# Method 1: Quick monitor (stty + cat)
stty -F /dev/ttyUSB0 115200 raw -echo && cat /dev/ttyUSB0

# Method 2: Robust monitor (pyserial script)
python3 -c "
import serial, sys
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
while True:
    if ser.in_waiting:
        print(ser.read(ser.in_waiting).decode('utf-8', errors='replace'), end='', flush=True)
"
```

Typical feature development cycle:
1. Understand affected modules and check proto definitions
2. Implement code changes
3. Re-run Terminal 1 command
4. Check Terminal 2 for output (no reboot unless code crashes)
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

Build, flash, and test through full reboot cycle:
```bash
ninja -C build sesame.axf && \
./tools/OpenOCD/flashprog.py --mcufw build/sesame.bin -r && \
stty -F /dev/ttyUSB0 115200 raw -echo && cat /dev/ttyUSB0
```

- Builds full flash binary
- Flashes permanently
- Device reboots automatically (`-r` option)
- Verify output and behavior persist through reboot

### Logs & Debugging

**Log output** – All operations logged via `LOG_*` macros (see `include/app_logging.h`):
- `LOG_ERROR(msg)`, `LOG_WARN(msg)`, `LOG_INFO(msg)`, `LOG_DEBUG(msg)`
- Sent to console, circular buffer, and optional syslog
- Includes task name and timestamp

**Capture logs to file** – Via tee:
```sh
stty -F /dev/ttyUSB0 115200 raw -echo && cat /dev/ttyUSB0 | tee output.log
```

**Stack traces** – Compiled with `USE_BACKTRACE=ON` by default; on crash, backtrace printed to console.

**GDB debugging** – Flash the executable when debugging. The `.axf` file contains debug symbols:
```sh
arm-none-eabi-gdb build/sesame.axf
(gdb) target remote :3333  # OpenOCD listening on 3333
```

### Unit Tests

On-device unit tests use the [Unity](https://github.com/ThrowTheSwitch/Unity) framework.
Tests run on the ARM Cortex-M4 target via JTAG RAM load (no flash write required).

**Build:**
```sh
ninja -C build test/sesame_tests.axf
```

**Run:**
```sh
./tools/OpenOCD/ramload.py build/test/sesame_tests.axf
```

**Monitor output:**
```sh
stty -F /dev/ttyUSB0 115200 raw -echo && cat /dev/ttyUSB0
```

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

### Host Tests

Board-independent modules can be tested on the host without hardware.
Tests run FreeRTOS + FreeRTOS-Plus-TCP via the POSIX kernel port and libslirp user-mode NIC,
driven by pytest from outside the process over real TCP/UDP.

**Prerequisites (Debian/Ubuntu):**
```sh
apt install libslirp-dev python3-uv
```

Host tests build automatically as part of the normal ARM build — `ninja -C build` builds them alongside the firmware via `ExternalProject_Add` into `build/host-tests/`.

To run the tests:
```sh
cd test/host && uv run pytest
```

Or via CTest:
```sh
ctest --test-dir build/host-tests --output-on-failure
```

To configure the host tests standalone (without the ARM toolchain):
```sh
cmake -B build-host -G Ninja
ninja -C build-host host_tests
```

The test sources are in `test/host/`. Each binary boots a minimal FreeRTOS+TCP stack, starts the module under test, and exposes a real TCP/UDP endpoint via libslirp port forwarding.

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
ninja -C build sesame.axf && \
./tools/OpenOCD/flashprog.py --mcufw build/sesame.bin -r && \
python3 test_boot.py
```

### Device Flashing (first-time install only)

```sh
./tools/OpenOCD/flashprog.py -l mw320_sdk/tools/boot2/layout.txt \
  --boot2 mw320_sdk/mw320_matter_flash/Matter/boot2.bin \
  --wififw mw320_sdk/mw320_matter_flash/Matter/mw32x_uapsta_W14.88.36.p172.bin \
  --mcufw build/sesame.bin -r
```

(Requires Tigard or similar JTAG board connected to J7 on iDCM board.)

### OTA Updates

**Manual flow:**
1. Build `sesame.axf`
2. Host on HTTP server (not HTTPS)
3. From device, POST to `/fwupgrade` with FirmwareUpgradeFetchRequest (URL in protobuf)
4. Device boots test image (OTA LED blinks blue)
5. Verify, then POST to `/promote` to commit or reboot to rollback

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

### External Libraries (via CMakeLists.txt / FetchContent)

| Library           | Purpose                                   |
| ----------------- | ----------------------------------------- |
| FreeRTOS Kernel   | RTOS scheduler, tasks, queues, semaphores |
| FreeRTOS-Plus-TCP | IPv4/IPv6 TCP/IP stack                    |
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
│   ├── main.c          # Entry point, FreeRTOS init
│   ├── mqtt.c          # MQTT agent (task + publish)
│   ├── httpd.c         # HTTP server callbacks
│   ├── network_manager.c # WiFi & TCP/IP state machine
│   ├── config_manager.c  # Flash config I/O
│   ├── ota.c, ota_client.c # OTA logic
│   └── ...
├── include/            # Public headers
│   ├── controller.h    # Common data structues for controller queue
│   ├── mqtt.h, httpd.h, network.h, etc.
│   └── config/         # FreeRTOS config headers
├── board/              # Board support
│   ├── board.c, clock_config.c, pin_mux.c
│   └── 88MW320_xx_xxxx_flash.ld (linker script)
├── proto/              # Protobuf definitions
│   ├── api.proto       # OTA request/response
│   └── app_config.proto # Configuration schema
├── CMakeLists.txt      # Main build configuration
├── toolchain.cmake     # ARM cross-compiler settings
└── mw320_sdk/          # (submodule) Hardware drivers, WiFi firmware, bootloader
```

## Coding Conventions

- **C99 standard** – See `target_compile_options` in CMakeLists.txt
- **Logging** – Use `LOG_ERROR`, `LOG_WARN`, `LOG_INFO`, `LOG_DEBUG` macros (defined in `app_logging.h`)
- **Task communication** – Via FreeRTOS queues (see `main.c` for queue definitions)
- **Protobuf messages** – Use nanopb for encode/decode; see `proto/` for schemas and `.options` for size hints
- **Error handling** – Check return codes from WiFi/MQTT/HTTP APIs; many return status enums
- **Comments** – Omit obvious comments; clarify non-obvious logic

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

**Monitor firmware output** – Use `stty` + `cat` for quick checks or `pyserial` for robust monitoring:
```sh
# Method 1: Quick monitor
stty -F /dev/ttyUSB0 115200 raw -echo && cat /dev/ttyUSB0

# Method 2: Robust monitor (pyserial script)
python3 -c "
import serial, sys
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
while True:
    if ser.in_waiting:
        print(ser.read(ser.in_waiting).decode('utf-8', errors='replace'), end='', flush=True)
"
```

**Features:**
- `Ctrl+C` to exit
- `| tee output.log` to capture logs
- Works well in scripts and CI/CD pipelines
...
**Programmatic testing** – Use pyserial directly in Python test scripts (see Integration Testing section). Claude can help write test automation that verifies device behavior, OTA updates, MQTT messages, etc.

**Quick test after build:**
```bash
python3 -c "
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
time.sleep(1)
data = ser.read(256).decode(errors='ignore')
print(data)
ser.close()
print('✓ Device is running' if 'Sesame' in data else '✗ No output')
"
```
