# Developing Sesame

The firmware can be built on Linux.

## Prerequisites

On Debian:
```sh
apt install cmake ninja-build python3-protobuf openocd qemu-system-arm python3-venv
```

## Bootstrapping the Project

This project uses a self-contained [West workspace topology](https://docs.zephyrproject.org/latest/develop/west/workspaces.html). Zephyr and its dependencies are managed directly within this repository.

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
   *Note: Ensure you do not have `ZEPHYR_BASE` exported in your environment before running this.*
   ```sh
   west init -l .
   west update
   ```

   *West will download the Zephyr RTOS, its modules (into `./deps/`), and the custom MCUboot bootloader.*

## Code Generation (ZAP / Matter IDL)

Matter uses ZCL Advanced Platform (ZAP) to define clusters and endpoints. 
The configuration is stored in `src/matter/window-app.zap` and its corresponding IDL format `src/matter/window-app.matter`.

If you need to change cluster configurations (e.g. adding a new endpoint or feature):
1. Modify `window-app.zap` using the ZAP UI:
   ```sh
   ./third_party/connectedhomeip/.environment/cipd/packages/zap/zap-cli \
     -z third_party/connectedhomeip/src/app/zap-templates/zcl/zcl.json \
     src/matter/window-app.zap
   ```
2. Generate the updated `window-app.matter` file from your changes:
   ```sh
   ./third_party/connectedhomeip/scripts/tools/zap/generate.py \
     src/matter/window-app.zap -o src/matter/
   ```
3. Rebuild the project. The build system will automatically invoke `codegen.py` to generate the updated C++ static cluster configurations from `window-app.matter`.

## Building

The Zephyr sysbuild system handles building both the MCUboot bootloader and the Sesame application automatically.
*Ensure you have activated the Python and Matter environment before building (see Bootstrapping above).*

```sh
source third_party/connectedhomeip/scripts/activate.sh

rm -rf build
west build --sysbuild
```

This produces `build/mcuboot/zephyr/mcuboot.bin` and `build/sesame/zephyr/zephyr.signed.bin`.

## QEMU Emulation

You can run the full application in QEMU for development and testing without hardware.
The QEMU build isolates board-dependent modules and uses semihosting for I/O. Both hardware and QEMU variants are built automatically.

```bash
ninja -C build
qemu-system-arm -M mps2-an386 -nographic -semihosting \
  -kernel build/test/sesame_tests-qemu.axf \
  -serial none -monitor none \
  -net nic,model=lan9118 -net user,hostfwd=tcp::8080-:80
```

The QEMU build uses a local file `sesame_psm.bin` to persist configuration (PSM) across restarts.
Networking is supported via QEMU's user-mode stack (SLIRP) with port forwarding.

## Tests

The project includes unit and integration tests using the [Unity](https://github.com/ThrowTheSwitch/Unity) and [pytest](https://pytest.org) frameworks. Tests can be run either on physical hardware or in the QEMU emulator.

**Hardware (JTAG) Build & Run:**
```sh
ninja -C build sesame_tests
./tools/run_on_device.sh build/test/sesame_tests.axf
```

**QEMU (Emulator) Build & Run:**
The build system automatically configures CTest to run all unit and integration tests in QEMU.
```sh
ninja -C build test
```

Expected output ends with:
```
34 Tests 0 Failures 0 Ignored
OK
TEST_RESULT:0
```

`TEST_RESULT:0` means all tests passed.

## References

For hardware details see the [teardown](teardown.md).

## Logs & Debugging

The system outputs logs via the serial interface (`/dev/ttyUSB0` at 115200 baud).
You can capture and monitor the serial output simultaneously by running:
```sh
./tools/flash_and_monitor.sh [timeout_sec] [logfile]
```

### Debugging Kernel Panics (Core Dumps)

If the device crashes due to a kernel panic or `abort()`, Zephyr will dump the memory core to the serial output (if `CONFIG_DEBUG_COREDUMP=y` is enabled in `prj.conf`).

To extract and decode the core dump:

1. **Extract the dump block:**
   Find the block in your serial log starting with `#CD:BEGIN#` and ending with `#CD:END#`. Save this entire block to a file, for example, `coredump.log`.

2. **Clean up the log (optional but recommended):**
   Serial monitors often inject ANSI escape codes or Windows carriage returns (`\r`) which break the Python parser. Clean it up before parsing:
   ```sh
   sed -r 's/\x1B\[[0-9;]*[a-zA-Z]//g' coredump.log | tr -d '\r' > coredump_clean.log
   ```

3. **Convert the log to a binary core file:**
   *Note: Ensure your Matter/Zephyr python environment is active!*
   ```sh
   python deps/zephyr/scripts/coredump/coredump_serial_log_parser.py coredump_clean.log core.bin
   ```

4. **Start the Zephyr GDB server:**
   Zephyr uses a custom python-based GDB server to translate the core binary. Start it in the background on an available port (e.g. 2345):
   ```sh
   python deps/zephyr/scripts/coredump/coredump_gdbserver.py build/sesame/zephyr/zephyr.elf core.bin --port 2345 &
   ```

5. **Run GDB to get the backtrace:**
   ```sh
   ~/zephyr-sdk/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb -batch -ex "target remote localhost:2345" -ex "bt" build/sesame/zephyr/zephyr.elf
   ```
   *Don't forget to kill the background gdbserver when you are done (`kill %1`).*
