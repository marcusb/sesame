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
   
   # Initialize only the connectedhomeip submodule
   git submodule update --init third_party/connectedhomeip
   
   # Use Matter's script to checkout only the required submodules for Zephyr and Linux host (for python bindings)
   cd third_party/connectedhomeip
   ./scripts/checkout_submodules.py --platform zephyr --platform linux
   cd ../..
   
   # Initialize any remaining submodules explicitly
   git submodule update --init
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

This produces `build/mcuboot/zephyr/mcuboot.bin`, `build/sesame/zephyr/zephyr.signed.bin`, and the native emulation artifact `build/native_sim/zephyr/zephyr.exe`.
 
## Building Matter Host Tools

For integration testing, you will need the Matter Python bindings.

To build the Python bindings:
```sh
source third_party/connectedhomeip/scripts/activate.sh
cd third_party/connectedhomeip

# Build the python bindings wheels
./scripts/build_python.sh -m minimal
cd ../..
```

You can then use `uv` to create a dedicated test environment and install the bindings and test dependencies:
```sh
uv venv --python python3 .venv_tests
source .venv_tests/bin/activate
uv pip install pytest pyserial protobuf requests third_party/connectedhomeip/out/python_lib/obj/src/controller/python/matter-controller-wheels/*.whl
```
This prepares the virtual environment to run `chip.ChipDeviceCtrl` scripts and Pytest hardware tests.

## native_sim Emulation

You can run the full application in a native POSIX environment (`native_sim`) for development and testing without hardware.
The `native_sim` build isolates board-dependent modules and uses the host OS for I/O. The `native_sim` variant is automatically orchestrated as a custom target during the main build via CMake's `ExternalProject_Add`.

```bash
# Build the native_sim variant
west build -b native_sim/native/64 -d build/native_sim
# Or it will be automatically built alongside the hardware build:
# west build --sysbuild

# Run the native simulation executable
./build/native_sim/zephyr/zephyr.exe
```

The `native_sim` build uses a local file `repl_storage.json` to persist configuration across restarts during tests.
Networking is supported via native OS tap devices or directly within the simulated application.

## Tests

The project includes unit, integration, and system tests using the [Unity](https://github.com/ThrowTheSwitch/Unity) and [pytest](https://pytest.org) frameworks. Tests can be run natively via `native_sim`, or on physical hardware.

The `./run_tests.sh` script provides convenient entrypoints to execute these test suites:

**1. Unit Tests (Type 1)**
Run native logic-only C tests compiled using Zephyr's Ztest framework under `native_sim`:
```sh
./run_tests.sh unit
```

**2. Integration Tests (Type 2)**
Run Matter integration tests via Pytest (e.g., testing the commissioning sequence using the CHIP python bindings against `native_sim`):
```sh
./run_tests.sh integration
```

**3. Physical System Tests (Type 3)**
Run black-box system tests on actual hardware via Pytest. Requires a flashed device connected at `/dev/ttyUSB0`:
```sh
./run_tests.sh system /dev/ttyUSB0
```

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
