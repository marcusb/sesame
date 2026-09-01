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
   Because we build Matter within Zephyr, we must integrate Pigweed's environment with Zephyr's `.venv`.

   ```sh
   # 1. Set up the local venv
   python3 -m venv .venv
   source .venv/bin/activate
   pip install west pyserial protobuf python-path
   
   # 2. Export PYTHONPATH so CMake/GN can find packages in the .venv
   export PYTHONPATH=$PWD/.venv/lib/python3.14/site-packages:$PYTHONPATH
   
   # 3. Bootstrap the Matter (CHIP) environment
   source third_party/connectedhomeip/scripts/activate.sh
   ```

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
export PYTHONPATH=$PWD/.venv/lib/python3.14/site-packages:$PYTHONPATH
source .venv/bin/activate
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
