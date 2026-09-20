#!/bin/bash
set -e

COMMAND=$1

setup_test_env() {
    WHEEL_DIR="third_party/connectedhomeip/out/python_lib/obj/src/controller/python/matter-controller-wheels"
    
    if ! ls $WHEEL_DIR/*.whl >/dev/null 2>&1; then
        echo "Matter python bindings wheel not found. Building it now..."
        # Run in subshell to avoid polluting test environment with Pigweed PATH
        (
            source third_party/connectedhomeip/scripts/activate.sh
            cd third_party/connectedhomeip
            ./scripts/build_python.sh -m minimal
        )
    fi

    # Reset VIRTUAL_ENV so uv targets the project's .venv properly
    unset VIRTUAL_ENV
    uv sync
    source .venv/bin/activate
    uv pip install $WHEEL_DIR/*.whl
}

run_unit() {
    echo "Running Unit Tests via Ztest..."
    (
        source third_party/connectedhomeip/scripts/activate.sh
        west twister -T tests/unit -p native_sim/native/64 -O build/twister-out
    )
}

run_integration() {
    echo "Building native_sim target..."
    (
        source third_party/connectedhomeip/scripts/activate.sh
        if [ ! -d "build/native_sim" ]; then
            west build -b native_sim/native/64 -d build/native_sim .
        else
            ninja -C build/native_sim
        fi
    )
    echo "Running Integration Tests via Pytest (QEMU/Native_Sim)..."
    setup_test_env
    python -m pytest tests/integration/test_matter_integration.py -v -s
}

run_system() {
    local port="${1:-/dev/ttyUSB0}"
    shift || true
    echo "Building sesame_test target..."
    (
        source third_party/connectedhomeip/scripts/activate.sh
        if [ ! -d "build/sesame_test" ]; then
            west build -b genie_idcm/88mw320/cpu0 -d build/sesame_test . -- -DEXTRA_CONF_FILE=tests/system/firmware/prj.conf -DDTC_OVERLAY_FILE=tests/system/firmware/overlay.overlay
        else
            ninja -C build/sesame_test
        fi
    )

    echo "Building MCUboot (Direct-XIP)..."
    (
        source third_party/connectedhomeip/scripts/activate.sh
        if [ ! -d "build" ]; then
            west build --sysbuild -d build .
        else
            ninja -C build mcuboot
        fi
    )

    echo "Generating OTA test images B and C..."
    SESAME_TEST_DIR="build/sesame_test/zephyr"
    SESAME_RAW_BIN="$SESAME_TEST_DIR/zephyr.bin"
    IMGTOOL="bootloader/mcuboot/scripts/imgtool.py"
    KEY="bootloader/mcuboot/root-rsa-2048.pem"

    python3 "$IMGTOOL" sign \
        --version 0.2.1+1 \
        --header-size 0x200 \
        --slot-size 1507328 \
        --align 4 \
        --key "$KEY" \
        "$SESAME_RAW_BIN" \
        "$SESAME_TEST_DIR/image_b.signed.bin"

    python3 "$IMGTOOL" sign \
        --version 0.2.2+1 \
        --header-size 0x200 \
        --slot-size 1507328 \
        --align 4 \
        --key "$KEY" \
        "$SESAME_RAW_BIN" \
        "$SESAME_TEST_DIR/image_c.signed.bin"

    echo "Preparing device: erasing image-1..."
    ./tools/OpenOCD/flashprog.py --erase 0,0x1a0000,0x170000

    echo "Flashing MCUboot..."
    ./tools/OpenOCD/flashprog.py --mcuboot build/mcuboot/zephyr/mcuboot.bin

    echo "Flashing sesame_test (Image A, confirmed) to image-0..."
    # Direct-XIP revert mode requires a valid confirmed trailer; a raw signed
    # image would be treated as a failed test image and erased by MCUboot.
    ./tools/OpenOCD/flashprog.py --image-0 "$SESAME_TEST_DIR/zephyr.signed.confirmed.bin" -r

    echo "Running System Tests via Pytest (Hardware)..."
    setup_test_env
    python -m pytest tests/system/ --device-port="$port" -v -s "$@"
}

case "$COMMAND" in
    unit)
        run_unit
        ;;
    integration)
        run_integration
        ;;
    system)
        PORT=${2:-"/dev/ttyUSB0"}
        run_system "$PORT" "${@:3}"
        ;;
    all)
        echo "=== Running all test stages (unit -> integration -> system) ==="
        PORT=${2:-"/dev/ttyUSB0"}
        echo "--- Stage 1: Unit Tests ---"
        run_unit
        echo "--- Stage 2: Integration Tests ---"
        run_integration
        echo "--- Stage 3: System Tests ---"
        run_system "$PORT" "${@:3}"
        echo "=== All test stages passed! ==="
        ;;
    *)
        echo "Usage: $0 {unit|integration|system|all} [port]"
        exit 1
        ;;
esac
