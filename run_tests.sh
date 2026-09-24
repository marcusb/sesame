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
        west build -b native_sim/native/64 -d build/native_sim .
    )
    echo "Running Integration Tests via Pytest (QEMU/Native_Sim)..."
    setup_test_env
    python -m pytest tests/integration/test_matter_integration.py -v -s
}

run_system() {
    local port="${1:-/dev/ttyUSB0}"
    shift || true
    echo "Building MCUboot and Sesame..."
    (
        source third_party/connectedhomeip/scripts/activate.sh
        west build --sysbuild -d build .
    )

    echo "Building sesame_test target..."
    (
        source third_party/connectedhomeip/scripts/activate.sh
        west build -b genie_idcm/88mw320/cpu0 -d build/sesame_test .
    )

    SESAME_TEST_DIR="build/sesame_test/zephyr"

    # image_b.signed.bin, image_c.signed.bin and image_b.ota are produced by the
    # sesame_test CMake post-build step (see CMakeLists.txt), so no manual imgtool
    # / ota_image_tool invocations are needed here.

    echo "Preparing device: erasing image-1..."
    ./tools/OpenOCD/flashprog.py --erase 0,0x1a0000,0x170000

    # Storage (AppConfig + Matter fabric) is mapped to in-RAM sim_flash by
    # sysbuild/sesame_test.overlay, so it starts clean on every boot
    # with no pre-existing Matter fabric -- no flash erase needed.

    echo "Flashing..."
    ./tools/OpenOCD/flashprog.py --mcuboot build/mcuboot/zephyr/mcuboot.bin --image-0 build/sesame_test/sesame_test.bin

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
