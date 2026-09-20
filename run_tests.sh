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
    SESAME_TEST_BIN="build/sesame_test/zephyr/zephyr.signed.bin"
    WAS_UP_TO_DATE=0
    if [ -f "$SESAME_TEST_BIN" ]; then
        NEWER_FILES=$(find src include drivers boards tests/system/firmware CMakeLists.txt prj.conf -type f -newer "$SESAME_TEST_BIN" 2>/dev/null || true)
        if [ -z "$NEWER_FILES" ]; then
            WAS_UP_TO_DATE=1
        fi
    fi

    BUILD_LOG=$(mktemp)
    (
        source third_party/connectedhomeip/scripts/activate.sh
        if [ ! -d "build/sesame_test" ]; then
            west build -b genie_idcm/88mw320/cpu0 -d build/sesame_test . -- -DEXTRA_CONF_FILE=tests/system/firmware/prj.conf -DDTC_OVERLAY_FILE=tests/system/firmware/overlay.overlay
        else
            ninja -C build/sesame_test
        fi
    ) > "$BUILD_LOG" 2>&1 || { cat "$BUILD_LOG"; rm -f "$BUILD_LOG"; exit 1; }
    cat "$BUILD_LOG"

    if [ "$WAS_UP_TO_DATE" -eq 1 ] && ! grep -qE "Building (C|CXX) object (CMakeFiles/app|drivers)" "$BUILD_LOG"; then
        echo "NOTE: sesame_test was not rebuilt, skipping device flash write"
    else
        echo "Flashing sesame_test to device..."
        ./tools/OpenOCD/flashprog.py --image-0 "$SESAME_TEST_BIN" -r
    fi
    rm -f "$BUILD_LOG"

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
