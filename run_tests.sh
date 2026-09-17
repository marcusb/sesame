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

    uv sync
    source .venv/bin/activate
    uv pip install $WHEEL_DIR/*.whl
}

case "$COMMAND" in
    unit)
        echo "Running Unit Tests via Ztest..."
        source third_party/connectedhomeip/scripts/activate.sh
        west twister -T tests/unit -p native_sim/native/64 -O build/twister-out
        ;;
    integration)
        echo "Running Integration Tests via Pytest (QEMU/Native_Sim)..."
        setup_test_env
        python -m pytest tests/integration/test_matter_integration.py -v -s
        ;;
    system)
        echo "Running System Tests via Pytest (Hardware)..."
        PORT=${2:-"/dev/ttyUSB0"}
        setup_test_env
        python -m pytest tests/system/ --device-port=$PORT -v -s "${@:3}"
        ;;
    *)
        echo "Usage: $0 {unit|integration|system} [port]"
        exit 1
        ;;
esac
