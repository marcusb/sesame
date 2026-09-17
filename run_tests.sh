#!/bin/bash
set -e

COMMAND=$1

case "$COMMAND" in
    unit)
        echo "Running Unit Tests via Ztest..."
        source third_party/connectedhomeip/scripts/activate.sh
        west twister -T tests/unit -p native_sim/native/64 -O build/twister-out
        ;;
    integration)
        echo "Running Integration Tests via Pytest (QEMU/Native_Sim)..."
        source third_party/connectedhomeip/scripts/activate.sh
        if [ ! -f third_party/connectedhomeip/out/python_env/bin/activate ]; then
            echo "Matter python bindings not found. Building them now..."
            (cd third_party/connectedhomeip && ./scripts/build_python.sh -m minimal -i out/python_env)
        fi
        source third_party/connectedhomeip/out/python_env/bin/activate
        python -m pytest tests/integration/test_matter_integration.py -v -s
        ;;
    system)
        echo "Running System Tests via Pytest (Hardware)..."
        PORT=${2:-"/dev/ttyUSB0"}
        source third_party/connectedhomeip/scripts/activate.sh
        if [ ! -f third_party/connectedhomeip/out/python_env/bin/activate ]; then
            echo "Matter python bindings not found. Building them now..."
            (cd third_party/connectedhomeip && ./scripts/build_python.sh -m minimal -i out/python_env)
        fi
        source third_party/connectedhomeip/out/python_env/bin/activate
        # Hardware tests need pyserial (serial) and the protobuf runtime for tests/system/proto
        python -c "import serial" 2>/dev/null || python -m pip install --quiet pyserial
        python -c "import google.protobuf" 2>/dev/null || python -m pip install --quiet protobuf
        python -m pytest tests/system/test_hardware.py --device-port=$PORT -v -s "${@:3}"
        ;;
    *)
        echo "Usage: $0 {unit|integration|system} [port]"
        exit 1
        ;;
esac
