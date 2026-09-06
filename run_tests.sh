#!/bin/bash
set -e

COMMAND=$1

case "$COMMAND" in
    unit)
        echo "Running Unit Tests via Ztest..."
        source third_party/connectedhomeip/scripts/activate.sh
        west twister -T tests/unit -p native_sim/native/64
        ;;
    integration)
        echo "Running Integration Tests via Pytest (QEMU/Native_Sim)..."
        source third_party/connectedhomeip/scripts/activate.sh
        source third_party/connectedhomeip/out/python_env/bin/activate
        python -m pytest tests/integration/test_matter_integration.py -v -s
        ;;
    system)
        echo "Running System Tests via Pytest (Hardware)..."
        if [ -z "$2" ]; then
            echo "Error: Please specify the serial port for system tests (e.g. ./run_tests.sh system /dev/ttyUSB0)"
            exit 1
        fi
        PORT=$2
        echo "Placeholder: Flash firmware, monitor $PORT, provision Wi-Fi, and run OTA."
        # Placeholder for pytest tests/system
        ;;
    *)
        echo "Usage: $0 {unit|integration|system} [port]"
        exit 1
        ;;
esac
