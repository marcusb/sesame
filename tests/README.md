# Sesame Testing Strategy

This repository employs a 3-tiered testing strategy:

## 1. Unit Tests (`tests/unit`)
- Run logic-only C tests compiled using Zephyr's Ztest framework.
- These run entirely under `native_sim` and mock hardware.
- Run with: `./run_tests.sh unit`

## 2. Integration Tests (`tests/integration`)
- Run via Pytest orchestration against a `native_sim` target.
- Uses the Matter Python Controller bindings to test Matter logic end-to-end.
- Run with: `./run_tests.sh integration`

## 3. System Tests (`tests/system`)
- Black-box tests executed on real physical hardware.
- Flashes MCUboot and the Firmware onto the board, connects to the WiFi AP, and provisions credentials.
- Tests Matter OTA and Picard UART responses over the real WiFi network.
- Run with: `./run_tests.sh system /dev/ttyUSB0`
