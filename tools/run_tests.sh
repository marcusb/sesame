#!/usr/bin/env bash
#
# Harness to run on-device tests via semihosting and capture results.
#
# Usage: tools/run_tests.sh <path/to/test.axf> [timeout-seconds]

set -euo pipefail

AXF="${1:?usage: $0 <path/to/test.axf> [timeout-seconds]}"
TIMEOUT="${2:-300}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)"
RAMLOAD="$SCRIPT_DIR/OpenOCD/ramload.py"

if [[ ! -f "$AXF" ]]; then
    echo "Error: file not found: $AXF"
    exit 1
fi

echo "Starting tests for $(basename "$AXF")..."

# We run ramload.py with semihosting enabled. 
# It will launch openocd, load the code, and keep openocd running to handle semihosting.
# ramload.py -s uses sh_load which keeps the session alive.

# Create a temporary file to capture output
LOGFILE=$(mktemp)
trap 'rm -f "$LOGFILE"' EXIT

# Start ramload in the background. We use --semihosting which maps to -s.
# ramload.py -s will keep OpenOCD running.
"$RAMLOAD" -s "$AXF" > "$LOGFILE" 2>&1 &
RAMLOAD_PID=$!

# Function to clean up background processes
cleanup() {
    kill $RAMLOAD_PID 2>/dev/null || true
    # Kill any openocd processes started by this script
    # We use a pattern that matches the config file we use
    pkill -f "openocd.*OpenOCD/openocd.cfg" 2>/dev/null || true
}
trap 'cleanup; rm -f "$LOGFILE"' EXIT

# Monitor the log file for the test result
RESULT=""
START_TIME=$(date +%s)
LAST_BYTE=0

while true; do
    if [[ -f "$LOGFILE" ]]; then
        # Print new bytes
        TOTAL_BYTES=$(stat -c%s "$LOGFILE")
        if (( TOTAL_BYTES > LAST_BYTE )); then
            # Read only new bytes
            dd if="$LOGFILE" bs=1 skip=$LAST_BYTE count=$((TOTAL_BYTES - LAST_BYTE)) 2>/dev/null | sed 's/\r//g'
            LAST_BYTE=$TOTAL_BYTES
        fi

        # Check for result line with a numeric value
        if grep -q "TEST_RESULT:[0-9]" "$LOGFILE"; then
            RESULT=$(grep "TEST_RESULT:[0-9]" "$LOGFILE" | tail -n1 | sed 's/.*TEST_RESULT:\([0-9]*\).*/\1/')
            break
        fi
    fi
    
    # Check if ramload/openocd died
    if ! kill -0 $RAMLOAD_PID 2>/dev/null; then
        # Give it a tiny bit of time to flush the very last bit
        sleep 0.2
        # Check one last time for result in log
        if grep -q "TEST_RESULT:[0-9]" "$LOGFILE"; then
            RESULT=$(grep "TEST_RESULT:[0-9]" "$LOGFILE" | tail -n1 | sed 's/.*TEST_RESULT:\([0-9]*\).*/\1/')
            break
        else
            echo "Error: ramload.py / OpenOCD terminated unexpectedly"
            # Final dump of log
            TOTAL_BYTES=$(stat -c%s "$LOGFILE")
            if (( TOTAL_BYTES > LAST_BYTE )); then
                dd if="$LOGFILE" bs=1 skip=$LAST_BYTE count=$((TOTAL_BYTES - LAST_BYTE)) 2>/dev/null | sed 's/\r//g'
            fi
            exit 1
        fi
    fi

    # Check for timeout
    CURRENT_TIME=$(date +%s)
    if (( CURRENT_TIME - START_TIME > TIMEOUT )); then
        echo "Error: Timeout reached ($TIMEOUT seconds)"
        exit 1
    fi
    
    sleep 0.5
done

# Print final result
if [[ "$RESULT" == "0" ]]; then
    echo "Tests PASSED"
    exit 0
else
    echo "Tests FAILED with result $RESULT"
    exit "$RESULT"
fi
