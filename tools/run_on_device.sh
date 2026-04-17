#!/usr/bin/env bash
# Load an .axf into RAM and capture serial output until TEST_RESULT: or timeout.
#
# Usage: tools/run_on_device.sh <path/to/binary.axf> [timeout-seconds] [--no-load]
set -euo pipefail

AXF="${1:?usage: $0 <path/to/binary.axf> [timeout-seconds] [--no-load]}"
TIMEOUT="${2:-30}"
NO_LOAD=0
if [[ "${3:-}" == "--no-load" ]]; then
    NO_LOAD=1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)"

"$SCRIPT_DIR/monitor.py" --timeout "$TIMEOUT" --until "TEST_RESULT:" &
MON_PID=$!
sleep 2
if [[ $NO_LOAD -eq 0 ]]; then
    "$SCRIPT_DIR/OpenOCD/ramload.py" "$AXF"
fi
wait "$MON_PID"
