#!/usr/bin/env bash
# Load an .axf into RAM and capture serial output until TEST_RESULT: or timeout.
#
# Usage: tools/run_on_device.sh <path/to/binary.axf> [timeout-seconds]
set -euo pipefail

AXF="${1:?usage: $0 <path/to/binary.axf> [timeout-seconds]}"
TIMEOUT="${2:-30}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$SCRIPT_DIR/monitor.py" --timeout "$TIMEOUT" --until "TEST_RESULT:" &
MON_PID=$!
sleep 2
"$SCRIPT_DIR/OpenOCD/ramload.py" "$AXF"
wait "$MON_PID"
