#!/usr/bin/env bash
# Flash build/sesame.bin and capture serial output for a fixed window.
# Usage: tools/flash_and_monitor.sh [timeout_sec] [logfile]
set -u
TIMEOUT="${1:-60}"
LOG="${2:-/tmp/sesame_matter_dbg.log}"

cd "$(dirname "$0")/.."

./tools/monitor.py --timeout "$TIMEOUT" > "$LOG" 2>&1 &
MONPID=$!
sleep 2
./tools/OpenOCD/flashprog.py --mcufw build/sesame.bin -r 2>&1 | tail -5
wait "$MONPID"
echo "=== log lines: $(wc -l < "$LOG") at $LOG ==="
