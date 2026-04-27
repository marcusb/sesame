#!/usr/bin/env bash
# Flash build/sesame.bin and capture serial output for a fixed window.
# Usage: tools/flash_and_monitor.sh [timeout_sec] [logfile]
set -u
TIMEOUT="${1:-60}"
LOG="${2:-/tmp/sesame_matter_dbg.log}"
FLASHLOG="${LOG%.log}.flash.log"

cd "$(dirname "$0")/.."

./tools/monitor.py --timeout "$TIMEOUT" > "$LOG" 2>&1 &
MONPID=$!
sleep 2

./tools/OpenOCD/flashprog.py --mcufw build/sesame.bin -r > "$FLASHLOG" 2>&1
RC=$?
tail -5 "$FLASHLOG"

# flashprog.py exits 0 even when the embedded flashprog reports errors
# (e.g. "doesn't fit in available flash space"), so scan the output too.
if [ "$RC" -ne 0 ] || grep -qE "^Error:|doesn't fit" "$FLASHLOG"; then
    echo "=== flashprog FAILED (rc=$RC); see $FLASHLOG ==="
    kill "$MONPID" 2>/dev/null
    wait "$MONPID" 2>/dev/null
    exit 1
fi

wait "$MONPID"
echo "=== log lines: $(wc -l < "$LOG") at $LOG ==="
