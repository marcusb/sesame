#!/usr/bin/env python3
"""Serial monitor for the MW320 board.

Reads from /dev/ttyUSB0 at 115200 baud and prints to stdout. Designed for
automated/CI use: exits cleanly on a sentinel string or after a timeout.

Usage:
    tools/monitor.py                       # run until Ctrl+C
    tools/monitor.py --timeout 20          # stop after 20 seconds
    tools/monitor.py --until TEST_RESULT:  # stop after sentinel seen
    tools/monitor.py --port /dev/ttyUSB1
"""

import argparse
import sys
import time

import serial


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--timeout", type=float, default=0.0,
                   help="stop after N seconds (0 = run forever)")
    p.add_argument("--until", dest="sentinel", default=None,
                   help="stop when this string appears in the output")
    args = p.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    deadline = time.time() + args.timeout if args.timeout > 0 else None
    buf = ""
    try:
        while True:
            if deadline is not None and time.time() >= deadline:
                return 0
            n = ser.in_waiting
            if n:
                data = ser.read(n).decode("utf-8", errors="replace")
                sys.stdout.write(data)
                sys.stdout.flush()
                if args.sentinel:
                    buf = (buf + data)[-4096:]
                    if args.sentinel in buf:
                        return 0
            else:
                time.sleep(0.02)
    except KeyboardInterrupt:
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
