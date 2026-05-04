#!/usr/bin/env python3
"""Save TAL220B/HX711 experiment serial output to a CSV file.

Example:
  python tools/serial_csv_logger.py --port COM5 --out data/zero_drift.csv

The firmware prints normal CSV data rows and comment/status rows beginning
with "#". MATLAB, pandas, and many CSV tools can ignore those comments.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import pathlib
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "pyserial is required. Install it with: python -m pip install pyserial"
    ) from exc


def default_output_name() -> str:
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"tal220b_hx711_experiment_{stamp}.csv"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default=default_output_name())
    parser.add_argument(
        "--command",
        action="append",
        default=[],
        help="Command to send after opening the port. May be repeated.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after this many seconds. 0 means run until Ctrl+C.",
    )
    args = parser.parse_args()

    out_path = pathlib.Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with serial.Serial(args.port, args.baud, timeout=1.0) as ser, out_path.open(
        "w", encoding="utf-8", newline=""
    ) as f:
        time.sleep(2.0)
        ser.reset_input_buffer()

        for command in args.command:
            ser.write((command.strip() + "\n").encode("ascii"))
            ser.flush()
            time.sleep(0.1)

        print(f"Logging {args.port} at {args.baud} baud to {out_path}")
        print("Press Ctrl+C to stop.")

        start = time.monotonic()
        try:
            while True:
                if args.duration > 0.0 and (time.monotonic() - start) >= args.duration:
                    break

                raw_line = ser.readline()
                if not raw_line:
                    continue

                line = raw_line.decode("utf-8", errors="replace").rstrip("\r\n")
                print(line)
                f.write(line + "\n")
                f.flush()
        except KeyboardInterrupt:
            print("\nStopped.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
