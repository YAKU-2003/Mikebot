#!/usr/bin/env python3
"""
Ping every possible servo ID on the LX-16A bus and report which respond.

    python3 scripts/ping_motors.py
    python3 scripts/ping_motors.py --max-id 50

Useful right after powering up to confirm wiring.
"""

import argparse
import sys
from pathlib import Path

# allow running as a script from the repo root
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from biped.lx16a import LX16ABus
from biped import config


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-id", type=int, default=32,
                    help="Highest ID to probe (default 32)")
    ap.add_argument("--port", default=config.SERIAL_PORT)
    args = ap.parse_args()

    print(f"Probing IDs 1..{args.max_id} on {args.port}")
    bus = LX16ABus(port=args.port)
    try:
        found = bus.ping(max_id=args.max_id)
    finally:
        bus.close()

    if not found:
        print("No servos responded. Check power, baud (115200), TX/RX wiring,"
              " and that you assigned IDs (servos all ship as ID=1).")
        sys.exit(1)

    print(f"Found {len(found)} servo(s): {found}")
    expected = set(config.SERVO_IDS.values())
    missing = expected - set(found)
    extra = set(found) - expected
    if missing:
        print(f"  MISSING from config:  {sorted(missing)}")
    if extra:
        print(f"  UNEXPECTED IDs:       {sorted(extra)}")
    if not missing and not extra:
        print("  All expected joints accounted for. ✓")


if __name__ == "__main__":
    main()