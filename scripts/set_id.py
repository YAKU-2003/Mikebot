#!/usr/bin/env python3
"""
Reassign the ID of the SINGLE servo currently on the bus.

WORKFLOW:
    1. Power off bus.
    2. Connect ONE servo only.
    3. Power on bus.
    4. python3 scripts/set_id.py --new-id 11
    5. Power off, connect the next servo, repeat with the next ID.

Why? Because all LX-16As ship with ID=1. If you connect them all at
once, broadcasting an ID change would assign every servo the same new
ID -- you'd be no better off.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from biped.lx16a import LX16ABus, BROADCAST_ID
from biped import config


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--current-id", type=int, default=None,
                    help="Existing ID (omit to auto-detect by broadcast read)")
    ap.add_argument("--new-id", type=int, required=True,
                    help="New ID to assign (1..253)")
    ap.add_argument("--port", default=config.SERIAL_PORT)
    args = ap.parse_args()

    bus = LX16ABus(port=args.port)
    try:
        if args.current_id is None:
            print("Auto-detecting current ID via broadcast read...")
            try:
                cur = bus.read_id(BROADCAST_ID)
            except Exception as e:
                print(f"Couldn't auto-detect: {e}")
                print("Pass --current-id explicitly, or check that exactly ONE "
                      "servo is connected.")
                sys.exit(1)
            print(f"Detected current ID = {cur}")
        else:
            cur = args.current_id

        print(f"Setting servo {cur} -> ID {args.new_id} ...")
        bus.set_id(cur, args.new_id)

        # Verify
        try:
            verified = bus.read_id(args.new_id)
            if verified == args.new_id:
                print(f"OK. Servo now responds as ID {verified}. ✓")
            else:
                print(f"Wrote {args.new_id} but servo reports {verified}. "
                      "Power-cycle and re-check.")
        except Exception as e:
            print(f"Wrote new ID but verification failed: {e}")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
