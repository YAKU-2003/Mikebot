#!/usr/bin/env python3
"""
Smoothly move into POSES['stand'] and hold until you Ctrl-C.

First end-to-end smoke test. Hold the bot up the first time you run this.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from biped.joints import Legs
from biped.poses import POSES, interpolate


def main():
    legs = Legs()
    print("Torquing on...")
    legs.torque(True)
    legs.lock_disabled(duration_ms=600)   # park any disabled joints (e.g. LAP)
    time.sleep(0.3)

    print("Interpolating into 'stand' over 2 s ...")
    interpolate(legs, POSES["stand"], duration_s=2.0, steps=40)

    print("Holding. Press Ctrl-C to release torque and exit.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nReleasing torque.")
    finally:
        legs.torque(False)


if __name__ == "__main__":
    main()
