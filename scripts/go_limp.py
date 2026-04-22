#!/usr/bin/env python3
"""
EMERGENCY STOP: torque off every servo (broadcast).

Keep an SSH session ready with this command queued. One Enter and the
bot goes limp. Run multiple times if you're paranoid -- it's idempotent.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from biped.lx16a import LX16ABus
from biped import config


def main():
    bus = LX16ABus(port=config.SERIAL_PORT)
    try:
        bus.torque_all_off()
        # Per-ID too, in case the broadcast packet got lost
        for sid in config.SERVO_IDS.values():
            try:
                bus.torque(sid, on=False)
            except Exception:
                pass
        print("All servos un-torqued.")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
