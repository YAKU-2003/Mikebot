#!/usr/bin/env python3
"""
Wake the bot from a limp / parked position and stand up smoothly.

    python3 scripts/stand_up.py            # park -> ready -> stand
    python3 scripts/stand_up.py --to ready # stop at the squatter 'ready' pose

The trick: when torque is OFF, you can still READ the servo positions.
We do that, then push those exact positions back as the setpoint with
duration=0, THEN turn torque on. That way the bot holds wherever it
physically is -- no jerk -- and we interpolate up from there.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from biped.joints import Legs
from biped.poses import POSES, interpolate
from biped import config


def capture_and_lock_setpoints(legs: Legs):
    """Read each (enabled) joint's current physical position and write it
    back as the move setpoint with time=0. Disabled joints get the
    locked angle. After this, torque-on holds the current pose without
    moving anywhere unexpected.
    """
    for joint in config.JOINT_NAMES:
        sid = config.SERVO_IDS[joint]
        if joint in legs.disabled:
            raw = legs.deg_to_raw(joint, legs.disabled[joint])
        else:
            try:
                raw = legs.bus.read_position(sid)
            except Exception as e:
                print(f"  [warn] couldn't read {joint} (id={sid}): {e}")
                continue
        legs.bus.move(sid, raw, time_ms=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--to", choices=["ready", "stand"], default="stand",
                    help="Final pose (default: stand)")
    ap.add_argument("--park-to-ready-s", type=float, default=2.5)
    ap.add_argument("--ready-to-stand-s", type=float, default=1.5)
    args = ap.parse_args()

    legs = Legs()

    print("Reading current physical pose (torque should be off) ...")
    capture_and_lock_setpoints(legs)

    print("Torquing on (holds current pose first) ...")
    legs.torque(True)
    legs.lock_disabled(duration_ms=400)
    time.sleep(0.4)

    print(f"Rising to 'ready' over {args.park_to_ready_s:.1f} s ...")
    interpolate(legs, POSES["ready"], duration_s=args.park_to_ready_s, steps=40)

    if args.to == "stand":
        print(f"Straightening to 'stand' over {args.ready_to_stand_s:.1f} s ...")
        interpolate(legs, POSES["stand"], duration_s=args.ready_to_stand_s, steps=30)

    print("Up. Holding. Ctrl-C to release torque.")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nReleasing torque.")
        legs.torque(False)


if __name__ == "__main__":
    main()
