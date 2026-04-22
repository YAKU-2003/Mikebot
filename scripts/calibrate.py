#!/usr/bin/env python3
"""
Interactive calibration. For each joint:

    1. Servo is un-torqued so you can move it by hand to its TRUE zero
       (the angle you call 0° in the joint convention -- see
       biped/config.py).
    2. We read the raw position, store it as that joint's zero_raw.
    3. We torque the joint, command +10° and -10°, and ask whether the
       motion direction matches the convention. If not, we flip SIGN.
    4. Move on to the next joint.

Output: data/calibration.json. joints.py loads this on import.

USAGE:
    Hold the bot up by the torso. For each joint, physically pose THAT
    joint to its convention zero (e.g. for a knee: leg straight; for a
    hip yaw: foot pointing straight forward). Then press Enter.

    Press Ctrl-C any time to abort. Calibration is only saved if you
    finish all joints.
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from biped import config
from biped.joints import Legs
from biped.lx16a import LX16ABus


JOINT_HINTS = {
    "LHY": "Left HIP YAW: pose left foot pointing straight forward.",
    "LTP": "Left THIGH PITCH: thigh hangs straight down (vertical).",
    "LKP": "Left KNEE: leg fully straight (knee at 0°).",
    "LAP": "Left ANKLE: foot perpendicular to shin (sole flat).",
    "RHY": "Right HIP YAW: pose right foot pointing straight forward.",
    "RTP": "Right THIGH PITCH: thigh hangs straight down (vertical).",
    "RKP": "Right KNEE: leg fully straight (knee at 0°).",
    "RAP": "Right ANKLE: foot perpendicular to shin (sole flat).",
}


def calibrate_joint(bus: LX16ABus, legs: Legs, joint: str) -> tuple:
    sid = config.SERVO_IDS[joint]
    print()
    print("=" * 60)
    print(f"  JOINT  {joint}   (servo ID {sid})")
    print(f"  {JOINT_HINTS.get(joint, '')}")
    print("=" * 60)

    # Un-torque so the user can move it by hand
    bus.torque(sid, on=False)
    input("  Pose the joint to its CONVENTION ZERO, then press Enter... ")

    raw = bus.read_position(sid)
    print(f"  Captured zero_raw = {raw}")

    # Torque on at this position so it doesn't slam
    bus.torque(sid, on=True)
    bus.move(sid, raw, time_ms=200)
    time.sleep(0.3)

    # Provisional sign = +1; we'll flip if the test motion goes the wrong way
    sign = +1
    legs.zero_raw[joint] = raw
    legs.sign[joint] = sign

    # Drive +10° according to the joint convention
    print("  Test: commanding +10° (should move in the joint's POSITIVE direction)")
    legs.set(joint, +10.0, duration_ms=500)
    time.sleep(0.7)
    answer = input("  Did it move in the joint's POSITIVE direction? [Y/n] ").strip().lower()
    if answer in ("n", "no"):
        sign = -1
        legs.sign[joint] = sign
        print("  Flipped sign to -1.")

    # Return to zero
    legs.set(joint, 0.0, duration_ms=400)
    time.sleep(0.5)
    bus.torque(sid, on=False)  # release for safety
    return raw, sign


def main():
    print(__doc__)
    print()
    confirm = input("Bot is held up and you're ready? [y/N] ").strip().lower()
    if confirm != "y":
        print("Aborted.")
        return

    bus = LX16ABus()
    legs = Legs(bus=bus)

    try:
        for joint in config.JOINT_NAMES:
            calibrate_joint(bus, legs, joint)
    except KeyboardInterrupt:
        print("\nAborted by user. NOT saving partial calibration.")
        bus.torque_all_off()
        return
    finally:
        bus.torque_all_off()
        bus.close()

    legs.save_calibration()
    print()
    print("Done. Calibration saved.")
    print("Next: scripts/stand.py")


if __name__ == "__main__":
    main()
