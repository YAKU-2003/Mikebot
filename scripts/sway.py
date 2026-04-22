#!/usr/bin/env python3
"""
Diagnostic: rock the bot forward & back at the ankles while printing
IMU pitch. Used to:

    - confirm the IMU orientation (pitch should track ankle pitch)
    - tune the safety monitor's tilt limit
    - sanity-check the complementary filter

Hold the bot up. The motion is small (~10°) but the IMU readings
should clearly swing positive (nose up) and negative (nose down).
"""

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from biped.joints import Legs
from biped.imu import make_imu
from biped.poses import POSES, interpolate


def main():
    legs = Legs()
    imu = make_imu()

    legs.torque(True)
    legs.lock_disabled(duration_ms=600)
    interpolate(legs, POSES["ready"], duration_s=2.0, steps=40)

    print("Sway test. Ctrl-C to exit.")
    print("frame |  ankle_cmd   imu_roll   imu_pitch   yaw_rate")
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            ankle = -10.0 * math.sin(2 * math.pi * 0.3 * t)  # ±10° at 0.3 Hz
            base = POSES["ready"]
            frame = dict(base)
            frame["LAP"] = base["LAP"] + ankle
            frame["RAP"] = base["RAP"] + ankle
            legs.set_many(frame, duration_ms=80)

            roll, pitch, yawr = imu.update()
            print(f"  {t:5.2f} | {ankle:+7.2f}   "
                  f"{roll:+7.2f}   {pitch:+7.2f}   {yawr:+7.2f}",
                  end="\r", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nDone. Releasing torque.")
    finally:
        legs.torque(False)


if __name__ == "__main__":
    main()
