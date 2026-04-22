#!/usr/bin/env python3
"""
Graceful shutdown: smoothly fold the bot down into the 'park' crouch,
then release torque so it can sit there without burning power.

    python3 scripts/sit_down.py
    python3 scripts/sit_down.py --hold        # crouch but keep torque on

Use this instead of go_limp.py at the end of a normal session. The
park pose puts the CoM low and over the feet, so the small drop when
torque releases is harmless. go_limp.py is for emergencies only.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from biped.joints import Legs
from biped.poses import POSES, interpolate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hold", action="store_true",
                    help="Stop in the park pose with torque ON (don't release)")
    ap.add_argument("--duration-s", type=float, default=3.0,
                    help="Seconds to take folding into park (default 3.0)")
    args = ap.parse_args()

    legs = Legs()

    # Make sure the bot is currently torqued -- if it's already limp,
    # turning torque on will hold WHATEVER pose it's currently in, then
    # the interpolation moves it into park smoothly.
    print("Torquing on (will hold current pose first) ...")
    legs.torque(True)
    legs.lock_disabled(duration_ms=600)
    time.sleep(0.3)

    print(f"Folding into 'park' over {args.duration_s:.1f} s ...")
    interpolate(legs, POSES["park"], duration_s=args.duration_s, steps=40)

    if args.hold:
        print("Holding park pose. Ctrl-C to release.")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass

    print("Releasing torque. Bot will settle.")
    legs.torque(False)


if __name__ == "__main__":
    main()
