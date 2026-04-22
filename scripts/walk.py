#!/usr/bin/env python3
"""
Main walking demo.

    python3 scripts/walk.py                       # 4-step shuffle walk
    python3 scripts/walk.py --steps 8
    python3 scripts/walk.py --gait dynamic        # try the foot-lift gait
    python3 scripts/walk.py --no-imu              # skip IMU + safety monitor
                                                  # (useful if IMU isn't wired yet)

Hold the bot up by the torso the first few runs. When the keyframes
look right, set it down on a flat, slightly-grippy surface (carpet
works better than wood for the shuffle gait).
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from biped.joints import Legs
from biped.poses import POSES, interpolate
from biped import gait


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=4)
    ap.add_argument("--gait", choices=["shuffle", "dynamic"], default="shuffle")
    ap.add_argument("--frame-s", type=float, default=0.4)
    ap.add_argument("--yaw-deg", type=float, default=12.0)
    ap.add_argument("--lean-deg", type=float, default=8.0)
    ap.add_argument("--no-imu", action="store_true",
                    help="Skip IMU + SafetyMonitor (run open-loop)")
    args = ap.parse_args()

    legs = Legs()

    safety = None
    if not args.no_imu:
        try:
            from biped.imu import make_imu
            from biped.safety import SafetyMonitor
            imu = make_imu()
            safety = SafetyMonitor(imu, legs)
            safety.start()
            print("Safety monitor running.")
        except Exception as e:
            print(f"IMU/safety init failed ({e}). Continuing open-loop.")
            safety = None

    print("Torquing on, going to 'ready' pose...")
    legs.torque(True)
    legs.lock_disabled(duration_ms=600)   # park any disabled joints (e.g. LAP)
    interpolate(legs, POSES["ready"], duration_s=2.0, steps=40)

    try:
        if args.gait == "shuffle":
            gait.shuffle_walk(legs, steps=args.steps,
                              yaw_deg=args.yaw_deg,
                              lean_deg=args.lean_deg,
                              frame_s=args.frame_s,
                              safety=safety)
        else:
            gait.dynamic_step(legs, steps=args.steps,
                              step_yaw_deg=args.yaw_deg,
                              lean_deg=args.lean_deg,
                              frame_s=args.frame_s,
                              safety=safety)
        print("Gait sequence complete. Holding 'ready' for 2 s.")
        interpolate(legs, POSES["ready"], duration_s=1.0, steps=20)
        time.sleep(2.0)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except RuntimeError as e:
        print(f"\nABORT: {e}")
    finally:
        if safety is not None:
            safety.stop()
        legs.torque(False)
        print("Torque off. Done.")


if __name__ == "__main__":
    main()
