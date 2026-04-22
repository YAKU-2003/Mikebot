"""
Named poses (joint-angle dictionaries) and a smooth interpolator.

A "pose" is just a dict of joint_name -> angle_deg. Missing joints
are left where they are. Use POSES["stand"] etc. as inputs to
gait sequences or one-shot moves.
"""

import time
from . import config

# All angles in degrees, all joint names from config.JOINT_NAMES.
POSES = {

    # Everything zeroed. Don't run this with the bot upright unless you
    # trust your calibration -- the legs will straighten fully.
    "zero": {j: 0.0 for j in config.JOINT_NAMES},

    # Neutral standing pose: knees slightly bent for shock absorption,
    # ankles compensating so feet are flat. Hips neutral.
    "stand": {
        "LHY": 0.0,  "LTP": 15.0, "LKP": 30.0, "LAP": -15.0,
        "RHY": 0.0,  "RTP": 15.0, "RKP": 30.0, "RAP": -15.0,
    },

    # Lower CoM, wider support polygon, ready to walk.
    "ready": {
        "LHY":  5.0, "LTP": 20.0, "LKP": 40.0, "LAP": -20.0,
        "RHY": -5.0, "RTP": 20.0, "RKP": 40.0, "RAP": -20.0,
    },

    # Squat (used for testing torque + leg geometry)
    "squat": {
        "LHY": 0.0,  "LTP": 45.0, "LKP": 80.0, "LAP": -25.0,
        "RHY": 0.0,  "RTP": 45.0, "RKP": 80.0, "RAP": -25.0,
    },

    # Lean body forward (ankle dorsiflexion); used to start a step.
    "lean_forward": {
        "LHY":  5.0, "LTP": 20.0, "LKP": 40.0, "LAP": -10.0,
        "RHY": -5.0, "RTP": 20.0, "RKP": 40.0, "RAP": -10.0,
    },

    # Lean body backward (ankle plantarflexion); used to recover.
    "lean_back": {
        "LHY":  5.0, "LTP": 20.0, "LKP": 40.0, "LAP": -30.0,
        "RHY": -5.0, "RTP": 20.0, "RKP": 40.0, "RAP": -30.0,
    },

    # PARK: compact crouch used as the "shutdown" pose.
    # Hips folded forward, knees bent ~80°. CoM is low and over the feet,
    # so torquing off from this pose causes only a small settle, not a fall.
    # Tune LTP up if the bot tips backward when reaching it; tune LTP down
    # if it tips forward. KP can go up to 90 if you want to sit even lower.
    "park": {
        "LHY": 0.0,  "LTP": 45.0, "LKP": 80.0, "LAP": -25.0,
        "RHY": 0.0,  "RTP": 45.0, "RKP": 80.0, "RAP": -25.0,
    },
}


def merge(base: dict, *overrides: dict) -> dict:
    """Right-most overrides win. Missing joints stay as in `base`."""
    out = dict(base)
    for ov in overrides:
        out.update(ov)
    return out


def interpolate(legs, target_pose: dict, duration_s: float = 1.0,
                steps: int = 20):
    """
    Smooth move to `target_pose` over `duration_s`, sampled at `steps`
    intermediate frames. Reads current pose first.

    This is gentler than handing the LX-16A a long single move because
    you can do safety checks (IMU, voltage, e-stop key) between frames.
    """
    current = legs.read_all()
    dt = duration_s / steps
    frame_ms = int(dt * 1000)
    for i in range(1, steps + 1):
        a = i / steps
        frame = {j: (1 - a) * current.get(j, 0.0) + a * target_pose[j]
                 for j in target_pose}
        legs.set_many(frame, duration_ms=frame_ms)
        time.sleep(dt)
