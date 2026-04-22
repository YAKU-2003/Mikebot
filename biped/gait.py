"""
Keyframe walking gaits.

Two flavors are provided. Start with `shuffle_walk()` -- it keeps both
feet on the ground and uses hip-yaw to scissor the legs, since you have
no roll DOF for lateral weight shift. When that works, try
`dynamic_step()` which actually picks a foot up using thigh+knee pitch
and accepts a small forward fall.

Each step takes a `Legs` instance, an optional `SafetyMonitor`, and
parameters. They block (sleep) between keyframes. Call them in a loop
from `scripts/walk.py`.
"""

import time
from . import poses


def _check_safety(safety):
    if safety is not None and safety.tripped:
        raise RuntimeError("safety monitor tripped -- aborting gait")


def go_to(legs, pose_name_or_dict, duration_s=1.0, steps=20, safety=None):
    """Smoothly move into a named pose or a custom angle dict."""
    target = (poses.POSES[pose_name_or_dict]
              if isinstance(pose_name_or_dict, str)
              else pose_name_or_dict)
    poses.interpolate(legs, target, duration_s=duration_s, steps=steps)
    _check_safety(safety)


# ---------------------------------------------------------------------------
# Gait 1: shuffle / yaw-swing walk (no foot lift)
# ---------------------------------------------------------------------------

def shuffle_walk(legs, *, steps: int = 4, yaw_deg: float = 12.0,
                 lean_deg: float = 8.0, frame_s: float = 0.4,
                 safety=None):
    """
    Both feet stay grounded. Each "step":
        1. lean forward slightly (ankle pitch reduction)
        2. scissor hips: front leg yaws forward, rear leg yaws back
        3. straighten ankles -> body lurches forward
        4. mirror

    Tunables:
        yaw_deg    how much each hip yaws per step
        lean_deg   how much ankle pitch to relax (toes-up reduction)
        frame_s    seconds per keyframe (lower = faster, less stable)
    """
    base = poses.POSES["ready"]
    go_to(legs, base, duration_s=1.0, safety=safety)

    # Convenience builder
    def frame(left_yaw, right_yaw, ankle_lean):
        f = dict(base)
        f["LHY"] = base["LHY"] + left_yaw
        f["RHY"] = base["RHY"] + right_yaw
        f["LAP"] = base["LAP"] + ankle_lean
        f["RAP"] = base["RAP"] + ankle_lean
        return f

    for i in range(steps):
        left_leads = (i % 2 == 0)

        # 1) lean forward (ankle dorsiflexion = LESS negative ankle angle)
        legs.hold(frame(0, 0, +lean_deg), duration_ms=int(frame_s * 1000))
        _check_safety(safety)

        # 2) scissor hips
        if left_leads:
            legs.hold(frame(+yaw_deg, -yaw_deg, +lean_deg),
                      duration_ms=int(frame_s * 1000))
        else:
            legs.hold(frame(-yaw_deg, +yaw_deg, +lean_deg),
                      duration_ms=int(frame_s * 1000))
        _check_safety(safety)

        # 3) straighten ankles -> body shoves forward over the now-front foot
        legs.hold(frame(+yaw_deg if left_leads else -yaw_deg,
                        -yaw_deg if left_leads else +yaw_deg,
                        0.0),
                  duration_ms=int(frame_s * 1000))
        _check_safety(safety)

        # 4) reset hips to neutral, ready for next step
        legs.hold(base, duration_ms=int(frame_s * 1000))
        _check_safety(safety)


# ---------------------------------------------------------------------------
# Gait 2: dynamic step with actual foot lift
# ---------------------------------------------------------------------------

def dynamic_step(legs, *, steps: int = 2, lift_deg: float = 25.0,
                 step_yaw_deg: float = 15.0, lean_deg: float = 10.0,
                 frame_s: float = 0.35, safety=None):
    """
    Lift swing leg (thigh-pitch up + knee bend) while planted leg pitches
    forward at the ankle. Without lateral DOF this WILL want to fall to
    the swing-leg side -- the forward speed has to be high enough that
    you re-plant before that happens.

    Treat this as your second-tier experiment. Get shuffle_walk solid first.
    """
    base = poses.POSES["ready"]
    go_to(legs, base, duration_s=1.0, safety=safety)

    def lifted(side: str, lift, yaw, lean):
        f = dict(base)
        # support side leans the body forward at the ankle
        sup = "R" if side == "L" else "L"
        f[f"{sup}AP"] = base[f"{sup}AP"] + lean
        # swing side lifts and yaws forward
        f[f"{side}TP"] = base[f"{side}TP"] + lift           # hip flexion
        f[f"{side}KP"] = base[f"{side}KP"] + lift           # knee bend = pull foot up
        f[f"{side}AP"] = base[f"{side}AP"] - lift / 2       # toe drop
        f[f"{side}HY"] = base[f"{side}HY"] + yaw            # swing forward
        f[f"{sup}HY"]  = base[f"{sup}HY"]  - yaw            # opposite hip rotates back
        return f

    def planted(side: str, yaw):
        f = dict(base)
        f[f"{side}HY"] = base[f"{side}HY"] + yaw
        sup = "R" if side == "L" else "L"
        f[f"{sup}HY"] = base[f"{sup}HY"] - yaw
        return f

    for i in range(steps):
        side = "L" if (i % 2 == 0) else "R"
        # 1) lift + lean
        legs.hold(lifted(side, lift_deg, step_yaw_deg / 2, lean_deg),
                  duration_ms=int(frame_s * 1000))
        _check_safety(safety)
        # 2) swing forward at full yaw
        legs.hold(lifted(side, lift_deg, step_yaw_deg, lean_deg),
                  duration_ms=int(frame_s * 1000))
        _check_safety(safety)
        # 3) plant: relax the lift, keep yaw
        legs.hold(planted(side, step_yaw_deg),
                  duration_ms=int(frame_s * 1000))
        _check_safety(safety)
        # 4) recenter hips
        legs.hold(base, duration_ms=int(frame_s * 1000))
        _check_safety(safety)
