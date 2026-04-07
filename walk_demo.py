import time
from pylx16a.lx16a import LX16A, ServoTimeoutError

PORT = "/dev/ttyUSB0"

# Motor IDs
RIGHT_HIP = 1
RIGHT_KNEE = 2
RIGHT_ANKLE = 3
LEFT_HIP = 4
LEFT_KNEE = 5
LEFT_ANKLE = 6

# -----------------------------
# Tuning parameters
# -----------------------------
STEP_COUNT = 6              # number of full walking cycles
HIP_SHIFT = 18              # outward shift for support leg
KNEE_LIFT = 16              # knee bend for swing leg
ANKLE_LIFT = 12             # ankle compensation for swing leg
STEP_FORWARD_EXTRA = 4      # tiny extra motion to make it look more like stepping

SEGMENTS = 8                # more = smoother motion
SEGMENT_TIME = 0.05         # seconds per interpolation segment
PHASE_PAUSE = 0.08          # tiny pause between phases

# Confirmed outward directions
RIGHT_HIP_OUTWARD_SIGN = +1   # increasing motor 1 angle moves outward
LEFT_HIP_OUTWARD_SIGN = -1    # decreasing motor 4 angle moves outward

# Knee/ankle directions
# Flip signs if a joint bends the wrong way
RIGHT_KNEE_SIGN = +1
RIGHT_ANKLE_SIGN = -1
LEFT_KNEE_SIGN = -1
LEFT_ANKLE_SIGN = +1


def clamp_angle(angle):
    return max(0, min(240, angle))


def smooth_move(servos, current_pose, target_pose, segments=SEGMENTS, dt=SEGMENT_TIME):
    """
    Smoothly interpolate from current_pose to target_pose.
    current_pose and target_pose are dicts with same keys.
    """
    for i in range(1, segments + 1):
        alpha = i / segments
        for name, servo in servos.items():
            start_angle = current_pose[name]
            end_angle = target_pose[name]
            angle = start_angle + (end_angle - start_angle) * alpha
            servo.move(clamp_angle(angle))
        time.sleep(dt)

    for name in current_pose:
        current_pose[name] = target_pose[name]


def read_pose(servos):
    pose = {}
    for name, servo in servos.items():
        pose[name] = servo.get_physical_angle()
    return pose


def print_pose(title, pose):
    print(f"\n{title}")
    for k, v in pose.items():
        print(f"  {k}: {v:.2f} deg")


def build_servos():
    return {
        "right_hip": LX16A(RIGHT_HIP),
        "right_knee": LX16A(RIGHT_KNEE),
        "right_ankle": LX16A(RIGHT_ANKLE),
        "left_hip": LX16A(LEFT_HIP),
        "left_knee": LX16A(LEFT_KNEE),
        "left_ankle": LX16A(LEFT_ANKLE),
    }


def center_pose(start_pose):
    return dict(start_pose)


def shift_left_pose(base_pose):
    pose = dict(base_pose)
    pose["left_hip"] = base_pose["left_hip"] + LEFT_HIP_OUTWARD_SIGN * HIP_SHIFT
    pose["right_hip"] = base_pose["right_hip"]
    return pose


def shift_right_pose(base_pose):
    pose = dict(base_pose)
    pose["right_hip"] = base_pose["right_hip"] + RIGHT_HIP_OUTWARD_SIGN * HIP_SHIFT
    pose["left_hip"] = base_pose["left_hip"]
    return pose


def right_swing_pose(base_pose):
    pose = dict(base_pose)

    # support stays on left leg
    pose["left_hip"] = base_pose["left_hip"] + LEFT_HIP_OUTWARD_SIGN * HIP_SHIFT

    # right leg swings
    pose["right_knee"] = base_pose["right_knee"] + RIGHT_KNEE_SIGN * KNEE_LIFT
    pose["right_ankle"] = base_pose["right_ankle"] + RIGHT_ANKLE_SIGN * ANKLE_LIFT

    # tiny extra accent so it looks more like stepping
    pose["right_knee"] += RIGHT_KNEE_SIGN * STEP_FORWARD_EXTRA

    return pose


def right_place_pose(base_pose):
    pose = dict(base_pose)

    # keep support shift while placing down
    pose["left_hip"] = base_pose["left_hip"] + LEFT_HIP_OUTWARD_SIGN * HIP_SHIFT

    # return right leg toward neutral
    pose["right_knee"] = base_pose["right_knee"]
    pose["right_ankle"] = base_pose["right_ankle"]

    return pose


def left_swing_pose(base_pose):
    pose = dict(base_pose)

    # support stays on right leg
    pose["right_hip"] = base_pose["right_hip"] + RIGHT_HIP_OUTWARD_SIGN * HIP_SHIFT

    # left leg swings
    pose["left_knee"] = base_pose["left_knee"] + LEFT_KNEE_SIGN * KNEE_LIFT
    pose["left_ankle"] = base_pose["left_ankle"] + LEFT_ANKLE_SIGN * ANKLE_LIFT

    # tiny extra accent
    pose["left_knee"] += LEFT_KNEE_SIGN * STEP_FORWARD_EXTRA

    return pose


def left_place_pose(base_pose):
    pose = dict(base_pose)

    # keep support shift while placing down
    pose["right_hip"] = base_pose["right_hip"] + RIGHT_HIP_OUTWARD_SIGN * HIP_SHIFT

    # return left leg toward neutral
    pose["left_knee"] = base_pose["left_knee"]
    pose["left_ankle"] = base_pose["left_ankle"]

    return pose


def main():
    print("=== SMOOTH SUPPORTED WALKING DEMO ===")
    print("Keep the robot lightly supported by hand for this test.")
    print("Make sure the feet can move freely and nothing is jammed.\n")

    try:
        LX16A.initialize(PORT)
        print(f"Bus initialized on {PORT}")
    except Exception as e:
        print(f"Failed to initialize bus: {e}")
        return

    try:
        servos = build_servos()
        start_pose = read_pose(servos)
        current_pose = dict(start_pose)

        print_pose("Starting pose:", start_pose)
        time.sleep(1.5)

        # Start centered
        smooth_move(servos, current_pose, center_pose(start_pose))
        time.sleep(PHASE_PAUSE)

        for step in range(STEP_COUNT):
            print(f"\nCycle {step + 1}/{STEP_COUNT}")

            # 1. Shift to left support
            target = shift_left_pose(start_pose)
            smooth_move(servos, current_pose, target)
            time.sleep(PHASE_PAUSE)

            # 2. Swing right leg
            target = right_swing_pose(start_pose)
            smooth_move(servos, current_pose, target)
            time.sleep(PHASE_PAUSE)

            # 3. Place right leg
            target = right_place_pose(start_pose)
            smooth_move(servos, current_pose, target)
            time.sleep(PHASE_PAUSE)

            # 4. Return to center
            target = center_pose(start_pose)
            smooth_move(servos, current_pose, target)
            time.sleep(PHASE_PAUSE)

            # 5. Shift to right support
            target = shift_right_pose(start_pose)
            smooth_move(servos, current_pose, target)
            time.sleep(PHASE_PAUSE)

            # 6. Swing left leg
            target = left_swing_pose(start_pose)
            smooth_move(servos, current_pose, target)
            time.sleep(PHASE_PAUSE)

            # 7. Place left leg
            target = left_place_pose(start_pose)
            smooth_move(servos, current_pose, target)
            time.sleep(PHASE_PAUSE)

            # 8. Return to center
            target = center_pose(start_pose)
            smooth_move(servos, current_pose, target)
            time.sleep(PHASE_PAUSE)

        print("\nWalking demo complete. Returning to start pose.")
        smooth_move(servos, current_pose, start_pose)
        print("Done.")

    except ServoTimeoutError as e:
        print(f"Servo timeout: {e}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()