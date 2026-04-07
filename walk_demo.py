import time
import math
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
# Gait tuning
# -----------------------------
STEP_COUNT = 8                 # number of gait cycles
CYCLE_TIME = 1.6               # seconds per full gait cycle
DT = 0.03                      # update interval

# Motion amplitudes
HIP_SHIFT = 5                  # reduced hip sway
KNEE_LIFT = 12
ANKLE_LIFT = 10

# Hip directions
RIGHT_HIP_OUTWARD_SIGN = +1    # motor 1: increase = outward
LEFT_HIP_OUTWARD_SIGN = -1     # motor 4: decrease = outward

# Knee/ankle directions
RIGHT_KNEE_SIGN = +1
RIGHT_ANKLE_SIGN = -1
LEFT_KNEE_SIGN = +1
LEFT_ANKLE_SIGN = -1           # using your updated left ankle sign


def clamp_angle(angle):
    return max(0, min(240, angle))


def build_servos():
    return {
        "right_hip": LX16A(RIGHT_HIP),
        "right_knee": LX16A(RIGHT_KNEE),
        "right_ankle": LX16A(RIGHT_ANKLE),
        "left_hip": LX16A(LEFT_HIP),
        "left_knee": LX16A(LEFT_KNEE),
        "left_ankle": LX16A(LEFT_ANKLE),
    }


def read_pose(servos):
    pose = {}
    for name, servo in servos.items():
        pose[name] = servo.get_physical_angle()
    return pose


def print_pose(title, pose):
    print(f"\n{title}")
    for k, v in pose.items():
        print(f"  {k}: {v:.2f} deg")


def send_pose(servos, pose):
    for name, servo in servos.items():
        servo.move(clamp_angle(pose[name]))


def smooth_move_to_pose(servos, current_pose, target_pose, duration=1.0, dt=0.03):
    steps = max(1, int(duration / dt))
    for i in range(1, steps + 1):
        alpha = i / steps
        pose = {}
        for name in current_pose:
            pose[name] = current_pose[name] + (target_pose[name] - current_pose[name]) * alpha
        send_pose(servos, pose)
        time.sleep(dt)

    for name in current_pose:
        current_pose[name] = target_pose[name]


def leg_swing_profile(phase):
    """
    Smooth swing profile from 0 to 1.
    Only active during half the cycle.
    """
    s = math.sin(phase)
    return max(0.0, s)


def main():
    print("=== FLUID SUPPORTED WALKING DEMO ===")
    print("Robot should be lightly supported by hand.")
    print("This version uses a smooth 50% phase-shifted gait.\n")

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

        # Move gently into start pose
        smooth_move_to_pose(servos, current_pose, start_pose, duration=1.0, dt=DT)
        time.sleep(0.5)

        total_time = STEP_COUNT * CYCLE_TIME
        start_time = time.time()

        while True:
            t = time.time() - start_time
            if t >= total_time:
                break

            # Main gait phase
            phase_r = 2 * math.pi * (t / CYCLE_TIME)
            phase_l = phase_r + math.pi   # 50% phase offset

            # Swing profiles: active when sin() > 0
            swing_r = leg_swing_profile(phase_r)
            swing_l = leg_swing_profile(phase_l)

            # Smooth hip sway
            hip_r = start_pose["right_hip"] + RIGHT_HIP_OUTWARD_SIGN * HIP_SHIFT * math.sin(phase_r)
            hip_l = start_pose["left_hip"] + LEFT_HIP_OUTWARD_SIGN * HIP_SHIFT * math.sin(phase_l)

            # Knee and ankle motion
            # Only one leg swings strongly at a time, but the other begins halfway through the cycle
            right_knee = start_pose["right_knee"] + RIGHT_KNEE_SIGN * KNEE_LIFT * swing_r
            right_ankle = start_pose["right_ankle"] + RIGHT_ANKLE_SIGN * ANKLE_LIFT * swing_r

            left_knee = start_pose["left_knee"] + LEFT_KNEE_SIGN * KNEE_LIFT * swing_l
            left_ankle = start_pose["left_ankle"] + LEFT_ANKLE_SIGN * ANKLE_LIFT * swing_l

            pose = {
                "right_hip": hip_r,
                "right_knee": right_knee,
                "right_ankle": right_ankle,
                "left_hip": hip_l,
                "left_knee": left_knee,
                "left_ankle": left_ankle,
            }

            send_pose(servos, pose)
            time.sleep(DT)

        print("\nWalking demo complete. Returning to start pose.")
        end_pose = read_pose(servos)
        smooth_move_to_pose(servos, end_pose, start_pose, duration=1.0, dt=DT)
        print("Done.")

    except ServoTimeoutError as e:
        print(f"Servo timeout: {e}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()