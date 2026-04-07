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

# Motion timing
MOVE_TIME_HIP = 600
MOVE_TIME_LEG = 500
PAUSE_SHORT = 0.25
PAUSE_LONG = 0.5

# Motion amounts
HIP_SHIFT = 12
KNEE_BEND = 10
ANKLE_BEND = 8

# Confirmed hip directions
RIGHT_HIP_OUTWARD_SIGN = +1
LEFT_HIP_OUTWARD_SIGN = -1

# These may need tuning later depending on your linkage direction
RIGHT_KNEE_SIGN = +1
RIGHT_ANKLE_SIGN = -1
LEFT_KNEE_SIGN = -1
LEFT_ANKLE_SIGN = +1


def clamp_angle(angle):
    return max(0, min(240, angle))


def safe_move(servo, angle, move_time):
    servo.move(clamp_angle(angle), time=move_time)


def wait_done(move_time):
    time.sleep(move_time / 1000 + 0.15)


def get_servo(motor_id):
    return LX16A(motor_id)


def read_start_pose(servos):
    pose = {}
    for name, servo in servos.items():
        pose[name] = servo.get_physical_angle()
    return pose


def print_pose(pose, title="Pose"):
    print(f"\n{title}:")
    for k, v in pose.items():
        print(f"  {k}: {v:.2f} deg")


def return_all_to_start(servos, start_pose):
    print("\nReturning all motors to starting pose...")
    for name, servo in servos.items():
        safe_move(servo, start_pose[name], 700)
    wait_done(700)


def shift_weight_to_left(servos, start_pose):
    print("\nShifting weight to LEFT leg...")

    left_hip_target = start_pose["left_hip"] + LEFT_HIP_OUTWARD_SIGN * HIP_SHIFT
    right_hip_target = start_pose["right_hip"]

    safe_move(servos["left_hip"], left_hip_target, MOVE_TIME_HIP)
    safe_move(servos["right_hip"], right_hip_target, MOVE_TIME_HIP)
    wait_done(MOVE_TIME_HIP)


def lift_right_leg(servos, start_pose):
    print("Lifting RIGHT leg...")

    right_knee_target = start_pose["right_knee"] + RIGHT_KNEE_SIGN * KNEE_BEND
    right_ankle_target = start_pose["right_ankle"] + RIGHT_ANKLE_SIGN * ANKLE_BEND

    safe_move(servos["right_knee"], right_knee_target, MOVE_TIME_LEG)
    safe_move(servos["right_ankle"], right_ankle_target, MOVE_TIME_LEG)
    wait_done(MOVE_TIME_LEG)

    # Return to original position
    safe_move(servos["right_knee"], start_pose["right_knee"], MOVE_TIME_LEG)
    safe_move(servos["right_ankle"], start_pose["right_ankle"], MOVE_TIME_LEG)
    wait_done(MOVE_TIME_LEG)


def shift_weight_to_right(servos, start_pose):
    print("\nShifting weight to RIGHT leg...")

    right_hip_target = start_pose["right_hip"] + RIGHT_HIP_OUTWARD_SIGN * HIP_SHIFT
    left_hip_target = start_pose["left_hip"]

    safe_move(servos["right_hip"], right_hip_target, MOVE_TIME_HIP)
    safe_move(servos["left_hip"], left_hip_target, MOVE_TIME_HIP)
    wait_done(MOVE_TIME_HIP)


def lift_left_leg(servos, start_pose):
    print("Lifting LEFT leg...")

    left_knee_target = start_pose["left_knee"] + LEFT_KNEE_SIGN * KNEE_BEND
    left_ankle_target = start_pose["left_ankle"] + LEFT_ANKLE_SIGN * ANKLE_BEND

    safe_move(servos["left_knee"], left_knee_target, MOVE_TIME_LEG)
    safe_move(servos["left_ankle"], left_ankle_target, MOVE_TIME_LEG)
    wait_done(MOVE_TIME_LEG)

    # Return to original position
    safe_move(servos["left_knee"], start_pose["left_knee"], MOVE_TIME_LEG)
    safe_move(servos["left_ankle"], start_pose["left_ankle"], MOVE_TIME_LEG)
    wait_done(MOVE_TIME_LEG)


def main():
    print("=== BIPED BALANCE TEST ===")

    try:
        LX16A.initialize(PORT)
        print(f"Bus initialized on {PORT}")
    except Exception as e:
        print(f"Failed to initialize bus: {e}")
        return

    try:
        servos = {
            "right_hip": get_servo(RIGHT_HIP),
            "right_knee": get_servo(RIGHT_KNEE),
            "right_ankle": get_servo(RIGHT_ANKLE),
            "left_hip": get_servo(LEFT_HIP),
            "left_knee": get_servo(LEFT_KNEE),
            "left_ankle": get_servo(LEFT_ANKLE),
        }

        start_pose = read_start_pose(servos)
        print_pose(start_pose, "Starting pose")

        print("\nSupport the robot lightly for the first run.")
        time.sleep(2)

        # Support on left, swing right
        shift_weight_to_left(servos, start_pose)
        lift_right_leg(servos, start_pose)
        return_all_to_start(servos, start_pose)
        time.sleep(PAUSE_SHORT)

        # Support on right, swing left
        shift_weight_to_right(servos, start_pose)
        lift_left_leg(servos, start_pose)
        return_all_to_start(servos, start_pose)

        print("\nBalance test complete.")

    except ServoTimeoutError as e:
        print(f"Servo timeout: {e}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()