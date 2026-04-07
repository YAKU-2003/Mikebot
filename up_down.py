import time
from pylx16a.lx16a import LX16A, ServoTimeoutError

PORT = "/dev/ttyUSB0"

# Stand pose
STAND_POSE = {
    1: 125.04,   # right_hip
    2: 140.00,   # right_knee
    3: 100.00,   # right_ankle
    4: 133.20,   # left_hip
    5: 140.00,   # left_knee
    6: 100.00,   # left_ankle
}

# Crouch pose
CROUCH_POSE = {
    1: 125.04,   # right_hip
    2: 219.36,   # right_knee
    3: 175.92,   # right_ankle
    4: 133.20,   # left_hip
    5: 211.68,   # left_knee
    6: 180.00,   # left_ankle
}

MOVE_TIME_STAND = 1200
MOVE_TIME_CROUCH = 1000
PAUSE_BETWEEN = 0.4


def clamp_angle(angle):
    return max(0, min(240, angle))


def move_all(servos, target_pose, move_time):
    for motor_id, servo in servos.items():
        servo.move(clamp_angle(target_pose[motor_id]), time=move_time)


def main():
    print("=== ROBOT SHUTDOWN ROUTINE ===")

    try:
        LX16A.initialize(PORT)
        print(f"Bus initialized on {PORT}")
    except Exception as e:
        print(f"Failed to initialize bus: {e}")
        return

    servos = {}

    try:
        for motor_id in STAND_POSE:
            servos[motor_id] = LX16A(motor_id)

        print("Moving from current pose to stand pose...")
        move_all(servos, STAND_POSE, MOVE_TIME_STAND)
        time.sleep(MOVE_TIME_STAND / 1000 + PAUSE_BETWEEN)

        print("Moving down to crouch pose...")
        move_all(servos, CROUCH_POSE, MOVE_TIME_CROUCH)
        time.sleep(MOVE_TIME_CROUCH / 1000 + PAUSE_BETWEEN)

        print("Standing back up...")
        move_all(servos, STAND_POSE, MOVE_TIME_STAND)
        time.sleep(MOVE_TIME_STAND / 1000 + PAUSE_BETWEEN)

        print("Stopping all motors...")
        for servo in servos.values():
            try:
                servo.moveStop()
            except Exception:
                pass

        print("Disabling torque...")
        for servo in servos.values():
            try:
                servo.disable_torque()
            except Exception:
                pass

        print("Shutdown complete. Robot ended in stand pose.")

    except ServoTimeoutError as e:
        print(f"Servo timeout: {e}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()