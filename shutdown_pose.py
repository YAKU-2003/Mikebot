import time
from pylx16a.lx16a import LX16A, ServoTimeoutError

PORT = "/dev/ttyUSB0"

# Desired shutdown pose
SHUTDOWN_POSE = {
    1: 125.04,   # right_hip
    2: 219.36,   # right_knee
    3: 175.92,   # right_ankle
    4: 133.20,   # left_hip
    5: 211.68,   # left_knee
    6: 180.00,   # left_ankle
}

MOVE_TIME = 1200  # milliseconds


def clamp_angle(angle):
    return max(0, min(240, angle))


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
        # Create servo objects
        for motor_id in SHUTDOWN_POSE:
            servos[motor_id] = LX16A(motor_id)

        print("Moving all motors to shutdown pose simultaneously...")

        # Send all move commands quickly so they move together
        for motor_id, servo in servos.items():
            servo.move(clamp_angle(SHUTDOWN_POSE[motor_id]), time=MOVE_TIME)

        # Wait for motion to complete
        time.sleep(MOVE_TIME / 1000 + 0.3)

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

        print("Shutdown complete. Robot moved to desired rest pose.")

    except ServoTimeoutError as e:
        print(f"Servo timeout: {e}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()