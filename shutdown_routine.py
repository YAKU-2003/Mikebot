import time
from pylx16a.lx16a import LX16A, ServoTimeoutError

PORT = "/dev/ttyUSB0"

MOTOR_IDS = [1, 2, 3, 4, 5, 6]

MOVE_TIME = 800  # smooth return

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
        # Load all servos
        for motor_id in MOTOR_IDS:
            servos[motor_id] = LX16A(motor_id)

        # Read current positions
        current_pose = {}
        for motor_id, servo in servos.items():
            current_pose[motor_id] = servo.get_physical_angle()

        print("Moving to neutral pose...")

        # Neutral pose = current pose (safe fallback)
        # You can customize this later if you want a specific stance
        for motor_id, servo in servos.items():
            servo.move(clamp_angle(current_pose[motor_id]), time=MOVE_TIME)

        time.sleep(MOVE_TIME / 1000 + 0.3)

        print("Stopping all motors...")

        # Stop all motors
        for servo in servos.values():
            try:
                servo.moveStop()
            except:
                pass

        print("Disabling torque...")

        # Disable torque (so you can move robot freely)
        for servo in servos.values():
            try:
                servo.disable_torque()
            except:
                pass

        print("Shutdown complete. Robot is safe.")

    except ServoTimeoutError as e:
        print(f"Servo timeout: {e}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()