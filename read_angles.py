import time
from pylx16a.lx16a import LX16A, ServoTimeoutError

PORT = "/dev/ttyUSB0"
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

MOTOR_NAMES = {
    1: "right_hip",
    2: "right_knee",
    3: "right_ankle",
    4: "left_hip",
    5: "left_knee",
    6: "left_ankle",
}

def main():
    print("=== CURRENT MOTOR ANGLE READER ===")

    try:
        LX16A.initialize(PORT)
        print(f"Bus initialized on {PORT}")
    except Exception as e:
        print(f"Failed to initialize bus: {e}")
        return

    servos = {}

    try:
        for motor_id in MOTOR_IDS:
            servos[motor_id] = LX16A(motor_id)

        while True:
            print("\nCurrent motor angles:")
            for motor_id in MOTOR_IDS:
                try:
                    angle = servos[motor_id].get_physical_angle()
                    print(f"  ID {motor_id} ({MOTOR_NAMES[motor_id]}): {angle:.2f} deg")
                except ServoTimeoutError:
                    print(f"  ID {motor_id} ({MOTOR_NAMES[motor_id]}): timeout")
                except Exception as e:
                    print(f"  ID {motor_id} ({MOTOR_NAMES[motor_id]}): error - {e}")

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped angle reader.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()