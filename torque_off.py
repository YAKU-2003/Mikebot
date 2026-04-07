import time
from pylx16a.lx16a import LX16A, ServoTimeoutError

PORT = "/dev/ttyUSB0"
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

def main():
    print("=== TORQUE OFF ROUTINE ===")

    try:
        LX16A.initialize(PORT)
        print(f"Bus initialized on {PORT}")
    except Exception as e:
        print(f"Failed to initialize bus: {e}")
        return

    for motor_id in MOTOR_IDS:
        try:
            servo = LX16A(motor_id)

            # Most common method name in pylx16a
            servo.disable_torque()

            print(f"ID {motor_id}: torque disabled")
        except ServoTimeoutError:
            print(f"ID {motor_id}: no response / timeout")
        except Exception as e:
            print(f"ID {motor_id}: failed - {e}")

    print("\nAll possible motors processed.")
    print("You should now be able to move the joints by hand.")

if __name__ == "__main__":
    main()