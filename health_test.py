import time
from pylx16a.lx16a import LX16A, ServoTimeoutError

PORT = "/dev/ttyUSB0"
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

OFFSET = 12       # degrees
MOVE_TIME = 700   # milliseconds
PAUSE = 0.8

def safe_move(servo, angle, move_time=MOVE_TIME):
    servo.move(angle, time=move_time)
    time.sleep(move_time / 1000 + 0.2)

def clamp_angle(angle):
    return max(0, min(240, angle))

def test_motor(motor_id):
    print(f"\nTesting motor ID {motor_id}...")

    try:
        servo = LX16A(motor_id)

        # Read original position
        original_angle = servo.get_physical_angle()
        print(f"  ID {motor_id}: original angle = {original_angle:.2f} deg")

        # Small test move
        forward_angle = clamp_angle(original_angle + OFFSET)
        backward_angle = clamp_angle(original_angle - OFFSET)

        safe_move(servo, forward_angle)
        print(f"  ID {motor_id}: moved to {forward_angle:.2f} deg")

        safe_move(servo, original_angle)
        print(f"  ID {motor_id}: returned to original position")

        safe_move(servo, backward_angle)
        print(f"  ID {motor_id}: moved to {backward_angle:.2f} deg")

        safe_move(servo, original_angle)
        print(f"  ID {motor_id}: returned to original position again")

        print(f"  RESULT: ID {motor_id} PASSED")
        return True

    except ServoTimeoutError:
        print(f"  RESULT: ID {motor_id} FAILED - no response / timeout")
        return False

    except Exception as e:
        print(f"  RESULT: ID {motor_id} FAILED - {e}")
        return False

def main():
    print("=== LX-16A ROBOT HEALTH TEST ROUTINE ===")
    print("Initializing bus...")

    try:
        LX16A.initialize(PORT)
        print(f"Bus initialized on {PORT}")
    except Exception as e:
        print(f"Could not initialize bus on {PORT}")
        print(f"Error: {e}")
        return

    passed = []
    failed = []

    print("\nStarting motor health test...\n")
    time.sleep(1)

    for motor_id in MOTOR_IDS:
        ok = test_motor(motor_id)
        if ok:
            passed.append(motor_id)
        else:
            failed.append(motor_id)

        time.sleep(PAUSE)

    print("\n=== HEALTH TEST SUMMARY ===")
    print(f"Passed motors: {passed if passed else 'None'}")
    print(f"Failed motors: {failed if failed else 'None'}")

    if not failed:
        print("\nAll motors are responding correctly.")
    else:
        print("\nSome motors failed. Check:")
        print("- wiring")
        print("- power supply")
        print("- servo ID")
        print("- serial port")
        print("- loose connectors")

if __name__ == "__main__":
    main()