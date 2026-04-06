import time
from pylx16a.lx16a import LX16A, ServoTimeoutError

# ---------- SETTINGS ----------
PORT = "/dev/ttyUSB0"   # change if your adapter shows another port
MOTOR_IDS = [1, 2, 3, 4, 5, 6]

# Safe-ish center test positions
CENTER = 500
OFFSET = 80            # small motion so robot does not jerk too hard
MOVE_TIME = 600        # milliseconds
PAUSE = 0.8            # seconds between moves
# -----------------------------

def safe_move(servo, position, move_time=MOVE_TIME):
    servo.moveTimeWrite(position, move_time)
    time.sleep(move_time / 1000 + 0.2)

def test_motor(motor_id):
    print(f"\nTesting motor ID {motor_id}...")

    try:
        servo = LX16A(motor_id)

        # Move to center first
        safe_move(servo, CENTER)
        print(f"  ID {motor_id}: moved to center ({CENTER})")

        # Small positive motion
        safe_move(servo, CENTER + OFFSET)
        print(f"  ID {motor_id}: moved to {CENTER + OFFSET}")

        # Back to center
        safe_move(servo, CENTER)
        print(f"  ID {motor_id}: returned to center")

        # Small negative motion
        safe_move(servo, CENTER - OFFSET)
        print(f"  ID {motor_id}: moved to {CENTER - OFFSET}")

        # Back to center
        safe_move(servo, CENTER)
        print(f"  ID {motor_id}: returned to center again")

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
