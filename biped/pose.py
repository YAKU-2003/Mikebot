# pose_finder.py
# Manual pose angle finder for LX-16A biped
# Left foot motor (ID 8) is excluded

import time

try:
    from lx16a import LX16A
except ImportError:
    from pylx16a.lx16a import LX16A

SERIAL_PORT = "/dev/ttyUSB0"

# Motor IDs
MOTORS = {
    1: "Right Hip",
    2: "Right Thigh",
    3: "Right Knee",
    4: "Right Foot",
    5: "Left Hip",
    6: "Left Thigh",
    7: "Left Knee",
    # 8 is dead, excluded
}

servos = {}

def clamp_angle(angle):
    return max(0, min(240, angle))

def print_help():
    print("\nCommands:")
    print("  list                    -> show motor IDs")
    print("  read                    -> read all current motor angles")
    print("  read <id>               -> read one motor angle")
    print("  move <id> <angle>       -> move one motor")
    print("  step <id> <delta>       -> move one motor by + or - amount")
    print("  pose                    -> read and print all angles in pose format")
    print("  stand                   -> save current angles as stand pose")
    print("  crouch                  -> save current angles as crouch pose")
    print("  center                  -> move all active motors to 120")
    print("  quit                    -> exit\n")

def initialize():
    LX16A.initialize(SERIAL_PORT)
    for motor_id in MOTORS:
        try:
            servos[motor_id] = LX16A(motor_id)
            print(f"Connected: ID {motor_id} -> {MOTORS[motor_id]}")
        except Exception as e:
            print(f"Could not connect to motor {motor_id}: {e}")

def read_angle(motor_id):
    try:
        return servos[motor_id].get_physical_angle()
    except Exception as e:
        print(f"Read failed for motor {motor_id}: {e}")
        return None

def move_motor(motor_id, angle, move_time=500):
    angle = clamp_angle(angle)
    try:
        servos[motor_id].move(angle, time=move_time)
        print(f"Moved ID {motor_id} ({MOTORS[motor_id]}) -> {angle:.1f}")
    except Exception as e:
        print(f"Move failed for motor {motor_id}: {e}")

def read_all():
    print("\nCurrent angles:")
    for motor_id in MOTORS:
        if motor_id in servos:
            ang = read_angle(motor_id)
            if ang is not None:
                print(f"  ID {motor_id} ({MOTORS[motor_id]}): {ang:.2f}")
    print()

def current_pose_dict():
    pose = {}
    for motor_id in MOTORS:
        if motor_id in servos:
            ang = read_angle(motor_id)
            if ang is not None:
                pose[motor_id] = round(ang, 2)
    return pose

def print_pose_block(pose_name, pose):
    print(f"\n{pose_name} = {{")
    for motor_id in sorted(pose.keys()):
        print(f"    {motor_id}: {pose[motor_id]},  # {MOTORS[motor_id]}")
    print("}\n")

def move_all_center():
    print("Moving all active motors to 120...")
    for motor_id in MOTORS:
        if motor_id in servos:
            move_motor(motor_id, 120, move_time=700)
            time.sleep(0.05)

def main():
    initialize()
    print_help()

    saved_stand = None
    saved_crouch = None

    while True:
        try:
            cmd = input("pose-finder> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not cmd:
            continue

        parts = cmd.split()

        if parts[0] == "quit":
            print("Exiting.")
            break

        elif parts[0] == "list":
            print("\nAvailable motors:")
            for motor_id, name in MOTORS.items():
                print(f"  {motor_id}: {name}")
            print()

        elif parts[0] == "read":
            if len(parts) == 1:
                read_all()
            elif len(parts) == 2:
                try:
                    motor_id = int(parts[1])
                    if motor_id not in servos:
                        print("Motor not available.")
                        continue
                    ang = read_angle(motor_id)
                    if ang is not None:
                        print(f"ID {motor_id} ({MOTORS[motor_id]}): {ang:.2f}")
                except ValueError:
                    print("Usage: read <id>")
            else:
                print("Usage: read OR read <id>")

        elif parts[0] == "move":
            if len(parts) != 3:
                print("Usage: move <id> <angle>")
                continue
            try:
                motor_id = int(parts[1])
                angle = float(parts[2])
                if motor_id not in servos:
                    print("Motor not available.")
                    continue
                move_motor(motor_id, angle)
            except ValueError:
                print("Usage: move <id> <angle>")

        elif parts[0] == "step":
            if len(parts) != 3:
                print("Usage: step <id> <delta>")
                continue
            try:
                motor_id = int(parts[1])
                delta = float(parts[2])
                if motor_id not in servos:
                    print("Motor not available.")
                    continue
                current = read_angle(motor_id)
                if current is None:
                    continue
                move_motor(motor_id, current + delta)
            except ValueError:
                print("Usage: step <id> <delta>")

        elif parts[0] == "pose":
            pose = current_pose_dict()
            print_pose_block("CURRENT_POSE", pose)

        elif parts[0] == "stand":
            saved_stand = current_pose_dict()
            print_pose_block("STAND_POSE", saved_stand)

        elif parts[0] == "crouch":
            saved_crouch = current_pose_dict()
            print_pose_block("CROUCH_POSE", saved_crouch)

        elif parts[0] == "center":
            move_all_center()

        else:
            print("Unknown command.")
            print_help()

    print("\nSaved poses summary:")
    if saved_stand:
        print_pose_block("STAND_POSE", saved_stand)
    if saved_crouch:
        print_pose_block("CROUCH_POSE", saved_crouch)

if __name__ == "__main__":
    main()