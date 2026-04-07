import time
import csv
from pylx16a.lx16a import LX16A, ServoTimeoutError

PORT = "/dev/ttyUSB0"
CSV_FILE = "stand_crouch_angles_log.csv"

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

MOVE_TIME_STAND = 1.2   # seconds
MOVE_TIME_CROUCH = 1.0  # seconds
PAUSE_BETWEEN = 0.4
DT = 0.03               # logging timestep


def clamp_angle(angle):
    return max(0, min(240, angle))


def build_servos():
    servos = {}
    for motor_id in STAND_POSE:
        servos[motor_id] = LX16A(motor_id)
    return servos


def read_current_pose(servos):
    pose = {}
    for motor_id, servo in servos.items():
        pose[motor_id] = servo.get_physical_angle()
    return pose


def send_pose(servos, pose):
    for motor_id, servo in servos.items():
        servo.move(clamp_angle(pose[motor_id]))


def interpolate_pose(start_pose, end_pose, alpha):
    pose = {}
    for motor_id in start_pose:
        pose[motor_id] = start_pose[motor_id] + (end_pose[motor_id] - start_pose[motor_id]) * alpha
    return pose


def move_and_log(writer, servos, start_pose, end_pose, duration, start_time):
    steps = max(1, int(duration / DT))

    for i in range(steps + 1):
        alpha = i / steps
        pose = interpolate_pose(start_pose, end_pose, alpha)
        send_pose(servos, pose)

        t = time.time() - start_time
        writer.writerow([
            round(t, 4),
            round(pose[1], 4),
            round(pose[2], 4),
            round(pose[3], 4),
            round(pose[4], 4),
            round(pose[5], 4),
            round(pose[6], 4),
        ])

        time.sleep(DT)

    return dict(end_pose)


def log_hold_pose(writer, pose, start_time, hold_time):
    steps = max(1, int(hold_time / DT))
    for _ in range(steps):
        t = time.time() - start_time
        writer.writerow([
            round(t, 4),
            round(pose[1], 4),
            round(pose[2], 4),
            round(pose[3], 4),
            round(pose[4], 4),
            round(pose[5], 4),
            round(pose[6], 4),
        ])
        time.sleep(DT)


def main():
    print("=== STAND-CROUCH-STAND ANGLE LOGGER ===")
    print(f"CSV output: {CSV_FILE}")

    try:
        LX16A.initialize(PORT)
        print(f"Bus initialized on {PORT}")
    except Exception as e:
        print(f"Failed to initialize bus: {e}")
        return

    try:
        servos = build_servos()
        current_pose = read_current_pose(servos)

        with open(CSV_FILE, "w", newline="") as f:
            writer = csv.writer(f)

            # Header
            writer.writerow([
                "time_s",
                "right_hip",
                "right_knee",
                "right_ankle",
                "left_hip",
                "left_knee",
                "left_ankle"
            ])

            motion_start_time = time.time()

            print("Moving from current pose to stand pose...")
            current_pose = move_and_log(
                writer, servos, current_pose, STAND_POSE, MOVE_TIME_STAND, motion_start_time
            )
            log_hold_pose(writer, current_pose, motion_start_time, PAUSE_BETWEEN)

            print("Moving from stand pose to crouch pose...")
            current_pose = move_and_log(
                writer, servos, current_pose, CROUCH_POSE, MOVE_TIME_CROUCH, motion_start_time
            )
            log_hold_pose(writer, current_pose, motion_start_time, PAUSE_BETWEEN)

            print("Moving from crouch pose back to stand pose...")
            current_pose = move_and_log(
                writer, servos, current_pose, STAND_POSE, MOVE_TIME_STAND, motion_start_time
            )
            log_hold_pose(writer, current_pose, motion_start_time, PAUSE_BETWEEN)

        print(f"Done. Angle log saved to {CSV_FILE}")

    except ServoTimeoutError as e:
        print(f"Servo timeout: {e}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
