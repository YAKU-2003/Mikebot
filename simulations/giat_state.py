import pybullet as p
import pybullet_data
import time

# -----------------------------
# Setup
# -----------------------------
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")

robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.40],
    useFixedBase=False
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.6,
    cameraYaw=50,
    cameraPitch=-15,
    cameraTargetPosition=[0, 0, 0.22]
)

# -----------------------------
# Joint mapping
# -----------------------------
RIGHT_HIP = 1
RIGHT_THIGH = 3
RIGHT_KNEE = 5
RIGHT_FOOT = 7

LEFT_HIP = 9
LEFT_THIGH = 11
LEFT_KNEE = 13
LEFT_FOOT = 15

ALL_JOINTS = [
    RIGHT_HIP, RIGHT_THIGH, RIGHT_KNEE, RIGHT_FOOT,
    LEFT_HIP, LEFT_THIGH, LEFT_KNEE, LEFT_FOOT
]

# -----------------------------
# Pose definitions
# -----------------------------
POSE_STAND = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.05,
    RIGHT_KNEE: 0.0,
    RIGHT_FOOT: 0.0,
    LEFT_HIP: 0.0,
    LEFT_THIGH: 0.05,
    LEFT_KNEE: 0.0,
    LEFT_FOOT: 0.0
}

POSE_RIGHT_THIGH_LIFT = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.18,
    RIGHT_KNEE: 0.0,
    RIGHT_FOOT: 0.0,
    LEFT_HIP: 0.0,
    LEFT_THIGH: 0.05,
    LEFT_KNEE: 0.0,
    LEFT_FOOT: 0.0
}

POSE_RIGHT_KNEE_BEND = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.18,
    RIGHT_KNEE: -0.22,
    RIGHT_FOOT: 0.0,
    LEFT_HIP: 0.0,
    LEFT_THIGH: 0.05,
    LEFT_KNEE: 0.0,
    LEFT_FOOT: 0.0
}

POSE_LEFT_THIGH_LIFT = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.05,
    RIGHT_KNEE: 0.0,
    RIGHT_FOOT: 0.0,
    LEFT_HIP: 0.0,
    LEFT_THIGH: 0.18,
    LEFT_KNEE: 0.0,
    LEFT_FOOT: 0.0
}

POSE_LEFT_KNEE_BEND = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.05,
    RIGHT_KNEE: 0.0,
    RIGHT_FOOT: 0.0,
    LEFT_HIP: 0.0,
    LEFT_THIGH: 0.18,
    LEFT_KNEE: -0.22,
    LEFT_FOOT: 0.0
}

# -----------------------------
# Helpers
# -----------------------------
def reset_to_pose(pose: dict[int, float]) -> None:
    for joint, angle in pose.items():
        p.resetJointState(robot, joint, angle)

def apply_pose(pose: dict[int, float], force: float = 280, max_vel: float = 1.2) -> None:
    for joint, angle in pose.items():
        # lock feet harder so they don't flap around
        if joint in (RIGHT_FOOT, LEFT_FOOT):
            joint_force = 500
            joint_vel = 0.8
        else:
            joint_force = force
            joint_vel = max_vel

        p.setJointMotorControl2(
            robot,
            joint,
            p.POSITION_CONTROL,
            targetPosition=angle,
            force=joint_force,
            maxVelocity=joint_vel
        )

def blend_pose(pose_a: dict[int, float], pose_b: dict[int, float], alpha: float) -> dict[int, float]:
    blended = {}
    for joint in pose_a:
        blended[joint] = (1.0 - alpha) * pose_a[joint] + alpha * pose_b[joint]
    return blended

def transition_pose(
    pose_from: dict[int, float],
    pose_to: dict[int, float],
    duration: float,
    force: float = 280,
    max_vel: float = 1.2
) -> None:
    steps = max(1, int(duration * 240))
    for i in range(steps):
        alpha = (i + 1) / steps
        current_pose = blend_pose(pose_from, pose_to, alpha)
        apply_pose(current_pose, force=force, max_vel=max_vel)
        p.stepSimulation()
        time.sleep(1 / 240)

def hold_pose(
    pose: dict[int, float],
    duration: float,
    force: float = 280,
    max_vel: float = 1.2
) -> None:
    steps = max(1, int(duration * 240))
    for _ in range(steps):
        apply_pose(pose, force=force, max_vel=max_vel)
        p.stepSimulation()
        time.sleep(1 / 240)

# -----------------------------
# State machine
# -----------------------------
states = [
    ("stand", POSE_STAND, 1.5),
    ("right_thigh_lift", POSE_RIGHT_THIGH_LIFT, 0.8),
    ("right_knee_bend", POSE_RIGHT_KNEE_BEND, 0.8),
    ("stand", POSE_STAND, 0.8),
    ("left_thigh_lift", POSE_LEFT_THIGH_LIFT, 0.8),
    ("left_knee_bend", POSE_LEFT_KNEE_BEND, 0.8),
    ("stand", POSE_STAND, 0.8),
]

# Start pose
reset_to_pose(POSE_STAND)
hold_pose(POSE_STAND, 2.0)

# Run several cycles
current_pose = POSE_STAND
num_cycles = 4

for cycle in range(num_cycles):
    print(f"\nCycle {cycle + 1}")
    for state_name, target_pose, hold_time in states:
        print(f"State: {state_name}")
        transition_pose(current_pose, target_pose, duration=0.9, force=280, max_vel=1.2)
        hold_pose(target_pose, duration=hold_time, force=280, max_vel=1.2)
        current_pose = target_pose

# Finish in stand pose
transition_pose(current_pose, POSE_STAND, duration=1.0, force=280, max_vel=1.2)
hold_pose(POSE_STAND, duration=2.0, force=280, max_vel=1.2)