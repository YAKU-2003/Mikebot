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
RIGHT_HIP = 1      # yaw
RIGHT_THIGH = 3    # pitch
RIGHT_KNEE = 5     # pitch
RIGHT_FOOT = 7     # roll

LEFT_HIP = 9
LEFT_THIGH = 11
LEFT_KNEE = 13
LEFT_FOOT = 15

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

# Shift weight to LEFT support leg so RIGHT leg can lift
POSE_SHIFT_LEFT = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.05,
    RIGHT_KNEE: 0.0,
    RIGHT_FOOT: 0.0,
    LEFT_HIP: 0.0,
    LEFT_THIGH: 0.05,
    LEFT_KNEE: 0.0,
    LEFT_FOOT: 0.14   # support-foot roll
}

POSE_RIGHT_THIGH_LIFT = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.14,
    RIGHT_KNEE: 0.0,
    RIGHT_FOOT: 0.0,
    LEFT_HIP: 0.0,
    LEFT_THIGH: 0.05,
    LEFT_KNEE: 0.0,
    LEFT_FOOT: 0.14
}

POSE_RIGHT_KNEE_BEND = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.14,
    RIGHT_KNEE: -0.12,
    RIGHT_FOOT: 0.0,
    LEFT_HIP: 0.0,
    LEFT_THIGH: 0.05,
    LEFT_KNEE: 0.0,
    LEFT_FOOT: 0.14
}

# Shift weight to RIGHT support leg so LEFT leg can lift
POSE_SHIFT_RIGHT = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.05,
    RIGHT_KNEE: 0.0,
    RIGHT_FOOT: -0.14,   # support-foot roll
    LEFT_HIP: 0.0,
    LEFT_THIGH: 0.05,
    LEFT_KNEE: 0.0,
    LEFT_FOOT: 0.0
}

POSE_LEFT_THIGH_LIFT = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.05,
    RIGHT_KNEE: 0.0,
    RIGHT_FOOT: -0.14,
    LEFT_HIP: 0.0,
    LEFT_THIGH: 0.14,
    LEFT_KNEE: 0.0,
    LEFT_FOOT: 0.0
}

POSE_LEFT_KNEE_BEND = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.05,
    RIGHT_KNEE: 0.0,
    RIGHT_FOOT: -0.14,
    LEFT_HIP: 0.0,
    LEFT_THIGH: 0.14,
    LEFT_KNEE: -0.12,
    LEFT_FOOT: 0.0
}

# -----------------------------
# Helpers
# -----------------------------
def reset_to_pose(pose):
    for joint, angle in pose.items():
        p.resetJointState(robot, joint, angle)

def apply_pose(pose, force=260, max_vel=0.8):
    for joint, angle in pose.items():
        if joint in (RIGHT_FOOT, LEFT_FOOT):
            joint_force = 450
            joint_vel = 0.6
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

def blend_pose(pose_a, pose_b, alpha):
    blended = {}
    for joint in pose_a:
        blended[joint] = (1.0 - alpha) * pose_a[joint] + alpha * pose_b[joint]
    return blended

def transition_pose(pose_from, pose_to, duration, force=260, max_vel=0.8):
    steps = max(1, int(duration * 240))
    for i in range(steps):
        alpha = (i + 1) / steps
        current_pose = blend_pose(pose_from, pose_to, alpha)
        apply_pose(current_pose, force=force, max_vel=max_vel)
        p.stepSimulation()
        time.sleep(1 / 240)

def hold_pose(pose, duration, force=260, max_vel=0.8):
    steps = max(1, int(duration * 240))
    for _ in range(steps):
        apply_pose(pose, force=force, max_vel=max_vel)
        p.stepSimulation()
        time.sleep(1 / 240)

# -----------------------------
# State machine
# -----------------------------
states = [
    ("stand", POSE_STAND, 1.2),

    ("shift_left", POSE_SHIFT_LEFT, 0.8),
    ("right_thigh_lift", POSE_RIGHT_THIGH_LIFT, 0.8),
    ("right_knee_bend", POSE_RIGHT_KNEE_BEND, 0.8),
    ("stand", POSE_STAND, 1.0),

    ("shift_right", POSE_SHIFT_RIGHT, 0.8),
    ("left_thigh_lift", POSE_LEFT_THIGH_LIFT, 0.8),
    ("left_knee_bend", POSE_LEFT_KNEE_BEND, 0.8),
    ("stand", POSE_STAND, 1.0),
]

reset_to_pose(POSE_STAND)
hold_pose(POSE_STAND, 2.0)

current_pose = POSE_STAND
num_cycles = 3

for cycle in range(num_cycles):
    print(f"\nCycle {cycle + 1}")
    for state_name, target_pose, hold_time in states:
        print(f"State: {state_name}")
        transition_pose(current_pose, target_pose, duration=1.0, force=260, max_vel=0.8)
        hold_pose(target_pose, duration=hold_time, force=260, max_vel=0.8)
        current_pose = target_pose

<<<<<<< Updated upstream
transition_pose(current_pose, POSE_STAND, duration=1.0, force=260, max_vel=0.8)
hold_pose(POSE_STAND, 2.0)
=======
# Finish in stand pose
transition_pose(current_pose, POSE_STAND, duration=1.0, force=280, max_vel=1.2)
hold_pose(POSE_STAND, duration=2.0, force=280, max_vel=1.2)
>>>>>>> Stashed changes
