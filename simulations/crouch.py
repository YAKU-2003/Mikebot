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
    basePosition=[0, 0, 0.45],
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

# Feet remain fixed at 0.0
POSE_CROUCH = {
    RIGHT_HIP: 0.0,
    RIGHT_THIGH: 0.22,
    RIGHT_KNEE: -0.30,
    RIGHT_FOOT: 0.0,

    LEFT_HIP: 0.0,
    LEFT_THIGH: -0.22,
    LEFT_KNEE: 0.30,
    LEFT_FOOT: 0.0
}

# -----------------------------
# Helpers
# -----------------------------
def reset_to_pose(pose):
    for joint, angle in pose.items():
        p.resetJointState(robot, joint, angle)

def apply_pose(pose, force=320, max_vel=0.8):
    for joint, angle in pose.items():
        if joint in (RIGHT_FOOT, LEFT_FOOT):
            # feet are only held fixed, not moved
            joint_force = 500
            joint_vel = 0.5
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
        blended[joint] = (1 - alpha) * pose_a[joint] + alpha * pose_b[joint]
    return blended

def transition_pose(pose_from, pose_to, duration, force=320, max_vel=0.8):
    steps = max(1, int(duration * 240))
    for i in range(steps):
        alpha = (i + 1) / steps
        current_pose = blend_pose(pose_from, pose_to, alpha)
        apply_pose(current_pose, force=force, max_vel=max_vel)
        p.stepSimulation()
        time.sleep(1 / 240)

def hold_pose(pose, duration, force=320, max_vel=0.8):
    steps = max(1, int(duration * 240))
    for _ in range(steps):
        apply_pose(pose, force=force, max_vel=max_vel)
        p.stepSimulation()
        time.sleep(1 / 240)

# -----------------------------
# Run motion
# -----------------------------
reset_to_pose(POSE_STAND)
hold_pose(POSE_STAND, 2.0)

for _ in range(3):
    print("Standing to crouch")
    transition_pose(POSE_STAND, POSE_CROUCH, duration=1.2, force=320, max_vel=0.8)
    hold_pose(POSE_CROUCH, 1.0, force=320, max_vel=0.8)

    print("Crouch to standing")
    transition_pose(POSE_CROUCH, POSE_STAND, duration=1.2, force=320, max_vel=0.8)
    hold_pose(POSE_STAND, 1.0, force=320, max_vel=0.8)

hold_pose(POSE_STAND, 2.0)
