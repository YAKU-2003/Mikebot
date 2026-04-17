import pybullet as p
import pybullet_data
import time

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
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-20,
    cameraTargetPosition=[0, 0, 0.22]
)

# Joint mapping
right_hip = 1
right_thigh = 3
right_knee = 5
right_foot = 7

left_hip = 9
left_thigh = 11
left_knee = 13
left_foot = 15

# Initial pose
pose_stand = {
    right_hip: 0.0,
    right_thigh: 0.05,
    right_knee: 0.0,
    right_foot: 0.0,
    left_hip: 0.0,
    left_thigh: 0.05,
    left_knee: 0.0,
    left_foot: 0.0
}

# Softer lifted poses
pose_right_lift = {
    right_hip: 0.0,
    right_thigh: 0.15,
    right_knee: -0.20,
    right_foot: 0.0,
    left_hip: 0.0,
    left_thigh: 0.05,
    left_knee: 0.0,
    left_foot: 0.0
}

pose_left_lift = {
    right_hip: 0.0,
    right_thigh: 0.05,
    right_knee: 0.0,
    right_foot: 0.0,
    left_hip: 0.0,
    left_thigh: 0.15,
    left_knee: -0.20,
    left_foot: 0.0
}

# Set initial pose instantly
for joint, angle in pose_stand.items():
    p.resetJointState(robot, joint, angle)

def apply_pose(pose, force=250, max_vel=1.2):
    for joint, angle in pose.items():
        p.setJointMotorControl2(
            robot,
            joint,
            p.POSITION_CONTROL,
            targetPosition=angle,
            force=force,
            maxVelocity=max_vel
        )

def blend_pose(pose_a, pose_b, alpha):
    blended = {}
    for joint in pose_a:
        blended[joint] = (1 - alpha) * pose_a[joint] + alpha * pose_b[joint]
    return blended

def transition(pose_from, pose_to, duration, steps_per_sec=240):
    steps = int(duration * steps_per_sec)
    for i in range(steps):
        alpha = (i + 1) / steps
        pose_now = blend_pose(pose_from, pose_to, alpha)
        apply_pose(pose_now, force=250, max_vel=1.2)
        p.stepSimulation()
        time.sleep(1 / 240)

def hold(pose, duration, steps_per_sec=240):
    steps = int(duration * steps_per_sec)
    for _ in range(steps):
        apply_pose(pose, force=250, max_vel=1.2)
        p.stepSimulation()
        time.sleep(1 / 240)

# Let robot settle
hold(pose_stand, 2.0)

# Gentle stepping sequence
for _ in range(4):
    transition(pose_stand, pose_right_lift, 1.0)
    hold(pose_right_lift, 0.6)
    transition(pose_right_lift, pose_stand, 1.0)
    hold(pose_stand, 0.5)

    transition(pose_stand, pose_left_lift, 1.0)
    hold(pose_left_lift, 0.6)
    transition(pose_left_lift, pose_stand, 1.0)
    hold(pose_stand, 0.5)

hold(pose_stand, 2.0)