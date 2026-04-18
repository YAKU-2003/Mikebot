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
    basePosition=[0, 0, 0.42],
    useFixedBase=False
)

# reduce bounce
p.changeDynamics(robot, -1, restitution=0.0, lateralFriction=1.0)
for j in range(p.getNumJoints(robot)):
    p.changeDynamics(robot, j, restitution=0.0, lateralFriction=1.0)

p.resetDebugVisualizerCamera(
    cameraDistance=1.8,
    cameraYaw=50,
    cameraPitch=-18,
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
# Stable standing pose
# -----------------------------
POSE_STAND = {
    "hip": 0.0,
    "thigh": -0.101,
    "knee": 0.197,
    "foot": 0.0
}

# Final crouch pose from slider
POSE_CROUCH = {
    "hip": 0.0,
    "thigh": -0.800,
    "knee": 0.700,
    "foot": 0.0
}

# -----------------------------
# Helpers
# -----------------------------
def apply_pose(pose, force_main=1200, force_foot=850, vel_main=4.0, vel_foot=2.0):
    # Right
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL,
                            targetPosition=pose["hip"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=pose["thigh"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=pose["knee"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL,
                            targetPosition=pose["foot"], force=force_foot, maxVelocity=vel_foot)

    # Left mirrored
    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL,
                            targetPosition=-pose["hip"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=-pose["thigh"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=-pose["knee"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL,
                            targetPosition=-pose["foot"], force=force_foot, maxVelocity=vel_foot)

def hold_pose(pose, duration, force_main=1200, force_foot=850, vel_main=4.0, vel_foot=2.0):
    steps = max(1, int(duration * 240))
    for _ in range(steps):
        apply_pose(pose, force_main, force_foot, vel_main, vel_foot)
        p.stepSimulation()
        time.sleep(1/240)

def blend_pose(pose_a, pose_b, alpha):
    return {
        "hip":   (1 - alpha) * pose_a["hip"]   + alpha * pose_b["hip"],
        "thigh": (1 - alpha) * pose_a["thigh"] + alpha * pose_b["thigh"],
        "knee":  (1 - alpha) * pose_a["knee"]  + alpha * pose_b["knee"],
        "foot":  (1 - alpha) * pose_a["foot"]  + alpha * pose_b["foot"],
    }

def transition_pose(start_pose, end_pose, duration, force_main=1200, force_foot=850, vel_main=4.0, vel_foot=2.0):
    steps = max(1, int(duration * 240))
    for i in range(steps):
        alpha = (i + 1) / steps
        pose = blend_pose(start_pose, end_pose, alpha)
        apply_pose(pose, force_main, force_foot, vel_main, vel_foot)
        p.stepSimulation()
        time.sleep(1/240)

# -----------------------------
# Build fine progressive stages
# knees first, then thighs
# -----------------------------
def partial_pose(knee_frac, thigh_frac):
    return {
        "hip": 0.0,
        "thigh": POSE_STAND["thigh"] + thigh_frac * (POSE_CROUCH["thigh"] - POSE_STAND["thigh"]),
        "knee":  POSE_STAND["knee"]  + knee_frac  * (POSE_CROUCH["knee"]  - POSE_STAND["knee"]),
        "foot": 0.0
    }

fractions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

CROUCH_STAGES = []
for f in fractions:
    CROUCH_STAGES.append(partial_pose(knee_frac=f, thigh_frac=max(0.0, f - 0.1)))
    CROUCH_STAGES.append(partial_pose(knee_frac=f, thigh_frac=f))

# -----------------------------
# Run
# -----------------------------
hold_pose(POSE_STAND, 3.0)

while True:
    current = POSE_STAND

    print("Progressive crouch down")
    for stage in CROUCH_STAGES:
        transition_pose(current, stage, duration=0.32, force_main=1200, force_foot=850, vel_main=4.0, vel_foot=2.0)
        hold_pose(stage, duration=0.10, force_main=1200, force_foot=850, vel_main=4.0, vel_foot=2.0)
        current = stage

    hold_pose(POSE_CROUCH, 0.8, force_main=1200, force_foot=850, vel_main=4.0, vel_foot=2.0)

    print("Return to stand")
    transition_pose(POSE_CROUCH, POSE_STAND, duration=2.4, force_main=1100, force_foot=850, vel_main=3.5, vel_foot=2.0)

    # longer recovery to stop backward wobble
    hold_pose(POSE_STAND, 3.0, force_main=1100, force_foot=850, vel_main=3.5, vel_foot=2.0)