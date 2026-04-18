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
# Stable pose (your tuned one)
# -----------------------------
POSE_STAND = {
    "hip": 0.0,
    "thigh": -0.101,
    "knee": 0.197,
    "foot": 0.0
}

# IMPORTANT: use a SAFE crouch (not full -0.8 / 0.7)
POSE_CROUCH = {
    "hip": 0.0,
    "thigh": -0.40,
    "knee": 0.35,
    "foot": 0.0
}

# -----------------------------
# Helpers
# -----------------------------
def apply_pose(pose, force_main=1000, force_foot=800, vel_main=3.0, vel_foot=2.0):
    # Right
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL, pose["hip"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL, pose["thigh"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL, pose["knee"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL, pose["foot"], force=force_foot, maxVelocity=vel_foot)

    # Left mirrored
    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL, -pose["hip"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL, -pose["thigh"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL, -pose["knee"], force=force_main, maxVelocity=vel_main)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL, -pose["foot"], force=force_foot, maxVelocity=vel_foot)

def blend_pose(a, b, alpha):
    return {
        "hip":   (1-alpha)*a["hip"]   + alpha*b["hip"],
        "thigh": (1-alpha)*a["thigh"] + alpha*b["thigh"],
        "knee":  (1-alpha)*a["knee"]  + alpha*b["knee"],
        "foot":  (1-alpha)*a["foot"]  + alpha*b["foot"],
    }

def go_to_pose(start, end, duration=0.22):
    steps = int(duration * 240)
    for i in range(steps):
        alpha = (i+1)/steps
        pose = blend_pose(start, end, alpha)
        apply_pose(pose)
        p.stepSimulation()
        time.sleep(1/240)

def hold(pose, steps=40):
    for _ in range(steps):
        apply_pose(pose)
        p.stepSimulation()
        time.sleep(1/240)

# -----------------------------
# Build 5% cycle
# -----------------------------
up = [i/100.0 for i in range(0, 101, 5)]
down = [i/100.0 for i in range(95, -1, -5)]
cycle = up + down

# -----------------------------
# Run
# -----------------------------
current = POSE_STAND
hold(POSE_STAND, 120)

while True:
    for f in cycle:
        target = blend_pose(POSE_STAND, POSE_CROUCH, f)

        go_to_pose(current, target, duration=0.22)
        hold(target, 35)

        current = target