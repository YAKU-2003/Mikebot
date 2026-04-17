import pybullet as p
import pybullet_data
import time
import math

# Connect
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane + robot
p.loadURDF("plane.urdf")

robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.8],
    useFixedBase=True
)

# Camera (optional but helpful)
p.resetDebugVisualizerCamera(
    cameraDistance=1.4,
    cameraYaw=45,
    cameraPitch=-20,
    cameraTargetPosition=[0, 0, 0.35]
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

t = 0.0

while True:
    s = math.sin(t)

    # Slightly reduced hip motion (so others are visible)
    hip_amp = 0.20

    # Separate amplitudes to balance left/right
    thigh_amp_right = 0.45
    thigh_amp_left  = 0.30

    knee_amp_right = 0.95
    knee_amp_left  = 0.75

    foot_amp_right = 0.45
    foot_amp_left  = 0.35

    # Targets
    right_hip_target = hip_amp * s
    left_hip_target  = -hip_amp * s

    right_thigh_target = thigh_amp_right * s
    left_thigh_target  = -thigh_amp_left * s

    right_knee_target = knee_amp_right * max(0.0, s)
    left_knee_target  = knee_amp_left * max(0.0, -s)

    right_foot_target = -foot_amp_right * max(0.0, s)
    left_foot_target  = -foot_amp_left * max(0.0, -s)

    # Apply control
    p.setJointMotorControl2(robot, right_hip, p.POSITION_CONTROL, right_hip_target, force=200, maxVelocity=12)
    p.setJointMotorControl2(robot, left_hip,  p.POSITION_CONTROL, left_hip_target,  force=200, maxVelocity=12)

    p.setJointMotorControl2(robot, right_thigh, p.POSITION_CONTROL, right_thigh_target, force=220, maxVelocity=14)
    p.setJointMotorControl2(robot, left_thigh,  p.POSITION_CONTROL, left_thigh_target,  force=220, maxVelocity=14)

    p.setJointMotorControl2(robot, right_knee, p.POSITION_CONTROL, right_knee_target, force=250, maxVelocity=16)
    p.setJointMotorControl2(robot, left_knee,  p.POSITION_CONTROL, left_knee_target,  force=250, maxVelocity=16)

    p.setJointMotorControl2(robot, right_foot, p.POSITION_CONTROL, right_foot_target, force=220, maxVelocity=16)
    p.setJointMotorControl2(robot, left_foot,  p.POSITION_CONTROL, left_foot_target,  force=220, maxVelocity=16)

    # Step simulation
    p.stepSimulation()
    time.sleep(1/240)

    # Speed of motion
    t += 0.06