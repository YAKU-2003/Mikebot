import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")

robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.8],
    useFixedBase=True
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.4,
    cameraYaw=45,
    cameraPitch=-20,
    cameraTargetPosition=[0, 0, 0.35]
)

# Joint map
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
    # sinusoidal gait
    hip_amp = 0.20
    thigh_amp = 0.45
    knee_amp = 0.90
    foot_amp = 0.45

    s = math.sin(t)

    # hips opposite phase
    right_hip_target = hip_amp * s
    left_hip_target = -hip_amp * s

    # thighs follow hips lightly
    right_thigh_target = thigh_amp * s
    left_thigh_target = thigh_amp * s

    # knees bend during swing
    right_knee_target = knee_amp * max(0.0, s)
    left_knee_target = knee_amp * max(0.0, -s)

    # feet compensate a little
    right_foot_target = -foot_amp * max(0.0, s)
    left_foot_target = foot_amp * max(0.0, -s)

    p.setJointMotorControl2(robot, right_hip, p.POSITION_CONTROL, right_hip_target, force=200, maxVelocity=12)
    p.setJointMotorControl2(robot, left_hip, p.POSITION_CONTROL, left_hip_target, force=200, maxVelocity=12)

    p.setJointMotorControl2(robot, right_thigh, p.POSITION_CONTROL, right_thigh_target, force=200, maxVelocity=12)
    p.setJointMotorControl2(robot, left_thigh, p.POSITION_CONTROL, left_thigh_target, force=200, maxVelocity=12)

    p.setJointMotorControl2(robot, right_knee, p.POSITION_CONTROL, right_knee_target, force=200, maxVelocity=12)
    p.setJointMotorControl2(robot, left_knee, p.POSITION_CONTROL, left_knee_target, force=200, maxVelocity=12)

    p.setJointMotorControl2(robot, right_foot, p.POSITION_CONTROL, right_foot_target, force=150, maxVelocity=12)
    p.setJointMotorControl2(robot, left_foot, p.POSITION_CONTROL, left_foot_target, force=150, maxVelocity=12)

    p.stepSimulation()
    time.sleep(1/240)
    t += 0.06
