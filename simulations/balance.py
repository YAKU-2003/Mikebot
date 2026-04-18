import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create a gentle slope
slope_angle_deg = 5
slope_angle_rad = math.radians(slope_angle_deg)

collision_shape = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[1.5, 1.5, 0.02]
)

visual_shape = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[1.5, 1.5, 0.02],
    rgbaColor=[0.7, 0.7, 0.7, 1]
)

slope = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=collision_shape,
    baseVisualShapeIndex=visual_shape,
    basePosition=[0, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([0, slope_angle_rad, 0])
)

robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.36],
    useFixedBase=False
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.8,
    cameraYaw=50,
    cameraPitch=-20,
    cameraTargetPosition=[0, 0, 0.22]
)

# joint ids
RIGHT_HIP = 1
RIGHT_THIGH = 3
RIGHT_KNEE = 5
RIGHT_FOOT = 7

LEFT_HIP = 9
LEFT_THIGH = 11
LEFT_KNEE = 13
LEFT_FOOT = 15

# standing pose
stand_hip = 0.0
stand_thigh = 0.12
stand_knee = 0.18
stand_foot = 0.0

p.resetJointState(robot, RIGHT_HIP, stand_hip)
p.resetJointState(robot, RIGHT_THIGH, stand_thigh)
p.resetJointState(robot, RIGHT_KNEE, stand_knee)
p.resetJointState(robot, RIGHT_FOOT, stand_foot)

p.resetJointState(robot, LEFT_HIP, -stand_hip)
p.resetJointState(robot, LEFT_THIGH, -stand_thigh)
p.resetJointState(robot, LEFT_KNEE, -stand_knee)
p.resetJointState(robot, LEFT_FOOT, -stand_foot)

# simple balance assist gains
Kp_thigh = 0.35
Kd_thigh = 0.05
Kp_knee = 0.20
Kd_knee = 0.03
Kp_foot = 0.08
Kd_foot = 0.01

max_thigh_cmd = 0.25
max_knee_cmd = 0.25
max_foot_cmd = 0.10

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

prev_pitch = 0.0

while True:
    dt = 1/240

    base_pos, base_orn = p.getBasePositionAndOrientation(robot)
    roll, pitch, yaw = p.getEulerFromQuaternion(base_orn)
    pitch_rate = (pitch - prev_pitch) / dt
    prev_pitch = pitch

    thigh_corr = -(Kp_thigh * pitch + Kd_thigh * pitch_rate)
    knee_corr  = -(Kp_knee * pitch + Kd_knee * pitch_rate)
    foot_corr  = -(Kp_foot * pitch + Kd_foot * pitch_rate)

    right_thigh_cmd = clamp(stand_thigh + thigh_corr, -max_thigh_cmd, max_thigh_cmd)
    right_knee_cmd  = clamp(stand_knee + knee_corr,  -max_knee_cmd,  max_knee_cmd)
    right_foot_cmd  = clamp(stand_foot + foot_corr,  -max_foot_cmd,  max_foot_cmd)

    left_thigh_cmd = -right_thigh_cmd
    left_knee_cmd  = -right_knee_cmd
    left_foot_cmd  = -right_foot_cmd

    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL, stand_hip, force=500, maxVelocity=3)
    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL, -stand_hip, force=500, maxVelocity=3)

    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL, right_thigh_cmd, force=1000, maxVelocity=5)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL, right_knee_cmd, force=1000, maxVelocity=5)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL, right_foot_cmd, force=800, maxVelocity=4)

    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL, left_thigh_cmd, force=1000, maxVelocity=5)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL, left_knee_cmd, force=1000, maxVelocity=5)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL, left_foot_cmd, force=800, maxVelocity=4)

    p.stepSimulation()
    time.sleep(dt)