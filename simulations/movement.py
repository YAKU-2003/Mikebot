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
    basePosition=[0, 0, 0.40],
    useFixedBase=False
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=50,
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

# Initial standing pose
right_hip_init = 0.0
right_thigh_init = 0.05
right_knee_init = -0.15
right_foot_init = 0.0

left_hip_init = 0.0
left_thigh_init = 0.05
left_knee_init = -0.15
left_foot_init = 0.0

p.resetJointState(robot, right_hip, right_hip_init)
p.resetJointState(robot, right_thigh, right_thigh_init)
p.resetJointState(robot, right_knee, right_knee_init)
p.resetJointState(robot, right_foot, right_foot_init)

p.resetJointState(robot, left_hip, left_hip_init)
p.resetJointState(robot, left_thigh, left_thigh_init)
p.resetJointState(robot, left_knee, left_knee_init)
p.resetJointState(robot, left_foot, left_foot_init)

# Let robot settle
for _ in range(1000):
    p.setJointMotorControl2(robot, right_hip, p.POSITION_CONTROL, right_hip_init, force=250)
    p.setJointMotorControl2(robot, right_thigh, p.POSITION_CONTROL, right_thigh_init, force=250)
    p.setJointMotorControl2(robot, right_knee, p.POSITION_CONTROL, right_knee_init, force=250)
    p.setJointMotorControl2(robot, right_foot, p.POSITION_CONTROL, right_foot_init, force=400)

    p.setJointMotorControl2(robot, left_hip, p.POSITION_CONTROL, left_hip_init, force=250)
    p.setJointMotorControl2(robot, left_thigh, p.POSITION_CONTROL, left_thigh_init, force=250)
    p.setJointMotorControl2(robot, left_knee, p.POSITION_CONTROL, left_knee_init, force=250)
    p.setJointMotorControl2(robot, left_foot, p.POSITION_CONTROL, left_foot_init, force=400)

    p.stepSimulation()
    time.sleep(1/240)

start_pos = p.getBasePositionAndOrientation(robot)[0]

t = 0.0
dt = 1/240
sim_time = 8.0

while t < sim_time:
    s = math.sin(2.0 * t)

    # keep hip yaw very small for now
    hip_yaw_amp = 0.03

    # main locomotion drivers
    thigh_amp = 0.35
    knee_amp = 0.55

    # alternating thigh swing
    right_thigh_target = 0.05 + thigh_amp * s
    left_thigh_target  = 0.05 - thigh_amp * s

    # bend knee only during forward swing
    right_knee_target = -0.15 - knee_amp * max(0.0, s)
    left_knee_target  = -0.15 - knee_amp * max(0.0, -s)

    # tiny hip yaw just to help alternating motion
    right_hip_target = hip_yaw_amp * s
    left_hip_target  = -hip_yaw_amp * s

    # lock feet
    right_foot_target = 0.0
    left_foot_target = 0.0

    p.setJointMotorControl2(robot, right_hip, p.POSITION_CONTROL, right_hip_target, force=180, maxVelocity=8)
    p.setJointMotorControl2(robot, left_hip, p.POSITION_CONTROL, left_hip_target, force=180, maxVelocity=8)

    p.setJointMotorControl2(robot, right_thigh, p.POSITION_CONTROL, right_thigh_target, force=260, maxVelocity=10)
    p.setJointMotorControl2(robot, left_thigh, p.POSITION_CONTROL, left_thigh_target, force=260, maxVelocity=10)

    p.setJointMotorControl2(robot, right_knee, p.POSITION_CONTROL, right_knee_target, force=260, maxVelocity=10)
    p.setJointMotorControl2(robot, left_knee, p.POSITION_CONTROL, left_knee_target, force=260, maxVelocity=10)

    p.setJointMotorControl2(robot, right_foot, p.POSITION_CONTROL, right_foot_target, force=500, maxVelocity=5)
    p.setJointMotorControl2(robot, left_foot, p.POSITION_CONTROL, left_foot_target, force=500, maxVelocity=5)

    p.stepSimulation()
    time.sleep(dt)
    t += dt

end_pos = p.getBasePositionAndOrientation(robot)[0]

dx = end_pos[0] - start_pos[0]
dy = end_pos[1] - start_pos[1]
distance = (dx**2 + dy**2) ** 0.5
speed = distance / sim_time

print("Start Position:", start_pos)
print("End Position:", end_pos)
print(f"Displacement in x: {dx:.4f} m")
print(f"Displacement in y: {dy:.4f} m")
print(f"Planar distance traveled: {distance:.4f} m")
print(f"Average speed: {speed:.4f} m/s")