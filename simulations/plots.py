import pybullet as p
import pybullet_data
import time
import math
import matplotlib.pyplot as plt

# Connect to PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load environment
p.loadURDF("plane.urdf")

# Load robot
robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.8],
    useFixedBase=True
)

# Camera view
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

# Data storage
time_data = []

right_hip_angle = []
right_hip_velocity = []
right_hip_torque = []

right_knee_angle = []
right_knee_velocity = []
right_knee_torque = []

left_hip_angle = []
left_hip_velocity = []
left_hip_torque = []

left_knee_angle = []
left_knee_velocity = []
left_knee_torque = []

# Simulation time variable
t = 0.0

while t < 10.0:
    s = math.sin(t)

    # Gait amplitudes
    hip_amp = 0.20
    thigh_amp_right = 0.45
    thigh_amp_left = 0.30
    knee_amp_right = 0.95
    knee_amp_left = 0.75
    foot_amp_right = 0.45
    foot_amp_left = 0.35

    # Joint targets
    right_hip_target = hip_amp * s
    left_hip_target = -hip_amp * s

    right_thigh_target = thigh_amp_right * s
    left_thigh_target = -thigh_amp_left * s

    right_knee_target = knee_amp_right * max(0.0, s)
    left_knee_target = knee_amp_left * max(0.0, -s)

    right_foot_target = -foot_amp_right * max(0.0, s)
    left_foot_target = -foot_amp_left * max(0.0, -s)

    # Apply motor control
    p.setJointMotorControl2(robot, right_hip, p.POSITION_CONTROL, right_hip_target, force=200, maxVelocity=12)
    p.setJointMotorControl2(robot, left_hip, p.POSITION_CONTROL, left_hip_target, force=200, maxVelocity=12)

    p.setJointMotorControl2(robot, right_thigh, p.POSITION_CONTROL, right_thigh_target, force=220, maxVelocity=14)
    p.setJointMotorControl2(robot, left_thigh, p.POSITION_CONTROL, left_thigh_target, force=220, maxVelocity=14)

    p.setJointMotorControl2(robot, right_knee, p.POSITION_CONTROL, right_knee_target, force=250, maxVelocity=16)
    p.setJointMotorControl2(robot, left_knee, p.POSITION_CONTROL, left_knee_target, force=250, maxVelocity=16)

    p.setJointMotorControl2(robot, right_foot, p.POSITION_CONTROL, right_foot_target, force=220, maxVelocity=16)
    p.setJointMotorControl2(robot, left_foot, p.POSITION_CONTROL, left_foot_target, force=220, maxVelocity=16)

    # Step simulation
    p.stepSimulation()
    time.sleep(1 / 240)

    # Log time
    time_data.append(t)

    # Right hip
    state = p.getJointState(robot, right_hip)
    right_hip_angle.append(state[0])
    right_hip_velocity.append(state[1])
    right_hip_torque.append(state[3])

    # Right knee
    state = p.getJointState(robot, right_knee)
    right_knee_angle.append(state[0])
    right_knee_velocity.append(state[1])
    right_knee_torque.append(state[3])

    # Left hip
    state = p.getJointState(robot, left_hip)
    left_hip_angle.append(state[0])
    left_hip_velocity.append(state[1])
    left_hip_torque.append(state[3])

    # Left knee
    state = p.getJointState(robot, left_knee)
    left_knee_angle.append(state[0])
    left_knee_velocity.append(state[1])
    left_knee_torque.append(state[3])

    # Increase time
    t += 0.06

# Plot joint angles
plt.figure()
plt.plot(time_data, right_hip_angle, label="Right Hip")
plt.plot(time_data, right_knee_angle, label="Right Knee")
plt.plot(time_data, left_hip_angle, label="Left Hip")
plt.plot(time_data, left_knee_angle, label="Left Knee")
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("Joint Angle vs Time")
plt.legend()
plt.grid(True)
plt.show()

# Plot joint velocities
plt.figure()
plt.plot(time_data, right_hip_velocity, label="Right Hip")
plt.plot(time_data, right_knee_velocity, label="Right Knee")
plt.plot(time_data, left_hip_velocity, label="Left Hip")
plt.plot(time_data, left_knee_velocity, label="Left Knee")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (rad/s)")
plt.title("Joint Velocity vs Time")
plt.legend()
plt.grid(True)
plt.show()

# Plot joint torques
plt.figure()
plt.plot(time_data, right_hip_torque, label="Right Hip")
plt.plot(time_data, right_knee_torque, label="Right Knee")
plt.plot(time_data, left_hip_torque, label="Left Hip")
plt.plot(time_data, left_knee_torque, label="Left Knee")
plt.xlabel("Time (s)")
plt.ylabel("Torque")
plt.title("Joint Torque vs Time")
plt.legend()
plt.grid(True)
plt.show()