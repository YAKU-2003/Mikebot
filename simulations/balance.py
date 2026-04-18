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
RIGHT_HIP = 1
RIGHT_THIGH = 3
RIGHT_KNEE = 5
RIGHT_FOOT = 7

LEFT_HIP = 9
LEFT_THIGH = 11
LEFT_KNEE = 13
LEFT_FOOT = 15

# -----------------------------
# Initial standing pose
# mirrored signs: right = +, left = -
# -----------------------------
stand_hip = 0.0
stand_thigh = 0.05
stand_knee = 0.0
stand_foot = 0.0

p.resetJointState(robot, RIGHT_HIP, stand_hip)
p.resetJointState(robot, RIGHT_THIGH, stand_thigh)
p.resetJointState(robot, RIGHT_KNEE, stand_knee)
p.resetJointState(robot, RIGHT_FOOT, stand_foot)

p.resetJointState(robot, LEFT_HIP, -stand_hip)
p.resetJointState(robot, LEFT_THIGH, -stand_thigh)
p.resetJointState(robot, LEFT_KNEE, -stand_knee)
p.resetJointState(robot, LEFT_FOOT, -stand_foot)

# -----------------------------
# Controller gains
# Tune these slowly if needed
# -----------------------------
Kp_thigh = 0.8
Kd_thigh = 0.15

Kp_knee = 0.4
Kd_knee = 0.08

# Joint limits for safety in controller
max_thigh_cmd = 0.35
max_knee_cmd = 0.35

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

# -----------------------------
# Let robot settle first
# -----------------------------
for _ in range(480):
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL,
                            targetPosition=stand_hip, force=1500, maxVelocity=8)
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=stand_thigh, force=1800, maxVelocity=8)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=stand_knee, force=1800, maxVelocity=8)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL,
                            targetPosition=stand_foot, force=2000, maxVelocity=6)

    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL,
                            targetPosition=-stand_hip, force=1500, maxVelocity=8)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=-stand_thigh, force=1800, maxVelocity=8)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=-stand_knee, force=1800, maxVelocity=8)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL,
                            targetPosition=-stand_foot, force=2000, maxVelocity=6)

    p.stepSimulation()
    time.sleep(1/240)

# -----------------------------
# Main balance loop
# -----------------------------
prev_pitch = 0.0

while True:
    dt = 1.0 / 240.0

    # Virtual IMU from simulation
    base_pos, base_orn = p.getBasePositionAndOrientation(robot)
    roll, pitch, yaw = p.getEulerFromQuaternion(base_orn)

    # Approximate pitch rate
    pitch_rate = (pitch - prev_pitch) / dt
    prev_pitch = pitch

    # ---------------------------------
    # Feedback controller
    # If robot tips forward/backward,
    # thighs and knees react to counter it
    # ---------------------------------
    thigh_correction = -(Kp_thigh * pitch + Kd_thigh * pitch_rate)
    knee_correction = -(Kp_knee * pitch + Kd_knee * pitch_rate)

    # Add corrections around standing pose
    right_thigh_cmd = clamp(stand_thigh + thigh_correction, -max_thigh_cmd, max_thigh_cmd)
    right_knee_cmd  = clamp(stand_knee + knee_correction,  -max_knee_cmd,  max_knee_cmd)

    # mirrored left side
    left_thigh_cmd = -right_thigh_cmd
    left_knee_cmd  = -right_knee_cmd

    # Keep hips and feet locked for now
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL,
                            targetPosition=stand_hip, force=1500, maxVelocity=8)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL,
                            targetPosition=stand_foot, force=2200, maxVelocity=6)

    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL,
                            targetPosition=-stand_hip, force=1500, maxVelocity=8)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL,
                            targetPosition=-stand_foot, force=2200, maxVelocity=6)

    # Legs move to balance the robot
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=right_thigh_cmd, force=2200, maxVelocity=10)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=right_knee_cmd, force=2200, maxVelocity=10)

    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=left_thigh_cmd, force=2200, maxVelocity=10)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=left_knee_cmd, force=2200, maxVelocity=10)

    p.stepSimulation()
    time.sleep(dt)