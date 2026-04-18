import pybullet as p
import pybullet_data
import time
import math

# -----------------------------
# Setup
# -----------------------------
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

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
# Standing pose
# mirrored signs: right = +, left = -
# -----------------------------
stand_hip = 0.0
stand_thigh = 0.10
stand_knee = 0.12
stand_foot = 0.0

# reset initial pose
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
# balance assist
# -----------------------------
Kp_thigh = 0.35
Kd_thigh = 0.05

Kp_knee = 0.18
Kd_knee = 0.03

Kp_foot = 0.06
Kd_foot = 0.01

# command limits
max_thigh_cmd = 0.28
max_knee_cmd = 0.28
max_foot_cmd = 0.10
max_hip_cmd = 0.08

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

# -----------------------------
# Settle first
# -----------------------------
for _ in range(480):
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL, stand_hip, force=500, maxVelocity=3)
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL, stand_thigh, force=900, maxVelocity=4)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL, stand_knee, force=900, maxVelocity=4)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL, stand_foot, force=700, maxVelocity=3)

    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL, -stand_hip, force=500, maxVelocity=3)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL, -stand_thigh, force=900, maxVelocity=4)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL, -stand_knee, force=900, maxVelocity=4)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL, -stand_foot, force=700, maxVelocity=3)

    p.stepSimulation()
    time.sleep(1/240)

prev_pitch = 0.0

# -----------------------------
# Main walking + balance loop
# -----------------------------
while True:
    dt = 1 / 240.0
    t = time.time()

    # --- virtual IMU ---
    base_pos, base_orn = p.getBasePositionAndOrientation(robot)
    roll, pitch, yaw = p.getEulerFromQuaternion(base_orn)
    pitch_rate = (pitch - prev_pitch) / dt
    prev_pitch = pitch

    # -----------------------------
    # Small walking pattern
    # -----------------------------
    gait_freq = 2.2
    s = math.sin(gait_freq * t)

    # very small alternating gait
    hip_gait = 0.02 * s
    thigh_gait = 0.06 * s

    # bend knee only during swing phase
    right_knee_gait = 0.06 * max(0.0, s)
    left_knee_gait  = 0.06 * max(0.0, -s)

    # slight foot assist
    right_foot_gait = 0.02 * max(0.0, s)
    left_foot_gait  = 0.02 * max(0.0, -s)

    # -----------------------------
    # Balance assist
    # -----------------------------
    thigh_corr = -(Kp_thigh * pitch + Kd_thigh * pitch_rate)
    knee_corr  = -(Kp_knee * pitch + Kd_knee * pitch_rate)
    foot_corr  = -(Kp_foot * pitch + Kd_foot * pitch_rate)

    # -----------------------------
    # Right side commands
    # -----------------------------
    right_hip_cmd = clamp(stand_hip + hip_gait, -max_hip_cmd, max_hip_cmd)
    right_thigh_cmd = clamp(stand_thigh + thigh_gait + thigh_corr, -max_thigh_cmd, max_thigh_cmd)
    right_knee_cmd  = clamp(stand_knee + right_knee_gait + knee_corr, -max_knee_cmd, max_knee_cmd)
    right_foot_cmd  = clamp(stand_foot + right_foot_gait + foot_corr, -max_foot_cmd, max_foot_cmd)

    # -----------------------------
    # Left side mirrored
    # -----------------------------
    left_hip_cmd = -right_hip_cmd
    left_thigh_cmd = clamp(-(stand_thigh - thigh_gait + thigh_corr), -max_thigh_cmd, max_thigh_cmd)
    left_knee_cmd  = clamp(-(stand_knee + left_knee_gait + knee_corr), -max_knee_cmd, max_knee_cmd)
    left_foot_cmd  = clamp(-(stand_foot + left_foot_gait + foot_corr), -max_foot_cmd, max_foot_cmd)

    # -----------------------------
    # Apply controls
    # -----------------------------
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL,
                            targetPosition=right_hip_cmd, force=500, maxVelocity=3)
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=right_thigh_cmd, force=1100, maxVelocity=5)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=right_knee_cmd, force=1100, maxVelocity=5)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL,
                            targetPosition=right_foot_cmd, force=850, maxVelocity=4)

    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL,
                            targetPosition=left_hip_cmd, force=500, maxVelocity=3)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=left_thigh_cmd, force=1100, maxVelocity=5)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=left_knee_cmd, force=1100, maxVelocity=5)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL,
                            targetPosition=left_foot_cmd, force=850, maxVelocity=4)

    p.stepSimulation()
    time.sleep(dt)