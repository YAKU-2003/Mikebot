import pybullet as p
import pybullet_data
import time
import math

# -----------------------------
# IK solver for 2-link leg
# -----------------------------
def leg_ik(x, z, L1, L2):
    D = (x*x + z*z - L1*L1 - L2*L2) / (2 * L1 * L2)
    D = max(-1.0, min(1.0, D))

    # knee-bent solution
    theta2 = math.atan2(-math.sqrt(1 - D*D), D)
    theta1 = math.atan2(z, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))

    return theta1, theta2

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
    useFixedBase=True
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
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
# Link lengths from CAD
# -----------------------------
L1 = 0.08
L2 = 0.12

# Desired foot point relative to thigh joint
x = 0.03
z = -0.16

thigh_angle, knee_angle = leg_ik(x, z, L1, L2)

print("Desired foot point:")
print(f"x = {x:.3f} m, z = {z:.3f} m")
print("IK output:")
print(f"thigh = {thigh_angle:.4f} rad")
print(f"knee  = {knee_angle:.4f} rad")

# -----------------------------
# IMPORTANT:
# You may need to flip signs depending on your URDF
# Start with these, then adjust if motion looks wrong
# -----------------------------
right_thigh_cmd = thigh_angle
right_knee_cmd = knee_angle

# keep everything else fixed
while True:
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL,
                            targetPosition=0.0, force=300, maxVelocity=1.0)
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=right_thigh_cmd, force=300, maxVelocity=1.0)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=right_knee_cmd, force=300, maxVelocity=1.0)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL,
                            targetPosition=0.0, force=500, maxVelocity=1.0)

    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL,
                            targetPosition=0.0, force=300, maxVelocity=1.0)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=0.0, force=300, maxVelocity=1.0)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=0.0, force=300, maxVelocity=1.0)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL,
                            targetPosition=0.0, force=500, maxVelocity=1.0)

    p.stepSimulation()
    time.sleep(1/240)