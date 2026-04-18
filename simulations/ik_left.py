import pybullet as p
import pybullet_data
import time
import math

def leg_ik(x, z, L1, L2):
    D = (x * x + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    D = max(-1.0, min(1.0, D))

    theta2 = math.atan2(-math.sqrt(1 - D * D), D)
    theta1 = math.atan2(z, x) - math.atan2(
        L2 * math.sin(theta2),
        L1 + L2 * math.cos(theta2)
    )
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
# Link lengths
# -----------------------------
L1 = 0.08
L2 = 0.12

# -----------------------------
# Initial pose
# -----------------------------
p.resetJointState(robot, RIGHT_HIP, 0.0)
p.resetJointState(robot, RIGHT_THIGH, 0.0)
p.resetJointState(robot, RIGHT_KNEE, 0.0)
p.resetJointState(robot, RIGHT_FOOT, 0.0)

p.resetJointState(robot, LEFT_HIP, 0.0)
p.resetJointState(robot, LEFT_THIGH, 0.0)
p.resetJointState(robot, LEFT_KNEE, 0.0)
p.resetJointState(robot, LEFT_FOOT, 0.0)

# -----------------------------
# Main loop
# -----------------------------
while True:
    t = time.time()

    # phase variable
    s = math.sin(3.0 * t)
    c = math.cos(3.0 * t)

    # More obvious stepping path:
    # x: back and forth
    # z: lift during forward swing
    x = 0.045 * s
    z = -0.16 + 0.05 * max(0.0, c)

    thigh_angle, knee_angle = leg_ik(x, z, L1, L2)

    # IMPORTANT:
    # Start with these signs.
    # If the leg still moves the wrong way, flip one or both.
    right_thigh_cmd = thigh_angle
    right_knee_cmd = knee_angle

    p.setJointMotorControl2(
        robot, RIGHT_HIP, p.POSITION_CONTROL,
        targetPosition=0.0, force=500, maxVelocity=4.0
    )
    p.setJointMotorControl2(
        robot, RIGHT_THIGH, p.POSITION_CONTROL,
        targetPosition=right_thigh_cmd, force=900, maxVelocity=8.0
    )
    p.setJointMotorControl2(
        robot, RIGHT_KNEE, p.POSITION_CONTROL,
        targetPosition=right_knee_cmd, force=900, maxVelocity=8.0
    )
    p.setJointMotorControl2(
        robot, RIGHT_FOOT, p.POSITION_CONTROL,
        targetPosition=0.0, force=600, maxVelocity=4.0
    )

    # Keep left leg fixed
    p.setJointMotorControl2(
        robot, LEFT_HIP, p.POSITION_CONTROL,
        targetPosition=0.0, force=500, maxVelocity=4.0
    )
    p.setJointMotorControl2(
        robot, LEFT_THIGH, p.POSITION_CONTROL,
        targetPosition=0.0, force=500, maxVelocity=4.0
    )
    p.setJointMotorControl2(
        robot, LEFT_KNEE, p.POSITION_CONTROL,
        targetPosition=0.0, force=500, maxVelocity=4.0
    )
    p.setJointMotorControl2(
        robot, LEFT_FOOT, p.POSITION_CONTROL,
        targetPosition=0.0, force=600, maxVelocity=4.0
    )

    p.stepSimulation()
    time.sleep(1 / 240)