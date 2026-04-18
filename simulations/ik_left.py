import pybullet as p
import pybullet_data
import time
import math

# -----------------------------
# IK solver for 2-link leg
# -----------------------------
def leg_ik(x, z, L1, L2):
    D = (x * x + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    D = max(-1.0, min(1.0, D))

    # knee-bent solution
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
# Link lengths from CAD
# -----------------------------
L1 = 0.08   # thigh length (hip/thigh joint to knee)
L2 = 0.12   # knee to foot joint

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
# Main loop: moving foot trajectory
# -----------------------------
while True:
    t = time.time()

    # IK-based stepping trajectory for right foot
    x = 0.03 * math.sin(4 * t)
    z = -0.16 + 0.04 * max(0.0, math.sin(4 * t))

    thigh_angle, knee_angle = leg_ik(x, z, L1, L2)

    print_thigh = thigh_angle
    print_knee = knee_angle

    # IMPORTANT:
    # If the physical motion is wrong, flip signs here.
    right_thigh_cmd = thigh_angle
    right_knee_cmd = knee_angle

    # Keep unused joints fixed
    p.setJointMotorControl2(
        robot, RIGHT_HIP, p.POSITION_CONTROL,
        targetPosition=0.0, force=500, maxVelocity=4.0
    )
    p.setJointMotorControl2(
        robot, RIGHT_THIGH, p.POSITION_CONTROL,
        targetPosition=right_thigh_cmd, force=800, maxVelocity=6.0
    )
    p.setJointMotorControl2(
        robot, RIGHT_KNEE, p.POSITION_CONTROL,
        targetPosition=right_knee_cmd, force=800, maxVelocity=6.0
    )
    p.setJointMotorControl2(
        robot, RIGHT_FOOT, p.POSITION_CONTROL,
        targetPosition=0.0, force=600, maxVelocity=4.0
    )

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