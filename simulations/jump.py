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
    useFixedBase=True
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=50,
    cameraPitch=-15,
    cameraTargetPosition=[0, 0, 0.2]
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
# Helper
# -----------------------------
def set_pose(thigh, knee, force=1800, max_vel=20):
    # right side
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=thigh, force=force, maxVelocity=max_vel)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=knee, force=force, maxVelocity=max_vel)

    # left side mirrored
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=-thigh, force=force, maxVelocity=max_vel)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=-knee, force=force, maxVelocity=max_vel)

    # keep hip and feet fixed
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL,
                            targetPosition=0.0, force=1000, maxVelocity=10)
    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL,
                            targetPosition=0.0, force=1000, maxVelocity=10)

    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL,
                            targetPosition=0.0, force=1000, maxVelocity=10)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL,
                            targetPosition=0.0, force=1000, maxVelocity=10)

# -----------------------------
# Main loop
# -----------------------------
while True:
    # stand
    for _ in range(120):
        set_pose(thigh=0.05, knee=0.0, force=1200, max_vel=8)
        p.stepSimulation()
        time.sleep(1/240)

    # crouch
    for _ in range(90):
        set_pose(thigh=0.28, knee=-0.35, force=1600, max_vel=10)
        p.stepSimulation()
        time.sleep(1/240)

    # explosive extension (simultaneous)
    for _ in range(10):
        set_pose(thigh=0.0, knee=0.0, force=2600, max_vel=45)
        p.stepSimulation()
        time.sleep(1/240)

    # hold extended pose briefly
    for _ in range(60):
        set_pose(thigh=0.0, knee=0.0, force=1600, max_vel=12)
        p.stepSimulation()
        time.sleep(1/240)

    # return to soft stand
    for _ in range(90):
        set_pose(thigh=0.05, knee=0.0, force=1200, max_vel=8)
        p.stepSimulation()
        time.sleep(1/240)