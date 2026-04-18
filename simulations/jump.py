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
    basePosition=[0, 0, 0.35],
    useFixedBase=False   # IMPORTANT for jump
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
# Pose definitions
# -----------------------------
def set_pose(thigh, knee):
    # adjust signs if needed
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=thigh, force=1200, maxVelocity=10)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=knee, force=1200, maxVelocity=10)

    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=-thigh, force=1200, maxVelocity=10)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=-knee, force=1200, maxVelocity=10)

    # keep rest fixed
    for j in [RIGHT_HIP, LEFT_HIP, RIGHT_FOOT, LEFT_FOOT]:
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                targetPosition=0.0, force=800, maxVelocity=6)

# -----------------------------
# Simulation loop
# -----------------------------
while True:

    # --- STEP 1: CROUCH ---
    for _ in range(120):
        set_pose(thigh=0.25, knee=-0.4)
        p.stepSimulation()
        time.sleep(1/240)

    # --- STEP 2: EXPLOSIVE PUSH ---
    for _ in range(30):
        set_pose(thigh=0.0, knee=0.0)
        p.stepSimulation()
        time.sleep(1/240)

    # --- STEP 3: AIR TIME ---
    for _ in range(120):
        p.stepSimulation()
        time.sleep(1/240)

    # --- STEP 4: LAND RESET ---
    for _ in range(120):
        set_pose(thigh=0.1, knee=-0.2)
        p.stepSimulation()
        time.sleep(1/240)