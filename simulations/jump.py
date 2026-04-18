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
    basePosition=[0, 0, 0.45],
    useFixedBase=True   # keep stable for demo
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.6,
    cameraYaw=50,
    cameraPitch=-15,
    cameraTargetPosition=[0, 0, 0.22]
)

# -----------------------------
# Joint IDs
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
# Pose function (mirrored)
# -----------------------------
def set_pose(thigh, knee, force=1800, vel=20):

    # RIGHT (positive)
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=thigh, force=force, maxVelocity=vel)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=knee, force=force, maxVelocity=vel)

    # LEFT (negative mirror)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=-thigh, force=force, maxVelocity=vel)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=-knee, force=force, maxVelocity=vel)

    # keep hips + feet fixed
    for j in [RIGHT_HIP, LEFT_HIP, RIGHT_FOOT, LEFT_FOOT]:
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL,
                                targetPosition=0.0, force=1000, maxVelocity=10)

# -----------------------------
# Main loop
# -----------------------------
while True:

    # --- 1. STAND ---
    for _ in range(120):
        set_pose(thigh=0.05, knee=0.05, force=1200, vel=8)
        p.stepSimulation()
        time.sleep(1/240)

    # --- 2. CROUCH ---
    for _ in range(100):
        set_pose(thigh=0.28, knee=0.35, force=1800, vel=12)
        p.stepSimulation()
        time.sleep(1/240)

    # --- 3. SNAP EXTENSION (SIMULTANEOUS) ---
    for _ in range(10):
        set_pose(thigh=0.0, knee=0.0, force=2600, vel=50)
        p.stepSimulation()
        time.sleep(1/240)

    # --- 4. HOLD ---
    for _ in range(80):
        set_pose(thigh=0.0, knee=0.0, force=1500, vel=10)
        p.stepSimulation()
        time.sleep(1/240)

    # --- 5. RESET ---
    for _ in range(100):
        set_pose(thigh=0.08, knee=0.08, force=1200, vel=8)
        p.stepSimulation()
        time.sleep(1/240)