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
    useFixedBase=False
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
def set_jump_pose(thigh, knee, force=1800, max_vel=20):
    # Right side
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=thigh, force=force, maxVelocity=max_vel)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=knee, force=force, maxVelocity=max_vel)

    # Left side (opposite signs, based on your mirrored setup)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=-thigh, force=force, maxVelocity=max_vel)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=-knee, force=force, maxVelocity=max_vel)

    # Keep hip and foot fixed
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
    # --- 1. neutral stand ---
    for _ in range(120):
        set_jump_pose(thigh=0.05, knee=0.0, force=1200, max_vel=8)
        p.stepSimulation()
        time.sleep(1/240)

    # --- 2. crouch ---
    for _ in range(80):
        set_jump_pose(thigh=0.30, knee=-0.45, force=1600, max_vel=10)
        p.stepSimulation()
        time.sleep(1/240)

    # --- 3. explosive extension (SNAPPY) ---
    # thighs + knees commanded together, very high speed/force
    for _ in range(12):
        set_jump_pose(thigh=0.0, knee=0.0, force=2500, max_vel=40)
        p.stepSimulation()
        time.sleep(1/240)

    # --- 4. airtime / recovery ---
    for _ in range(120):
        p.stepSimulation()
        time.sleep(1/240)

    # --- 5. landing stabilization ---
    for _ in range(100):
        set_jump_pose(thigh=0.10, knee=-0.10, force=1400, max_vel=12)
        p.stepSimulation()
        time.sleep(1/240)