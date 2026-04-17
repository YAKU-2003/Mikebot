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
# Initial pose
# -----------------------------
p.resetJointState(robot, RIGHT_HIP, 0.0)
p.resetJointState(robot, RIGHT_THIGH, 0.05)
p.resetJointState(robot, RIGHT_KNEE, 0.0)
p.resetJointState(robot, RIGHT_FOOT, 0.0)

p.resetJointState(robot, LEFT_HIP, 0.0)
p.resetJointState(robot, LEFT_THIGH, -0.05)
p.resetJointState(robot, LEFT_KNEE, 0.0)
p.resetJointState(robot, LEFT_FOOT, 0.0)

# -----------------------------
# Sliders
# -----------------------------
hip_slider = p.addUserDebugParameter("Hip (both)", -1.0, 1.0, 0.0)
thigh_slider = p.addUserDebugParameter("Thigh (sync opposite)", -1.0, 1.0, 0.05)
knee_slider = p.addUserDebugParameter("Knee (sync opposite)", -1.5, 1.5, 0.0)

# -----------------------------
# Main loop
# -----------------------------
while True:
    hip_val = p.readUserDebugParameter(hip_slider)
    thigh_val = p.readUserDebugParameter(thigh_slider)
    knee_val = p.readUserDebugParameter(knee_slider)

    # Hip same on both sides
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL,
                            targetPosition=hip_val, force=300, maxVelocity=1.5)
    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL,
                            targetPosition=hip_val, force=300, maxVelocity=1.5)

    # Thigh opposite signs
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=thigh_val, force=300, maxVelocity=1.5)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=-thigh_val, force=300, maxVelocity=1.5)

    # Knee opposite signs
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=knee_val, force=300, maxVelocity=1.5)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=-knee_val, force=300, maxVelocity=1.5)

    # Feet locked
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL,
                            targetPosition=0.0, force=500, maxVelocity=1.0)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL,
                            targetPosition=0.0, force=500, maxVelocity=1.0)

    p.stepSimulation()
    time.sleep(1 / 240)