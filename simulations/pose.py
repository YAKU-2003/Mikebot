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
    basePosition=[0, 0, 0.36],
    useFixedBase=False
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.8,
    cameraYaw=50,
    cameraPitch=-18,
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
# mirrored signs: right = +, left = -
# -----------------------------
init_hip = 0.00
init_thigh = 0.10
init_knee = 0.12
init_foot = 0.00

p.resetJointState(robot, RIGHT_HIP, init_hip)
p.resetJointState(robot, RIGHT_THIGH, init_thigh)
p.resetJointState(robot, RIGHT_KNEE, init_knee)
p.resetJointState(robot, RIGHT_FOOT, init_foot)

p.resetJointState(robot, LEFT_HIP, -init_hip)
p.resetJointState(robot, LEFT_THIGH, -init_thigh)
p.resetJointState(robot, LEFT_KNEE, -init_knee)
p.resetJointState(robot, LEFT_FOOT, -init_foot)

# -----------------------------
# Sliders
# -----------------------------
hip_slider = p.addUserDebugParameter("Hip (mirrored)", -0.5, 0.5, init_hip)
thigh_slider = p.addUserDebugParameter("Thigh (mirrored)", -0.8, 0.8, init_thigh)
knee_slider = p.addUserDebugParameter("Knee (mirrored)", -0.8, 0.8, init_knee)
foot_slider = p.addUserDebugParameter("Foot (mirrored)", -0.5, 0.5, init_foot)

# optional force sliders
force_slider = p.addUserDebugParameter("Main Joint Force", 100, 3000, 1200)
foot_force_slider = p.addUserDebugParameter("Foot Force", 100, 3000, 1000)

# -----------------------------
# Main loop
# -----------------------------
while True:
    hip_val = p.readUserDebugParameter(hip_slider)
    thigh_val = p.readUserDebugParameter(thigh_slider)
    knee_val = p.readUserDebugParameter(knee_slider)
    foot_val = p.readUserDebugParameter(foot_slider)

    main_force = p.readUserDebugParameter(force_slider)
    foot_force = p.readUserDebugParameter(foot_force_slider)

    # Right side
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL,
                            targetPosition=hip_val, force=main_force, maxVelocity=4.0)
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=thigh_val, force=main_force, maxVelocity=4.0)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=knee_val, force=main_force, maxVelocity=4.0)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL,
                            targetPosition=foot_val, force=foot_force, maxVelocity=3.0)

    # Left side (mirrored)
    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL,
                            targetPosition=-hip_val, force=main_force, maxVelocity=4.0)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=-thigh_val, force=main_force, maxVelocity=4.0)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=-knee_val, force=main_force, maxVelocity=4.0)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL,
                            targetPosition=-foot_val, force=foot_force, maxVelocity=3.0)

    p.stepSimulation()
    time.sleep(1 / 240)