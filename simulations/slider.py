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
# Initial standing pose
# -----------------------------
p.resetJointState(robot, RIGHT_HIP, 0.0)
p.resetJointState(robot, RIGHT_THIGH, 0.05)
p.resetJointState(robot, RIGHT_KNEE, 0.0)
p.resetJointState(robot, RIGHT_FOOT, 0.0)

p.resetJointState(robot, LEFT_HIP, 0.0)
p.resetJointState(robot, LEFT_THIGH, 0.05)
p.resetJointState(robot, LEFT_KNEE, 0.0)
p.resetJointState(robot, LEFT_FOOT, 0.0)

# -----------------------------
# Sliders
# -----------------------------
# Right leg
right_hip_slider = p.addUserDebugParameter("Right Hip", -1.0, 1.0, 0.0)
right_thigh_slider = p.addUserDebugParameter("Right Thigh", -1.0, 1.0, 0.05)
right_knee_slider = p.addUserDebugParameter("Right Knee", -1.5, 1.5, 0.0)

# Left leg
left_hip_slider = p.addUserDebugParameter("Left Hip", -1.0, 1.0, 0.0)
left_thigh_slider = p.addUserDebugParameter("Left Thigh", -1.0, 1.0, 0.05)
left_knee_slider = p.addUserDebugParameter("Left Knee", -1.5, 1.5, 0.0)

# Optional foot lock sliders
right_foot_slider = p.addUserDebugParameter("Right Foot", -1.0, 1.0, 0.0)
left_foot_slider = p.addUserDebugParameter("Left Foot", -1.0, 1.0, 0.0)

# -----------------------------
# Main loop
# -----------------------------
while True:
    # Read slider values
    right_hip_val = p.readUserDebugParameter(right_hip_slider)
    right_thigh_val = p.readUserDebugParameter(right_thigh_slider)
    right_knee_val = p.readUserDebugParameter(right_knee_slider)

    left_hip_val = p.readUserDebugParameter(left_hip_slider)
    left_thigh_val = p.readUserDebugParameter(left_thigh_slider)
    left_knee_val = p.readUserDebugParameter(left_knee_slider)

    right_foot_val = p.readUserDebugParameter(right_foot_slider)
    left_foot_val = p.readUserDebugParameter(left_foot_slider)

    # Apply motor commands
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL,
                            targetPosition=right_hip_val, force=300, maxVelocity=1.5)
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=right_thigh_val, force=300, maxVelocity=1.5)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=right_knee_val, force=300, maxVelocity=1.5)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL,
                            targetPosition=right_foot_val, force=500, maxVelocity=1.0)

    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL,
                            targetPosition=left_hip_val, force=300, maxVelocity=1.5)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=left_thigh_val, force=300, maxVelocity=1.5)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=left_knee_val, force=300, maxVelocity=1.5)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL,
                            targetPosition=left_foot_val, force=500, maxVelocity=1.0)

    p.stepSimulation()
    time.sleep(1 / 240)