import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load ground
p.loadURDF("plane.urdf")

# Load robot as a free body
robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.14],
    useFixedBase=False
)

# Optional: reduce bounciness and improve contact behavior
p.changeDynamics(robot, -1, restitution=0.0, lateralFriction=1.0)
for j in range(p.getNumJoints(robot)):
    p.changeDynamics(robot, j, restitution=0.0, lateralFriction=1.0)

# Camera
p.resetDebugVisualizerCamera(
    cameraDistance=1.2,
    cameraYaw=45,
    cameraPitch=-20,
    cameraTargetPosition=[0, 0, 0.15]
)

# Joint mapping
right_hip = 1
right_thigh = 3
right_knee = 5
right_foot = 7

left_hip = 9
left_thigh = 11
left_knee = 13
left_foot = 15

# Initial crouched pose
right_hip_init = 0.0
right_thigh_init = 0.15
right_knee_init = -0.55
right_foot_init = 0.0   # foot locked

left_hip_init = 0.0
left_thigh_init = 0.15
left_knee_init = -0.55
left_foot_init = 0.0    # foot locked

# Instantly set initial pose
p.resetJointState(robot, right_hip, right_hip_init)
p.resetJointState(robot, right_thigh, right_thigh_init)
p.resetJointState(robot, right_knee, right_knee_init)
p.resetJointState(robot, right_foot, right_foot_init)

p.resetJointState(robot, left_hip, left_hip_init)
p.resetJointState(robot, left_thigh, left_thigh_init)
p.resetJointState(robot, left_knee, left_knee_init)
p.resetJointState(robot, left_foot, left_foot_init)

# Hold the pose for a while
for _ in range(5000):
    # Right leg
    p.setJointMotorControl2(robot, right_hip, p.POSITION_CONTROL,
                            targetPosition=right_hip_init, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, right_thigh, p.POSITION_CONTROL,
                            targetPosition=right_thigh_init, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, right_knee, p.POSITION_CONTROL,
                            targetPosition=right_knee_init, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, right_foot, p.POSITION_CONTROL,
                            targetPosition=right_foot_init, force=500, maxVelocity=5)

    # Left leg
    p.setJointMotorControl2(robot, left_hip, p.POSITION_CONTROL,
                            targetPosition=left_hip_init, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, left_thigh, p.POSITION_CONTROL,
                            targetPosition=left_thigh_init, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, left_knee, p.POSITION_CONTROL,
                            targetPosition=left_knee_init, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, left_foot, p.POSITION_CONTROL,
                            targetPosition=left_foot_init, force=500, maxVelocity=5)

    p.stepSimulation()
    time.sleep(1 / 240)