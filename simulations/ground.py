import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.14],
    useFixedBase=False
)

# Joint map
right_hip = 1
right_thigh = 3
right_knee = 5
right_foot = 7

left_hip = 9
left_thigh = 11
left_knee = 13
left_foot = 15

# Start in a crouched pose
p.resetJointState(robot, right_hip, 0.0)
p.resetJointState(robot, right_thigh, 0.15)
p.resetJointState(robot, right_knee, -0.55)
p.resetJointState(robot, right_foot, 0.25)

p.resetJointState(robot, left_hip, 0.0)
p.resetJointState(robot, left_thigh, 0.15)
p.resetJointState(robot, left_knee, -0.55)
p.resetJointState(robot, left_foot, 0.25)

# Hold that pose so it settles instead of collapsing instantly
for _ in range(3000):
    p.setJointMotorControl2(robot, right_hip, p.POSITION_CONTROL, 0.0, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, right_thigh, p.POSITION_CONTROL, 0.15, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, right_knee, p.POSITION_CONTROL, -0.55, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, right_foot, p.POSITION_CONTROL, 0.25, force=250, maxVelocity=8)

    p.setJointMotorControl2(robot, left_hip, p.POSITION_CONTROL, 0.0, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, left_thigh, p.POSITION_CONTROL, 0.15, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, left_knee, p.POSITION_CONTROL, -0.55, force=300, maxVelocity=8)
    p.setJointMotorControl2(robot, left_foot, p.POSITION_CONTROL, 0.25, force=250, maxVelocity=8)

    p.stepSimulation()
    time.sleep(1/240)