import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.40],
    useFixedBase=True
)

RIGHT_HIP = 1
RIGHT_THIGH = 3
RIGHT_KNEE = 5
RIGHT_FOOT = 7

LEFT_HIP = 9
LEFT_THIGH = 11
LEFT_KNEE = 13
LEFT_FOOT = 15

# BIG commanded pose
right_thigh = 0.6
right_knee = -0.8

left_thigh = -0.6
left_knee = 0.8

while True:
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL, 0.0, force=2000, maxVelocity=20)
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL, right_thigh, force=3000, maxVelocity=30)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL, right_knee, force=3000, maxVelocity=30)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL, 0.0, force=2000, maxVelocity=20)

    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL, 0.0, force=2000, maxVelocity=20)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL, left_thigh, force=3000, maxVelocity=30)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL, left_knee, force=3000, maxVelocity=30)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL, 0.0, force=2000, maxVelocity=20)

    p.stepSimulation()
    time.sleep(1/240)