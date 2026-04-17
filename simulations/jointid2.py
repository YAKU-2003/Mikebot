import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")

robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.35],
    useFixedBase=True
)

joint_id = 1

# move forward (~12 sec)
for _ in range(3000):
    p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 0.5, force=30)
    p.stepSimulation()
    time.sleep(1/240)

# move backward (~12 sec)
for _ in range(3000):
    p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, -0.5, force=30)
    p.stepSimulation()
    time.sleep(1/240)

# hold center (~12 sec)
for _ in range(3000):
    p.setJointMotorControl2(robot, joint_id, p.POSITION_CONTROL, 0.0, force=30)
    p.stepSimulation()
    time.sleep(1/240)