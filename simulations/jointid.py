import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF(
    "C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.3],
    useFixedBase=False
)

num_joints = p.getNumJoints(robot)
print("Number of joints:", num_joints)

for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    joint_name = info[1].decode("utf-8")
    joint_type = info[2]
    print(f"Joint ID: {i}, Joint Name: {joint_name}, Joint Type: {joint_type}")

while True:
    p.stepSimulation()
    time.sleep(1/240)