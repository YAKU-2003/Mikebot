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

num_joints = p.getNumJoints(robot)
print("Number of joints:", num_joints)

for i in range(num_joints):
    info = p.getJointInfo(robot, i)

    joint_name = info[1].decode("utf-8")
    joint_type = info[2]
    child_link_name = info[12].decode("utf-8")
    parent_index = info[16]

    if parent_index == -1:
        parent_link_name = "base_link"
    else:
        parent_link_name = p.getJointInfo(robot, parent_index)[12].decode("utf-8")

    print(f"Joint ID: {i}")
    print(f"  Joint Name: {joint_name}")
    print(f"  Joint Type: {joint_type}")
    print(f"  Parent Link: {parent_link_name}")
    print(f"  Child Link: {child_link_name}")
    print("-" * 40)

for _ in range(3000):
    p.stepSimulation()
    time.sleep(1/240)