import pybullet as p
import pybullet_data
import time

# Start PyBullet GUI
p.connect(p.GUI)

# Gravity
p.setGravity(0, 0, -9.81)

# Load ground plane
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

# Load your robot
robot = p.loadURDF(
    "urdf/simready.urdf",
    basePosition=[0, 0, 0.3],
    useFixedBase=True
)

# Print joint info
num_joints = p.getNumJoints(robot)
print("Number of joints:", num_joints)

for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    print(i, info[1].decode("utf-8"))

# Keep simulation running
while True:
    p.stepSimulation()
    time.sleep(1 / 240)