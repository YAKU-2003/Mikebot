import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")

robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.45],
    useFixedBase=False
)

time.sleep(1)

total_mass = 0.0
weighted_x = 0.0
weighted_y = 0.0
weighted_z = 0.0

# Base link
base_mass = p.getDynamicsInfo(robot, -1)[0]
base_pos = p.getBasePositionAndOrientation(robot)[0]

total_mass += base_mass
weighted_x += base_mass * base_pos[0]
weighted_y += base_mass * base_pos[1]
weighted_z += base_mass * base_pos[2]

# Other links
num_joints = p.getNumJoints(robot)

for i in range(num_joints):
    link_mass = p.getDynamicsInfo(robot, i)[0]
    link_state = p.getLinkState(robot, i, computeForwardKinematics=True)
    link_pos = link_state[0]   # world COM position of the link

    total_mass += link_mass
    weighted_x += link_mass * link_pos[0]
    weighted_y += link_mass * link_pos[1]
    weighted_z += link_mass * link_pos[2]

com_x = weighted_x / total_mass
com_y = weighted_y / total_mass
com_z = weighted_z / total_mass

print("Total mass:", total_mass)
print("Center of Mass (COM):")
print(f"x = {com_x:.4f}, y = {com_y:.4f}, z = {com_z:.4f}")

for _ in range(5000):
    p.stepSimulation()
    time.sleep(1/240)
