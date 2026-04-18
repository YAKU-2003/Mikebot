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
    basePosition=[0, 0, 0.4],
    useFixedBase=False
)

# -----------------------------
# Compute centroid
# -----------------------------
positions = []

base_pos, _ = p.getBasePositionAndOrientation(robot)
positions.append(base_pos)


# Other links

num_joints = p.getNumJoints(robot)

for i in range(num_joints):
    link_state = p.getLinkState(robot, i)
    link_pos = link_state[0]

    positions.append(link_pos)


# Average
n = len(positions)

centroid_x = sum(pos[0] for pos in positions) / n

centroid_y = sum(pos[1] for pos in positions) / n
centroid_z = sum(pos[2] for pos in positions) / n

print("Centroid:")
print(f"x = {centroid_x:.4f}")
print(f"y = {centroid_y:.4f}")
print(f"z = {centroid_z:.4f}")

# -----------------------------
# Visualize centroid (red sphere)

# -----------------------------
sphere_radius = 0.03


visual_shape = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=sphere_radius,
    rgbaColor=[1, 0, 0, 1]   # RED

)

centroid_marker = p.createMultiBody(
    baseMass=0,
    baseVisualShapeIndex=visual_shape,
    basePosition=[centroid_x, centroid_y, centroid_z]
)

# -----------------------------
# Keep simulation running
# -----------------------------
while True:
    p.stepSimulation()

    time.sleep(1 / 240)
