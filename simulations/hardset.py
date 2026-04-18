import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.40],
    useFixedBase=True
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.6,
    cameraYaw=50,
    cameraPitch=-15,
    cameraTargetPosition=[0, 0, 0.22]
)

# print joint info again
print("Num joints:", p.getNumJoints(robot))
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    print(i, info[1].decode(), "type=", info[2], "lower=", info[8], "upper=", info[9])

RIGHT_THIGH = 3
RIGHT_KNEE = 5
LEFT_THIGH = 11
LEFT_KNEE = 13

# neutral first
p.resetJointState(robot, RIGHT_THIGH, 0.0)
p.resetJointState(robot, RIGHT_KNEE, 0.0)
p.resetJointState(robot, LEFT_THIGH, 0.0)
p.resetJointState(robot, LEFT_KNEE, 0.0)

for _ in range(240):
    p.stepSimulation()
    time.sleep(1/240)

# hard set to obvious values
p.resetJointState(robot, RIGHT_THIGH, 0.8)
p.resetJointState(robot, RIGHT_KNEE, -0.8)
p.resetJointState(robot, LEFT_THIGH, -0.8)
p.resetJointState(robot, LEFT_KNEE, 0.8)

print("Applied hard-set pose")

while True:
    p.stepSimulation()
    time.sleep(1/240)