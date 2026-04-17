import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")

robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.8],
    useFixedBase=True
)

joint_id = 11   # try knee

t = 0

while True:
    target = 1.0 * math.sin(t)   # amplitude

    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_id,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target,
        force=200,
        maxVelocity=10
    )

    p.stepSimulation()
    time.sleep(1/240)

    t += 0.1   # 🔥 THIS controls speed
