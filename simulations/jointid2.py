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

joint_id = 1  # try 1 first (since it's revolute)

# move forward
for _ in range(300):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_id,
        controlMode=p.POSITION_CONTROL,
        targetPosition=0.5,
        force=30
    )
    p.stepSimulation()
    time.sleep(1/240)

# move backward
for _ in range(300):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_id,
        controlMode=p.POSITION_CONTROL,
        targetPosition=-0.5,
        force=30
    )
    p.stepSimulation()
    time.sleep(1/240)

# hold center
for _ in range(300):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_id,
        controlMode=p.POSITION_CONTROL,
        targetPosition=0.0,
        force=30
    )
    p.stepSimulation()
    time.sleep(1/240)