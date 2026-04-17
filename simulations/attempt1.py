import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

robot = p.loadURDF(
    "C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.1]
)

while True:
    p.stepSimulation()
    time.sleep(1./240.)
