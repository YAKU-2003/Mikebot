import pybullet as p
import time
import pybullet_data

# Connect to physics server
physicsClient = p.connect(p.GUI)

if physicsClient < 0:
    print("Failed to connect to GUI, switching to DIRECT mode")
    physicsClient = p.connect(p.DIRECT)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf")

cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("r2d2.urdf", cubeStartPos, cubeStartOrientation)

for i in range(10000):
    if not p.isConnected(physicsClient):
        print(f"Lost connection at step {i}")
        break
    p.stepSimulation()
    time.sleep(1. / 240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print("Final Position:", cubePos)
print("Final Orientation:", cubeOrn)

p.disconnect()