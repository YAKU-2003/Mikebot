import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("simready1.urdf", 
                   basePosition=[0, 0, 0.5],
                   useFixedBase=True)  # FIXED BASE for now — no falling

# From your URDF analysis:
FOOT_1_LINK = 7   # foot_1 (left leg)

# Add sliders to move the foot target around
sx = p.addUserDebugParameter("foot_x", -0.15, 0.15,  0.00)
sy = p.addUserDebugParameter("foot_y",  0.00, 0.20,  0.08)
sz = p.addUserDebugParameter("foot_z", -0.45, -0.05, -0.30)

while True:
    # Read slider values
    x = p.readUserDebugParameter(sx)
    y = p.readUserDebugParameter(sy)
    z = p.readUserDebugParameter(sz)

    # Target position in world space
    target = [x, y, z]

    # Ask PyBullet to solve IK
    joint_angles = p.calculateInverseKinematics(
        robot,
        endEffectorLinkIndex=FOOT_1_LINK,
        targetPosition=target,
        maxNumIterations=100,
        residualThreshold=1e-5
    )

    # Apply ALL joints from IK solution
    for i in range(p.getNumJoints(robot)):
        p.setJointMotorControl2(
            robot, i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_angles[i],
            force=50
        )

    # Draw a red dot at the target so you can see where you're aiming
    p.addUserDebugPoints([target], [[1, 0, 0]], pointSize=10, lifeTime=0.1)

    # Print actual foot position vs target
    actual = p.getLinkState(robot, FOOT_1_LINK)[0]
    print(f"Target: {[round(v,3) for v in target]}  "
          f"Actual: {[round(v,3) for v in actual]}  "
          f"Error: {round(sum((a-b)**2 for a,b in zip(target,actual))**0.5 * 1000, 1)}mm", 
          end='\r')

    p.stepSimulation()
    time.sleep(1./240.)
