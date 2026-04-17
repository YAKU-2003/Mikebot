import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")

robot = p.loadURDF(
    r"C:/Users/ykulk/Downloads/simulations/attempt1/simready1.urdf",
    basePosition=[0, 0, 0.40],
    useFixedBase=False
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-20,
    cameraTargetPosition=[0, 0, 0.22]
)

# Joint mapping
right_hip = 1
right_thigh = 3
right_knee = 5
right_foot = 7

left_hip = 9
left_thigh = 11
left_knee = 13
left_foot = 15

# Stable initial pose
right_hip_init = 0.0
right_thigh_init = 0.05
right_knee_init = 0.0
right_foot_init = 0.0

left_hip_init = 0.0
left_thigh_init = 0.05
left_knee_init = 0.0
left_foot_init = 0.0

# Apply initial pose
p.resetJointState(robot, right_hip, right_hip_init)
p.resetJointState(robot, right_thigh, right_thigh_init)
p.resetJointState(robot, right_knee, right_knee_init)
p.resetJointState(robot, right_foot, right_foot_init)

p.resetJointState(robot, left_hip, left_hip_init)
p.resetJointState(robot, left_thigh, left_thigh_init)
p.resetJointState(robot, left_knee, left_knee_init)
p.resetJointState(robot, left_foot, left_foot_init)

def hold_pose(duration,
              r_hip, r_thigh, r_knee, r_foot,
              l_hip, l_thigh, l_knee, l_foot):
    steps = int(duration * 240)
    for _ in range(steps):
        p.setJointMotorControl2(robot, right_hip, p.POSITION_CONTROL, r_hip, force=250, maxVelocity=6)
        p.setJointMotorControl2(robot, right_thigh, p.POSITION_CONTROL, r_thigh, force=300, maxVelocity=6)
        p.setJointMotorControl2(robot, right_knee, p.POSITION_CONTROL, r_knee, force=300, maxVelocity=6)
        p.setJointMotorControl2(robot, right_foot, p.POSITION_CONTROL, r_foot, force=500, maxVelocity=4)

        p.setJointMotorControl2(robot, left_hip, p.POSITION_CONTROL, l_hip, force=250, maxVelocity=6)
        p.setJointMotorControl2(robot, left_thigh, p.POSITION_CONTROL, l_thigh, force=300, maxVelocity=6)
        p.setJointMotorControl2(robot, left_knee, p.POSITION_CONTROL, l_knee, force=300, maxVelocity=6)
        p.setJointMotorControl2(robot, left_foot, p.POSITION_CONTROL, l_foot, force=500, maxVelocity=4)

        p.stepSimulation()
        time.sleep(1/240)

# Let robot settle first
hold_pose(
    2.0,
    right_hip_init, right_thigh_init, right_knee_init, right_foot_init,
    left_hip_init, left_thigh_init, left_knee_init, left_foot_init
)

# Repeated step sequence
for _ in range(4):
    # RIGHT LEG: thigh first
    hold_pose(
        0.6,
        0.0, 0.30, 0.0, 0.0,
        0.0, 0.05, 0.0, 0.0
    )

    # RIGHT LEG: then knee bends
    hold_pose(
        0.6,
        0.0, 0.30, -0.45, 0.0,
        0.0, 0.05, 0.0, 0.0
    )

    # Return right leg
    hold_pose(
        0.6,
        0.0, 0.05, 0.0, 0.0,
        0.0, 0.05, 0.0, 0.0
    )

    # LEFT LEG: thigh first
    hold_pose(
        0.6,
        0.0, 0.05, 0.0, 0.0,
        0.0, 0.30, 0.0, 0.0
    )

    # LEFT LEG: then knee bends
    hold_pose(
        0.6,
        0.0, 0.05, 0.0, 0.0,
        0.0, 0.30, -0.45, 0.0
    )

    # Return left leg
    hold_pose(
        0.6,
        0.0, 0.05, 0.0, 0.0,
        0.0, 0.05, 0.0, 0.0
    )

# Hold final standing pose
hold_pose(
    2.0,
    right_hip_init, right_thigh_init, right_knee_init, right_foot_init,
    left_hip_init, left_thigh_init, left_knee_init, left_foot_init
)