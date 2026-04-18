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
    basePosition=[0, 0, 0.40],
    useFixedBase=True
)

p.resetDebugVisualizerCamera(
    cameraDistance=1.6,
    cameraYaw=50,
    cameraPitch=-15,
    cameraTargetPosition=[0, 0, 0.22]
)

# -----------------------------
# Joint mapping
# -----------------------------
RIGHT_HIP = 1
RIGHT_THIGH = 3
RIGHT_KNEE = 5
RIGHT_FOOT = 7

LEFT_HIP = 9
LEFT_THIGH = 11
LEFT_KNEE = 13
LEFT_FOOT = 15

# -----------------------------
# Current pose state
# mirrored signs: right = +, left = -
# -----------------------------
pose = {
    "hip": 0.0,
    "thigh": 0.05,
    "knee": 0.0,
    "foot": 0.0
}

# -----------------------------
# Apply pose
# -----------------------------
def apply_pose(hip, thigh, knee, foot,
               force_hip=1200, force_thigh=1800, force_knee=1800, force_foot=1500,
               vel_hip=4, vel_thigh=8, vel_knee=8, vel_foot=4):

    # Right side
    p.setJointMotorControl2(robot, RIGHT_HIP, p.POSITION_CONTROL,
                            targetPosition=hip, force=force_hip, maxVelocity=vel_hip)
    p.setJointMotorControl2(robot, RIGHT_THIGH, p.POSITION_CONTROL,
                            targetPosition=thigh, force=force_thigh, maxVelocity=vel_thigh)
    p.setJointMotorControl2(robot, RIGHT_KNEE, p.POSITION_CONTROL,
                            targetPosition=knee, force=force_knee, maxVelocity=vel_knee)
    p.setJointMotorControl2(robot, RIGHT_FOOT, p.POSITION_CONTROL,
                            targetPosition=foot, force=force_foot, maxVelocity=vel_foot)

    # Left side (mirrored)
    p.setJointMotorControl2(robot, LEFT_HIP, p.POSITION_CONTROL,
                            targetPosition=-hip, force=force_hip, maxVelocity=vel_hip)
    p.setJointMotorControl2(robot, LEFT_THIGH, p.POSITION_CONTROL,
                            targetPosition=-thigh, force=force_thigh, maxVelocity=vel_thigh)
    p.setJointMotorControl2(robot, LEFT_KNEE, p.POSITION_CONTROL,
                            targetPosition=-knee, force=force_knee, maxVelocity=vel_knee)
    p.setJointMotorControl2(robot, LEFT_FOOT, p.POSITION_CONTROL,
                            targetPosition=-foot, force=force_foot, maxVelocity=vel_foot)

def hold_current_pose(duration):
    steps = int(duration * 240)
    for _ in range(steps):
        apply_pose(pose["hip"], pose["thigh"], pose["knee"], pose["foot"])
        p.stepSimulation()
        time.sleep(1/240)

def move_single_joint(joint_name, target_value, duration,
                      force_hip=1200, force_thigh=1800, force_knee=1800, force_foot=1500,
                      vel_hip=4, vel_thigh=8, vel_knee=8, vel_foot=4):
    start_value = pose[joint_name]
    steps = max(1, int(duration * 240))

    for i in range(steps):
        alpha = (i + 1) / steps
        current_value = (1 - alpha) * start_value + alpha * target_value

        temp_pose = pose.copy()
        temp_pose[joint_name] = current_value

        apply_pose(
            temp_pose["hip"], temp_pose["thigh"], temp_pose["knee"], temp_pose["foot"],
            force_hip=force_hip, force_thigh=force_thigh, force_knee=force_knee, force_foot=force_foot,
            vel_hip=vel_hip, vel_thigh=vel_thigh, vel_knee=vel_knee, vel_foot=vel_foot
        )
        p.stepSimulation()
        time.sleep(1/240)

    pose[joint_name] = target_value

# -----------------------------
# Start from stand
# -----------------------------
apply_pose(pose["hip"], pose["thigh"], pose["knee"], pose["foot"])
hold_current_pose(2.0)

# -----------------------------
# Main loop
# -----------------------------
while True:
    # 1. hips move first, others locked
    print("State 1: hips move")
    move_single_joint("hip", 0.08, duration=0.6, vel_hip=8)
    hold_current_pose(0.4)

    # 2. hips locked, thighs move
    print("State 2: thighs move")
    move_single_joint("thigh", 0.22, duration=0.8, vel_thigh=10)
    hold_current_pose(0.4)

    # 3. hips+thighs locked, knees move
    print("State 3: knees move")
    move_single_joint("knee", 0.28, duration=0.8, vel_knee=10)
    hold_current_pose(0.8)

    # 4. reverse: knees back
    print("State 4: knees return")
    move_single_joint("knee", 0.0, duration=0.6, vel_knee=10)
    hold_current_pose(0.3)

    # 5. thighs back
    print("State 5: thighs return")
    move_single_joint("thigh", 0.05, duration=0.6, vel_thigh=10)
    hold_current_pose(0.3)

    # 6. hips back
    print("State 6: hips return")
    move_single_joint("hip", 0.0, duration=0.5, vel_hip=8)
    hold_current_pose(1.0)