import pybullet as p
import pybullet_data
import time
import math

# =========================
# Connect to PyBullet
# =========================
client = p.connect(p.GUI)   # use p.DIRECT if no window needed
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / 240.0)

# Optional but often useful for URDF debugging
p.setPhysicsEngineParameter(enableFileCaching=0)

# =========================
# Load ground and robot
# =========================
plane_id = p.loadURDF("plane.urdf")

# Change this path to your URDF
robot_start_pos = [0, 0, 0.25]
robot_start_ori = p.getQuaternionFromEuler([0, 0, 0])

robot_id = p.loadURDF(
    "simready1.urdf",
    robot_start_pos,
    robot_start_ori,
    useFixedBase=False
)

# =========================
# Print joint info
# =========================
print("\n=== JOINT LIST ===")
joint_name_to_index = {}

for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    joint_index = info[0]
    joint_name = info[1].decode("utf-8")
    joint_type = info[2]

    print(f"Index {joint_index}: {joint_name}, type={joint_type}")
    joint_name_to_index[joint_name] = joint_index

# =========================
# Joint names
# Change these if your URDF uses different names
# =========================
JOINTS = {
    "left_hip_yaw": "left_hip_yaw",
    "left_thigh_pitch": "left_thigh_pitch",
    "left_knee_pitch": "left_knee_pitch",
    "left_foot_roll": "left_foot_roll",

    "right_hip_yaw": "right_hip_yaw",
    "right_thigh_pitch": "right_thigh_pitch",
    "right_knee_pitch": "right_knee_pitch",
    "right_foot_roll": "right_foot_roll",
}

# Convert names to indices
joint_indices = {}
for key, joint_name in JOINTS.items():
    if joint_name not in joint_name_to_index:
        raise ValueError(f"Joint '{joint_name}' not found in URDF.")
    joint_indices[key] = joint_name_to_index[joint_name]

# =========================
# Helper functions
# =========================
def deg_to_rad(angle_deg):
    return math.radians(angle_deg)

def command_pose_deg(pose_deg, kp_force=40):
    """
    pose_deg: dict of joint_key -> angle_in_degrees
    """
    for joint_key, angle_deg in pose_deg.items():
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_indices[joint_key],
            controlMode=p.POSITION_CONTROL,
            targetPosition=deg_to_rad(angle_deg),
            force=kp_force
        )

def step_for_duration(seconds):
    steps = int(seconds / (1.0 / 240.0))
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

def print_base_pose():
    pos, ori = p.getBasePositionAndOrientation(robot_id)
    euler = p.getEulerFromQuaternion(ori)
    print(f"Base pos: {pos}, euler: {euler}")

# =========================
# Example gait poses in DEGREES
# You must tune these for your robot
# =========================
STAND = {
    "left_hip_yaw": 0,
    "left_thigh_pitch": 0,
    "left_knee_pitch": 15,
    "left_foot_roll": 0,

    "right_hip_yaw": 0,
    "right_thigh_pitch": 0,
    "right_knee_pitch": 15,
    "right_foot_roll": 0,
}

SHIFT_LEFT = {
    "left_hip_yaw": 0,
    "left_thigh_pitch": 0,
    "left_knee_pitch": 15,
    "left_foot_roll": -6,

    "right_hip_yaw": 0,
    "right_thigh_pitch": 0,
    "right_knee_pitch": 15,
    "right_foot_roll": +6,
}

LIFT_RIGHT = {
    "left_hip_yaw": 0,
    "left_thigh_pitch": -2,
    "left_knee_pitch": 18,
    "left_foot_roll": -6,

    "right_hip_yaw": 0,
    "right_thigh_pitch": +10,
    "right_knee_pitch": 30,
    "right_foot_roll": +3,
}

PLACE_RIGHT = {
    "left_hip_yaw": 0,
    "left_thigh_pitch": 0,
    "left_knee_pitch": 15,
    "left_foot_roll": -2,

    "right_hip_yaw": 0,
    "right_thigh_pitch": +5,
    "right_knee_pitch": 18,
    "right_foot_roll": 0,
}

SHIFT_RIGHT = {
    "left_hip_yaw": 0,
    "left_thigh_pitch": 0,
    "left_knee_pitch": 15,
    "left_foot_roll": +6,

    "right_hip_yaw": 0,
    "right_thigh_pitch": 0,
    "right_knee_pitch": 15,
    "right_foot_roll": -6,
}

LIFT_LEFT = {
    "left_hip_yaw": 0,
    "left_thigh_pitch": +10,
    "left_knee_pitch": 30,
    "left_foot_roll": +3,

    "right_hip_yaw": 0,
    "right_thigh_pitch": -2,
    "right_knee_pitch": 18,
    "right_foot_roll": -6,
}

PLACE_LEFT = {
    "left_hip_yaw": 0,
    "left_thigh_pitch": +5,
    "left_knee_pitch": 18,
    "left_foot_roll": 0,

    "right_hip_yaw": 0,
    "right_thigh_pitch": 0,
    "right_knee_pitch": 15,
    "right_foot_roll": -2,
}

# =========================
# Stabilize initial state
# =========================
command_pose_deg(STAND, kp_force=60)
step_for_duration(2.0)
print_base_pose()

# =========================
# Test each phase once
# =========================
test_sequence = [
    ("STAND", STAND, 1.0),
    ("SHIFT_LEFT", SHIFT_LEFT, 0.8),
    ("LIFT_RIGHT", LIFT_RIGHT, 0.8),
    ("PLACE_RIGHT", PLACE_RIGHT, 0.8),
    ("STAND", STAND, 0.8),
    ("SHIFT_RIGHT", SHIFT_RIGHT, 0.8),
    ("LIFT_LEFT", LIFT_LEFT, 0.8),
    ("PLACE_LEFT", PLACE_LEFT, 0.8),
    ("STAND", STAND, 1.0),
]

for name, pose, duration in test_sequence:
    print(f"Running pose: {name}")
    command_pose_deg(pose, kp_force=60)
    step_for_duration(duration)

# =========================
# Walking loop
# =========================
for cycle in range(5):
    print(f"Walk cycle {cycle + 1}")

    command_pose_deg(SHIFT_LEFT, kp_force=60)
    step_for_duration(0.35)

    command_pose_deg(LIFT_RIGHT, kp_force=60)
    step_for_duration(0.35)

    command_pose_deg(PLACE_RIGHT, kp_force=60)
    step_for_duration(0.30)

    command_pose_deg(STAND, kp_force=60)
    step_for_duration(0.25)

    command_pose_deg(SHIFT_RIGHT, kp_force=60)
    step_for_duration(0.35)

    command_pose_deg(LIFT_LEFT, kp_force=60)
    step_for_duration(0.35)

    command_pose_deg(PLACE_LEFT, kp_force=60)
    step_for_duration(0.30)

    command_pose_deg(STAND, kp_force=60)
    step_for_duration(0.25)

print("Done. Keeping sim open...")
while True:
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
