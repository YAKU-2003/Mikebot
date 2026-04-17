import pybullet as p
import pybullet_data
import time
import math
import os

# =============================
# Paths
# =============================
URDF_DIR = r"C:\Users\ykulk\Downloads\simulations\attempt1"
URDF_PATH = r"C:\Users\ykulk\Downloads\simulations\attempt1\simready1.urdf"
PLANE_PATH = os.path.join(pybullet_data.getDataPath(), "plane.urdf")

# =============================
# Connect to PyBullet
# =============================
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / 240.0)

# =============================
# Debug path check
# =============================
print("Running from:", os.getcwd())
print("URDF exists:", os.path.exists(URDF_PATH))
print("URDF dir exists:", os.path.exists(URDF_DIR))
print("Plane exists:", os.path.exists(PLANE_PATH))

# =============================
# Load plane and robot
# =============================
plane = p.loadURDF(PLANE_PATH)

robot_start_pos = [0, 0, 0.45]
robot_start_ori = p.getQuaternionFromEuler([0, 0, 0])

robot = p.loadURDF(
    URDF_PATH,
    robot_start_pos,
    robot_start_ori,
    useFixedBase=False
)

# =============================
# Print joint map
# =============================
print("\n=== JOINT MAP ===")
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    joint_name = info[1].decode("utf-8")
    child_link = info[12].decode("utf-8")
    joint_type = info[2]
    print(f"Index {i}: joint='{joint_name}', type={joint_type}, child link='{child_link}'")

# =============================
# JOINT INDICES
# Change these if wrong
# =============================
RIGHT_THIGH = 5
LEFT_THIGH = 1

RIGHT_KNEE = 6
LEFT_KNEE = 2

RIGHT_FOOT = 7
LEFT_FOOT = 3

# =============================
# Disable default motors
# =============================
for i in range(p.getNumJoints(robot)):
    p.setJointMotorControl2(
        robot,
        i,
        controlMode=p.VELOCITY_CONTROL,
        force=0
    )

# =============================
# Sliders (radians)
# =============================
thigh_slider = p.addUserDebugParameter("thigh angle (rad)", -0.8, 0.8, 0.0)
knee_slider = p.addUserDebugParameter("knee angle (rad)", -1.2, 1.2, 0.2)

# =============================
# Debug helpers
# =============================
def debug_joint(robot_id, joint_index, name):
    state = p.getJointState(robot_id, joint_index)
    pos = state[0]
    vel = state[1]
    torque = state[3]

    print(
        f"{name}: "
        f"{pos:.3f} rad ({math.degrees(pos):.1f} deg), "
        f"vel={vel:.3f}, torque={torque:.3f}"
    )

def print_debug(thigh_val, knee_val):
    print("\n--- DEBUG ---")
    print(f"Thigh slider: {thigh_val:.3f} rad ({math.degrees(thigh_val):.1f} deg)")
    print(f"Knee slider : {knee_val:.3f} rad ({math.degrees(knee_val):.1f} deg)")

    debug_joint(robot, RIGHT_THIGH, "RIGHT_THIGH")
    debug_joint(robot, LEFT_THIGH, "LEFT_THIGH")
    debug_joint(robot, RIGHT_KNEE, "RIGHT_KNEE")
    debug_joint(robot, LEFT_KNEE, "LEFT_KNEE")
    debug_joint(robot, RIGHT_FOOT, "RIGHT_FOOT")
    debug_joint(robot, LEFT_FOOT, "LEFT_FOOT")

# =============================
# Let robot settle
# =============================
for _ in range(240):
    p.stepSimulation()
    time.sleep(1 / 240)

# =============================
# Main loop
# =============================
last_thigh = None
last_knee = None

try:
    while True:
        thigh_val = p.readUserDebugParameter(thigh_slider)
        knee_val = p.readUserDebugParameter(knee_slider)

        # Thighs: opposite signs
        p.setJointMotorControl2(
            robot,
            RIGHT_THIGH,
            p.POSITION_CONTROL,
            targetPosition=thigh_val,
            force=300,
            maxVelocity=1.5
        )

        p.setJointMotorControl2(
            robot,
            LEFT_THIGH,
            p.POSITION_CONTROL,
            targetPosition=-thigh_val,
            force=300,
            maxVelocity=1.5
        )

        # Knees: opposite signs
        p.setJointMotorControl2(
            robot,
            RIGHT_KNEE,
            p.POSITION_CONTROL,
            targetPosition=knee_val,
            force=300,
            maxVelocity=1.5
        )

        p.setJointMotorControl2(
            robot,
            LEFT_KNEE,
            p.POSITION_CONTROL,
            targetPosition=-knee_val,
            force=300,
            maxVelocity=1.5
        )

        # Feet locked
        p.setJointMotorControl2(
            robot,
            RIGHT_FOOT,
            p.POSITION_CONTROL,
            targetPosition=0.0,
            force=500,
            maxVelocity=1.0
        )

        p.setJointMotorControl2(
            robot,
            LEFT_FOOT,
            p.POSITION_CONTROL,
            targetPosition=0.0,
            force=500,
            maxVelocity=1.0
        )

        p.stepSimulation()
        time.sleep(1 / 240)

        changed = (
            last_thigh is None or
            abs(thigh_val - last_thigh) > 0.02 or
            abs(knee_val - last_knee) > 0.02
        )

        if changed:
            print_debug(thigh_val, knee_val)
            last_thigh = thigh_val
            last_knee = knee_val

except KeyboardInterrupt:
    print("\nStopped by user.")
    p.disconnect()
