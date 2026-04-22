"""
Hardware + joint configuration. Edit this file when:
  - You change which serial port the LX-16A bus is on
  - You change servo IDs
  - You measure your link lengths
  - You re-mount the IMU in a different orientation

Run-time calibration (per-joint zero offset, sign) is loaded from
data/calibration.json by joints.py and overrides the defaults here.
"""

from pathlib import Path

# ---------------------------------------------------------------------------
# Bus / port
# ---------------------------------------------------------------------------
# /dev/ttyUSB0  -> LewanSoul USB->bus adapter (recommended)
# /dev/serial0  -> Raspberry Pi hardware UART (needs half-duplex circuit)
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
SERIAL_TIMEOUT_S = 0.05  # response timeout per command

# ---------------------------------------------------------------------------
# Joint -> servo ID mapping
# ---------------------------------------------------------------------------
# Joint name convention:
#   L/R = left / right side (from robot's own POV, looking forward)
#   HY  = hip yaw      (rotates leg about vertical axis)
#   TP  = thigh pitch  (a.k.a. hip pitch, swings thigh forward/back)
#   KP  = knee pitch
#   AP  = ankle pitch
JOINT_NAMES = ["LHY", "LTP", "LKP", "LAP", "RHY", "RTP", "RKP", "RAP"]

SERVO_IDS = {
    # Right leg
    "RHY": 1,
    "RTP": 2,
    "RKP": 3,
    "RAP": 4,
    # Left leg
    "LHY": 5,
    "LTP": 6,
    "LKP": 7,
    "LAP": 8,
}

# ---------------------------------------------------------------------------
# Joint angle conventions and safety limits (degrees)
# ---------------------------------------------------------------------------
# Sign convention (positive direction):
#   HY (hip yaw)     -> leg toes outward
#   TP (thigh pitch) -> knee comes up toward chest (hip flexion)
#   KP (knee pitch)  -> heel toward butt (knee flexion)
#   AP (ankle pitch) -> toes up (dorsiflexion)
#
# A SIGN of +1 means the servo's positive direction matches the joint's
# positive direction. -1 inverts (which it often will, depending on how
# you mounted the horn). Calibration will tell you.

JOINT_LIMITS_DEG = {
    "LHY": (-30,  30),
    "LTP": (-45,  60),
    "LKP": (  0,  90),
    "LAP": (-30,  30),
    "RHY": (-30,  30),
    "RTP": (-45,  60),
    "RKP": (  0,  90),
    "RAP": (-30,  30),
}

# Default zero offset (in raw servo units, 0-1000) BEFORE calibration.
# These get overwritten by data/calibration.json once you've calibrated.
DEFAULT_ZERO_RAW = {j: 500 for j in JOINT_NAMES}
DEFAULT_SIGN     = {j: +1  for j in JOINT_NAMES}

# ---------------------------------------------------------------------------
# Disabled / partially broken joints
# ---------------------------------------------------------------------------
# If a servo is unreliable, list it here mapped to the angle (in degrees)
# you want it locked at. Effects:
#   - joints.Legs.set() / set_many() silently drop writes to these joints
#   - joints.Legs.lock_disabled() commands them to the locked angle (call
#     this once after torque-on so the joint actually goes there)
#   - joints.Legs.read_all() returns the locked angle for these joints
#     instead of polling the (possibly unresponsive) servo
#
# Set to {} for normal operation.
DISABLED_JOINTS = {
    "LAP": -15.0,    # left ankle pitch -- flaky; lock near neutral standing angle
}

# LX-16A: 0..1000 raw covers 0..240 degrees, so 1 raw unit ≈ 0.24 degrees
DEG_PER_RAW = 240.0 / 1000.0
RAW_PER_DEG = 1000.0 / 240.0

# Maximum joint speed (deg / sec). Used to clamp commanded move durations.
MAX_JOINT_SPEED_DPS = 180.0

# ---------------------------------------------------------------------------
# Link lengths (mm) -- measured from CAD
# ---------------------------------------------------------------------------
HIP_YAW_TO_HIP_PITCH_MM = 50.7   # hip-yaw axis to thigh-pitch (hip-pitch) axis
LINK_THIGH_MM           = 83.0   # thigh-pitch axis to knee-pitch axis
LINK_SHIN_MM            = 84.0   # knee-pitch axis to ankle-pitch axis
LINK_FOOT_MM            = 63.0   # ankle-pitch axis to ground
HIP_SPACING_MM          = 75.5   # lateral distance between L and R hip-yaw axes
FOOT_LENGTH_MM          = 100.0
FOOT_WIDTH_MM           = 50.0

# Total bot mass (kg). Used for sanity-checking torque margin.
TOTAL_MASS_KG = 8.0

# ---------------------------------------------------------------------------
# IMU orientation
# ---------------------------------------------------------------------------
# Body frame: +X forward, +Y left, +Z up. Set the IMU rotation that maps
# the chip's frame into the body frame. Use one of:
#   "identity"      chip mounted X-fwd, Y-left, Z-up
#   "rot_z_90"      chip rotated 90° CCW from body
#   "rot_z_180"     chip rotated 180°
#   "rot_z_-90"     chip rotated 90° CW
#   "flip_x"        chip upside-down about X
# Add more cases in imu.py:_apply_rotation() if needed.
IMU_ROTATION = "identity"

# Fall detection: trigger if body tilt (pitch or roll) magnitude exceeds this
FALL_TILT_DEG = 35.0

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = REPO_ROOT / "data"
CALIBRATION_FILE = DATA_DIR / "calibration.json"
