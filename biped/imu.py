"""
IMU abstraction. Auto-detects whichever Adafruit/community library is
installed for your sensor and exposes one tiny interface:

    imu = make_imu()
    while True:
        roll, pitch, yaw_rate = imu.update()       # roll/pitch in deg
        ...

`yaw_rate` is degrees/second around the vertical axis. We don't try to
integrate yaw because gyro drift will eat it; for a keyframe walker
yaw_rate is enough.

Roll/pitch are computed with a complementary filter: trust the gyro
short-term, lean on the accelerometer for the long-term gravity
reference.
"""

import math
import time
from . import config


def _apply_rotation(ax, ay, az, gx, gy, gz, rotation: str):
    """Map sensor frame -> body frame (+X fwd, +Y left, +Z up)."""
    if rotation == "identity":
        return ax, ay, az, gx, gy, gz
    if rotation == "rot_z_90":
        return -ay, ax, az, -gy, gx, gz
    if rotation == "rot_z_180":
        return -ax, -ay, az, -gx, -gy, gz
    if rotation == "rot_z_-90":
        return ay, -ax, az, gy, -gx, gz
    if rotation == "flip_x":
        return ax, -ay, -az, gx, -gy, -gz
    raise ValueError(f"Unknown IMU_ROTATION: {rotation}")


class ComplementaryFilter:
    """Simple 1st-order complementary filter for roll & pitch."""

    def __init__(self, alpha: float = 0.98):
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self._last_t = time.time()

    def update(self, ax, ay, az, gx_dps, gy_dps):
        now = time.time()
        dt = now - self._last_t
        self._last_t = now
        if dt <= 0 or dt > 0.5:
            dt = 0.01
        # accel-derived (gravity vector)
        # body: +X fwd, +Y left, +Z up. gravity points -Z when level.
        # pitch = rotation about Y (nose up positive)
        # roll  = rotation about X (right-side down positive ... we'll use sign:
        #         body tilts to its left = positive roll)
        denom_p = math.sqrt(ay * ay + az * az) or 1e-9
        pitch_acc = math.degrees(math.atan2(-ax, denom_p))
        denom_r = math.sqrt(ax * ax + az * az) or 1e-9
        roll_acc  = math.degrees(math.atan2(ay, denom_r))
        # gyro integration
        self.roll  = self.alpha * (self.roll  + gx_dps * dt) + (1 - self.alpha) * roll_acc
        self.pitch = self.alpha * (self.pitch + gy_dps * dt) + (1 - self.alpha) * pitch_acc
        return self.roll, self.pitch


# ---------------------------------------------------------------------------
# Concrete drivers
# ---------------------------------------------------------------------------

class _IMUBase:
    def __init__(self):
        self._filter = ComplementaryFilter()
        self._yaw_rate = 0.0

    def update(self):
        ax, ay, az, gx, gy, gz = self._read_raw()
        ax, ay, az, gx, gy, gz = _apply_rotation(ax, ay, az, gx, gy, gz,
                                                 config.IMU_ROTATION)
        roll, pitch = self._filter.update(ax, ay, az, gx, gy)
        self._yaw_rate = gz
        return roll, pitch, gz

    def _read_raw(self):
        raise NotImplementedError


class _AdafruitIMU(_IMUBase):
    """For LSM6DSOX and ICM-20948 (both expose .acceleration and .gyro)."""
    def __init__(self, sensor):
        super().__init__()
        self._s = sensor

    def _read_raw(self):
        # adafruit returns acceleration in m/s^2 and gyro in rad/s
        ax, ay, az = self._s.acceleration
        gx, gy, gz = self._s.gyro
        # accel doesn't need units conversion for tilt math (we use ratios),
        # but gyro -> deg/s for the complementary filter
        return (ax, ay, az,
                math.degrees(gx), math.degrees(gy), math.degrees(gz))


class _MPU6050IMU(_IMUBase):
    """For mpu6050-raspberrypi library."""
    def __init__(self, sensor):
        super().__init__()
        self._s = sensor

    def _read_raw(self):
        a = self._s.get_accel_data()  # m/s^2 dict
        g = self._s.get_gyro_data()   # deg/s dict
        return (a["x"], a["y"], a["z"], g["x"], g["y"], g["z"])


def make_imu():
    """Try each known library in order. First one that imports + initializes wins."""
    errors = []

    # 1) LSM6DSOX
    try:
        import board, busio
        from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = LSM6DSOX(i2c)
        print("[imu] using Adafruit LSM6DSOX")
        return _AdafruitIMU(sensor)
    except Exception as e:
        errors.append(f"LSM6DSOX: {e}")

    # 2) ICM-20948
    try:
        import board, busio
        from adafruit_icm20x import ICM20948
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = ICM20948(i2c)
        print("[imu] using Adafruit ICM-20948")
        return _AdafruitIMU(sensor)
    except Exception as e:
        errors.append(f"ICM20948: {e}")

    # 3) MPU6050
    try:
        from mpu6050 import mpu6050
        sensor = mpu6050(0x68)
        print("[imu] using MPU6050 (community lib)")
        return _MPU6050IMU(sensor)
    except Exception as e:
        errors.append(f"MPU6050: {e}")

    raise RuntimeError("No IMU library worked. Tried:\n  " + "\n  ".join(errors))
