"""
Safety / E-stop helpers.

    safety = SafetyMonitor(imu, legs)
    safety.start()           # spawns a background thread
    ...
    safety.stop()
"""

import threading
import time
from . import config


class SafetyMonitor:
    """
    Background thread that polls the IMU and triggers torque-off if the
    body tilts past FALL_TILT_DEG (config). Also listens for an external
    `tripped` flag so you can wire it into your main loop.
    """

    def __init__(self, imu, legs, tilt_limit_deg: float = None,
                 poll_hz: float = 50.0):
        self.imu = imu
        self.legs = legs
        self.tilt_limit = tilt_limit_deg if tilt_limit_deg is not None \
                                          else config.FALL_TILT_DEG
        self.period = 1.0 / poll_hz
        self.tripped = False
        self._thread = None
        self._stop = threading.Event()

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self.tripped = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _loop(self):
        while not self._stop.is_set():
            try:
                roll, pitch, _ = self.imu.update()
                if abs(roll) > self.tilt_limit or abs(pitch) > self.tilt_limit:
                    print(f"[safety] FALL DETECTED roll={roll:.1f} pitch={pitch:.1f}"
                          f" -- torque off")
                    self.tripped = True
                    try:
                        self.legs.torque(False)
                    except Exception as e:
                        print(f"[safety] torque-off failed: {e}")
                    return
            except Exception as e:
                print(f"[safety] IMU read error: {e}")
            time.sleep(self.period)
