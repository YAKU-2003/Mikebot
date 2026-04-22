"""
High-level joint control. Hides the LX-16A raw-units / per-servo-mounting
mess behind a simple API:

    legs = Legs()
    legs.torque(True)
    legs.set("LKP", 30.0)                         # set left knee to 30 deg
    legs.set_many({"LTP": 20, "LKP": 40,          # synchronized move
                   "RTP": 20, "RKP": 40},
                  duration_ms=400)
    pos = legs.read("LKP")                        # current angle in deg
    legs.torque(False)                            # E-stop

Internally:
    raw = ZERO + SIGN * RAW_PER_DEG * deg
    deg = SIGN * (raw - ZERO) * DEG_PER_RAW
"""

import json
import time
from pathlib import Path

from . import config
from .lx16a import LX16ABus, BROADCAST_ID


class Legs:
    def __init__(self, bus: LX16ABus = None):
        self.bus = bus or LX16ABus()
        self.zero_raw = dict(config.DEFAULT_ZERO_RAW)
        self.sign     = dict(config.DEFAULT_SIGN)
        # joint -> locked angle in degrees. Writes to these joints are dropped
        # by set()/set_many(). Use lock_disabled() to actually command them.
        self.disabled = dict(config.DISABLED_JOINTS)
        if self.disabled:
            print(f"[joints] disabled joints (locked): "
                  f"{ {k: f'{v:+.1f} deg' for k, v in self.disabled.items()} }")
        self._load_calibration()

    # ------------------------------------------------------------------
    # Calibration I/O
    # ------------------------------------------------------------------
    def _load_calibration(self):
        path: Path = config.CALIBRATION_FILE
        if not path.exists():
            print(f"[joints] WARNING: no calibration at {path}. Using defaults.")
            return
        with open(path) as f:
            data = json.load(f)
        for j in config.JOINT_NAMES:
            if j in data:
                self.zero_raw[j] = int(data[j]["zero_raw"])
                self.sign[j]     = int(data[j]["sign"])
        print(f"[joints] loaded calibration for {len(data)} joints")

    def save_calibration(self):
        path: Path = config.CALIBRATION_FILE
        path.parent.mkdir(parents=True, exist_ok=True)
        out = {j: {"zero_raw": self.zero_raw[j], "sign": self.sign[j]}
               for j in config.JOINT_NAMES}
        with open(path, "w") as f:
            json.dump(out, f, indent=2)
        print(f"[joints] saved calibration to {path}")

    # ------------------------------------------------------------------
    # Conversions
    # ------------------------------------------------------------------
    def deg_to_raw(self, joint: str, deg: float) -> int:
        # Clamp to soft joint limits first
        lo, hi = config.JOINT_LIMITS_DEG[joint]
        deg_clamped = max(lo, min(hi, float(deg)))
        raw = self.zero_raw[joint] + self.sign[joint] * config.RAW_PER_DEG * deg_clamped
        return int(round(max(0, min(1000, raw))))

    def raw_to_deg(self, joint: str, raw: int) -> float:
        return self.sign[joint] * (raw - self.zero_raw[joint]) * config.DEG_PER_RAW

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------
    def torque(self, on: bool):
        """Bulk torque on/off across all joints (broadcast)."""
        self.bus.torque(BROADCAST_ID, on=on)

    def set(self, joint: str, deg: float, duration_ms: int = 200):
        """Move one joint to `deg` over `duration_ms`.

        Disabled joints (config.DISABLED_JOINTS) are silently skipped --
        they're held at their locked angle by lock_disabled() instead.
        """
        if joint in self.disabled:
            return
        sid = config.SERVO_IDS[joint]
        raw = self.deg_to_raw(joint, deg)
        self.bus.move(sid, raw, time_ms=duration_ms)

    def lock_disabled(self, duration_ms: int = 600):
        """Command every disabled joint to its locked angle (bypasses the
        set() filter). Call this once after you've torqued the bus on so
        the locked joints actually move into position.
        """
        for joint, locked_deg in self.disabled.items():
            sid = config.SERVO_IDS[joint]
            raw = self.deg_to_raw(joint, locked_deg)
            self.bus.move(sid, raw, time_ms=duration_ms)

    def set_many(self, angles_deg: dict, duration_ms: int = 300):
        """
        Move multiple joints simultaneously. The LX-16A protocol has no
        true sync-write, so we send commands back-to-back. With 8 servos
        at 115200 baud the command bursts in ~5 ms which is well below
        any reasonable motion duration.
        """
        # Auto-clamp duration so we don't ask for impossibly fast moves
        duration_ms = max(1, int(duration_ms))
        for j, d in angles_deg.items():
            self.set(j, d, duration_ms=duration_ms)

    def hold(self, angles_deg: dict, duration_ms: int = 300):
        """Send a target pose and block for duration_ms while it executes."""
        self.set_many(angles_deg, duration_ms=duration_ms)
        time.sleep(duration_ms / 1000.0)

    # ------------------------------------------------------------------
    # Sensing
    # ------------------------------------------------------------------
    def read(self, joint: str) -> float:
        # For disabled joints, return the locked-angle assumption rather
        # than polling a (possibly unresponsive) servo.
        if joint in self.disabled:
            return self.disabled[joint]
        sid = config.SERVO_IDS[joint]
        raw = self.bus.read_position(sid)
        return self.raw_to_deg(joint, raw)

    def read_all(self) -> dict:
        out = {}
        for j in config.JOINT_NAMES:
            try:
                out[j] = self.read(j)
            except Exception as e:
                # Be resilient -- one timed-out servo shouldn't kill the loop.
                # Fall back to whatever we last commanded (assume zero if unknown).
                print(f"[joints] read({j}) failed: {e}; assuming 0")
                out[j] = 0.0
        return out
