"""
Minimal LX-16A bus servo driver (LewanSoul / Hiwonder protocol).

Frame format:
    0x55 0x55 ID LEN CMD [PARAMS...] CHECKSUM

Where:
    ID        target servo (0xFE = broadcast, no reply)
    LEN       3 + number_of_params
    CMD       command byte (see CMD_* constants)
    CHECKSUM  ~(ID + LEN + CMD + sum(PARAMS)) & 0xFF

Position units: 0..1000 raw  ==  0..240 degrees  (1 raw ≈ 0.24°)
Time units: milliseconds, 0..30000.
"""

import struct
import time
import threading
import serial

from . import config

# Commands we use
CMD_MOVE_TIME_WRITE       = 1
CMD_POS_READ              = 28
CMD_LOAD_OR_UNLOAD_WRITE  = 31
CMD_LOAD_OR_UNLOAD_READ   = 32
CMD_ID_WRITE              = 13
CMD_ID_READ               = 14
CMD_ANGLE_OFFSET_ADJUST   = 17  # temporary, RAM only
CMD_ANGLE_OFFSET_WRITE    = 18  # commit RAM offset to EEPROM
CMD_ANGLE_LIMIT_WRITE     = 20
CMD_TEMP_READ             = 26
CMD_VIN_READ              = 27

BROADCAST_ID = 0xFE
HEADER = b"\x55\x55"


def _checksum(buf: bytes) -> int:
    """Standard LX-16A checksum: bitwise NOT of byte sum, ignoring header."""
    s = 0
    for b in buf:
        s += b
    return (~s) & 0xFF


def _build_frame(servo_id: int, cmd: int, params: bytes = b"") -> bytes:
    length = 3 + len(params)
    body = bytes([servo_id, length, cmd]) + params
    return HEADER + body + bytes([_checksum(body)])


class LX16ABus:
    """
    Thread-safe wrapper around the half-duplex LX-16A serial bus.

    All public methods take a `servo_id` (1..253) or use `BROADCAST_ID`.
    """

    def __init__(self, port: str = None, baud: int = None, timeout: float = None):
        port = port or config.SERIAL_PORT
        baud = baud or config.BAUD_RATE
        timeout = timeout if timeout is not None else config.SERIAL_TIMEOUT_S
        self._ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self._lock = threading.Lock()

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Low-level send / receive
    # ------------------------------------------------------------------
    def _send(self, frame: bytes):
        with self._lock:
            # On a hand-rolled half-duplex circuit you'll receive your own
            # echo. Flush input before reading reply.
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            self._ser.flush()

    def _send_recv(self, frame: bytes, expected_param_len: int) -> bytes:
        """Send a command that expects a reply. Returns the param bytes only."""
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            self._ser.flush()
            # Read echo (some setups echo TX onto RX) plus reply.
            # We just keep reading until we find a valid reply frame or time out.
            deadline = time.time() + max(0.05, self._ser.timeout * 4)
            buf = bytearray()
            while time.time() < deadline:
                chunk = self._ser.read(64)
                if chunk:
                    buf.extend(chunk)
                    reply = self._extract_reply(buf, expected_param_len)
                    if reply is not None:
                        return reply
                else:
                    # short sleep to avoid spin
                    time.sleep(0.001)
            raise TimeoutError("LX-16A: no reply (check wiring, ID, baud)")

    @staticmethod
    def _extract_reply(buf: bytearray, expected_param_len: int):
        """Scan a buffer for a valid LX-16A frame and return its params.

        Frame layout (indices relative to start of frame `i`):
            i+0,i+1  header 0x55 0x55
            i+2      ID
            i+3      LEN   (counts from itself through CRC inclusive,
                            i.e. LEN = 3 + N where N = number of params)
            i+4      CMD
            i+5..    params (N bytes)
            i+5+N    CRC                <- last byte
        Frame end (exclusive) = i + LEN + 3.
        """
        i = 0
        last_params = None
        while i + 6 <= len(buf):
            if buf[i] == 0x55 and buf[i + 1] == 0x55:
                if i + 4 > len(buf):
                    break
                length = buf[i + 3]
                frame_end = i + length + 3  # exclusive end index
                if frame_end > len(buf):
                    break
                body = buf[i + 2 : frame_end - 1]   # ID, LEN, CMD, params
                crc = buf[frame_end - 1]
                if crc == _checksum(bytes(body)):
                    params = bytes(body[3:])        # skip ID, LEN, CMD
                    if expected_param_len is None or len(params) == expected_param_len:
                        last_params = params
                    i = frame_end
                    continue
            i += 1
        return last_params

    # ------------------------------------------------------------------
    # Convenience commands
    # ------------------------------------------------------------------
    def move(self, servo_id: int, position_raw: int, time_ms: int = 0):
        """
        Command a servo to position_raw (0..1000) reaching there in time_ms.
        time_ms = 0 means as fast as the servo can.
        """
        position_raw = max(0, min(1000, int(position_raw)))
        time_ms = max(0, min(30000, int(time_ms)))
        params = struct.pack("<HH", position_raw, time_ms)
        self._send(_build_frame(servo_id, CMD_MOVE_TIME_WRITE, params))

    def read_position(self, servo_id: int) -> int:
        """Return the servo's current position in raw 0..1000 units."""
        frame = _build_frame(servo_id, CMD_POS_READ)
        params = self._send_recv(frame, expected_param_len=2)
        (pos,) = struct.unpack("<h", params)  # signed!
        return pos

    def torque(self, servo_id: int, on: bool):
        """Enable (on=True) or disable (on=False) the servo's holding torque."""
        self._send(_build_frame(servo_id, CMD_LOAD_OR_UNLOAD_WRITE,
                                bytes([1 if on else 0])))

    def is_torqued(self, servo_id: int) -> bool:
        params = self._send_recv(_build_frame(servo_id, CMD_LOAD_OR_UNLOAD_READ),
                                 expected_param_len=1)
        return params[0] == 1

    def set_id(self, current_id: int, new_id: int):
        """Change a servo's ID. ONLY ONE SERVO ON THE BUS at a time."""
        if not (0 < new_id < 254):
            raise ValueError("ID must be 1..253")
        self._send(_build_frame(current_id, CMD_ID_WRITE, bytes([new_id])))

    def read_id(self, servo_id: int = BROADCAST_ID) -> int:
        params = self._send_recv(_build_frame(servo_id, CMD_ID_READ),
                                 expected_param_len=1)
        return params[0]

    def set_angle_offset(self, servo_id: int, offset_raw: int, persist: bool = False):
        """
        Set a small angle offset (signed -125..125 raw units, ~±30°).
        If persist=True, write to EEPROM (survives power cycle).
        """
        offset_raw = max(-125, min(125, int(offset_raw)))
        # signed byte
        b = offset_raw if offset_raw >= 0 else (256 + offset_raw)
        self._send(_build_frame(servo_id, CMD_ANGLE_OFFSET_ADJUST, bytes([b])))
        if persist:
            time.sleep(0.01)
            self._send(_build_frame(servo_id, CMD_ANGLE_OFFSET_WRITE))

    def set_angle_limits(self, servo_id: int, min_raw: int, max_raw: int):
        """Hard-clamp the servo's range in EEPROM."""
        min_raw = max(0, min(1000, int(min_raw)))
        max_raw = max(0, min(1000, int(max_raw)))
        if min_raw > max_raw:
            min_raw, max_raw = max_raw, min_raw
        params = struct.pack("<HH", min_raw, max_raw)
        self._send(_build_frame(servo_id, CMD_ANGLE_LIMIT_WRITE, params))

    def read_voltage_mv(self, servo_id: int) -> int:
        params = self._send_recv(_build_frame(servo_id, CMD_VIN_READ),
                                 expected_param_len=2)
        (mv,) = struct.unpack("<H", params)
        return mv

    def read_temp_c(self, servo_id: int) -> int:
        params = self._send_recv(_build_frame(servo_id, CMD_TEMP_READ),
                                 expected_param_len=1)
        return params[0]

    # ------------------------------------------------------------------
    # Bus-wide
    # ------------------------------------------------------------------
    def torque_all_off(self):
        """Broadcast torque-off. Servos go limp."""
        self.torque(BROADCAST_ID, on=False)

    def ping(self, max_id: int = 32) -> list:
        """Return the list of servo IDs that respond."""
        found = []
        for i in range(1, max_id + 1):
            try:
                _ = self.read_position(i)
                found.append(i)
            except TimeoutError:
                pass
        return found
