"""
hardware/mirror.py — steering mirror axis controllers.

TicAxis    — azimuth axis via Pololu Tic T834 USB stepper driver (ticlib).
KDC101Axis — altitude axis via Thorlabs KDC101 DC servo controller (APT/serial).
"""

import logging
import struct
import time
import serial

log = logging.getLogger(__name__)


# ── Tic T834 azimuth axis ─────────────────────────────────────────────────────

class TicAxis:
    """Azimuth axis via Pololu Tic T834 over USB.

    ticlib is imported lazily so a missing installation only raises on
    instantiation, not on module import.

    Args:
        step_size:  Microsteps per keypress.
    """

    def __init__(self, step_size: int = 50, serial_number: str | None = None):
        from ticlib import TicUSB          # lazy — ticlib may not be installed
        self._tic = TicUSB(serial_number=serial_number)
        self._step_size = max(1, int(step_size))
        self._tic.energize()
        self._tic.exit_safe_start()
        log.info('TicAxis initialised, serial=%s step_size=%d',
                 serial_number or 'auto', self._step_size)

    def step(self, direction: int) -> None:
        """Relative move. direction: +1 (positive) or -1 (negative)."""
        pos    = self._tic.get_current_position()
        target = pos + direction * self._step_size
        self._tic.set_target_position(target)
        log.info('TicAxis %+d steps → %d', direction * self._step_size, target)

    @property
    def step_size(self) -> int:
        return self._step_size

    @step_size.setter
    def step_size(self, value: int) -> None:
        self._step_size = max(1, int(value))

    def shutdown(self) -> None:
        try:
            self._tic.deenergize()
        except Exception:
            pass


# ── KDC101 altitude axis (minimal APT protocol over serial) ──────────────────

_APT_HOST   = 0x01
_APT_DEVICE = 0x50   # generic USB unit address used by KDC101


def _apt_short(msg_id: int, p1: int = 0, p2: int = 0) -> bytes:
    """Build a 6-byte short APT message (no data payload)."""
    return struct.pack('<HBBBB', msg_id, p1, p2, _APT_DEVICE, _APT_HOST)


def _apt_move_relative(channel: int, counts: int) -> bytes:
    """Build MGMSG_MOT_MOVE_RELATIVE (0x0448) — relative move by encoder counts."""
    data   = struct.pack('<Hi', channel, counts)          # uint16 chan, int32 dist
    header = struct.pack('<HHBB', 0x0448, len(data),
                         _APT_DEVICE | 0x80, _APT_HOST)  # long msg: dest|0x80
    return header + data


class KDC101Axis:
    """Altitude axis via Thorlabs KDC101 over USB serial (APT protocol).

    The KDC101 presents as a virtual COM port.  Only the MOVE_RELATIVE command
    is used; no position readback is implemented.

    Args:
        port:       Serial port, e.g. '/dev/ttyUSB3'.
        step_size:  Encoder counts per keypress.
        channel:    Motor channel (always 1 for KDC101).
        baud:       APT default 115200.
        timeout:    Serial read timeout in seconds.
    """

    def __init__(self, port: str, step_size: int = 100,
                 channel: int = 1, baud: int = 115200, timeout: float = 1.0):
        self._ser      = serial.Serial(port, baud, timeout=timeout)
        self._channel  = channel
        self._step_size = max(1, int(step_size))
        time.sleep(0.1)
        self._ser.reset_input_buffer()
        # Enable the motor channel
        self._ser.write(_apt_short(0x0210, channel, 0x01))
        log.info('KDC101Axis on %s initialised, step_size=%d', port, self._step_size)

    def step(self, direction: int) -> None:
        """Relative move. direction: +1 (positive) or -1 (negative)."""
        counts = direction * self._step_size
        self._ser.write(_apt_move_relative(self._channel, counts))
        log.info('KDC101Axis %+d counts', counts)

    @property
    def step_size(self) -> int:
        return self._step_size

    @step_size.setter
    def step_size(self, value: int) -> None:
        self._step_size = max(1, int(value))

    def shutdown(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass
