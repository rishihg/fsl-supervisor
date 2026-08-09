"""
hardware/mdt693b.py — Thorlabs MDT693B 3-axis open-loop piezo controller.

Communicates over USB serial (/dev/ttyACM0) at 115200 baud.
The mdt69x library's _check_port method does not recognise ttyACM devices,
so we bypass it and open the port directly.
"""

import logging
import serial
import time

log = logging.getLogger(__name__)

# Voltage range for this unit (set by rear-panel switch)
VOLTAGE_MIN = 0.0
VOLTAGE_MAX = 150.0


class MDT693B:
    """Minimal driver for the Thorlabs MDT693B piezo controller.

    Only X and Y axes are used for the HST piezo steering mirror.
    Z is available but not wired to anything.

    Args:
        port:       Serial port, e.g. '/dev/ttyACM0'.
        step_v:     Voltage step per keypress (volts).
        timeout:    Serial read/write timeout in seconds.
    """

    def __init__(self, port: str = '/dev/ttyACM0',
                 step_v: float = 1.0, timeout: float = 1.0):
        self._ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=timeout,
        )
        # DTR/RTS low — required by MDT693B
        self._ser.setDTR(False)
        self._ser.setRTS(False)
        time.sleep(0.1)
        self._ser.flushInput()
        # Silence echo and set compatibility mode
        self._send('echo=0')
        self._send('cm=0')
        self._step_v = float(step_v)
        log.info('MDT693B on %s initialised, step=%.2f V', port, self._step_v)

    # ── low-level serial ──────────────────────────────────────────────────────

    def _send(self, cmd: str) -> str:
        """Send a command and return the response string (stripped)."""
        self._ser.write((cmd + '\r').encode('utf-8'))
        time.sleep(0.05)
        raw = self._ser.read(256)
        return raw.decode('utf-8', errors='replace').strip()

    # ── voltage get/set ───────────────────────────────────────────────────────

    def get_voltage(self, axis: str) -> float:
        """Read voltage on axis ('x', 'y', or 'z'). Returns float volts."""
        resp = self._send(f'{axis.lower()}voltage?')
        # Response is e.g. '[37.50]' or '37.50'
        resp = resp.strip('[]> \r\n')
        try:
            return float(resp)
        except ValueError:
            if resp:
                log.warning('MDT693B: unexpected voltage response %r', resp)
                return 0.0

    def set_voltage(self, axis: str, voltage: float) -> None:
        """Set voltage on axis ('x', 'y', or 'z') in volts."""
        voltage = max(VOLTAGE_MIN, min(VOLTAGE_MAX, float(voltage)))
        self._send(f'{axis.lower()}voltage={voltage:.3f}')
        log.info('MDT693B %s → %.3f V', axis.upper(), voltage)

    # ── step interface (matches TicAxis.step() pattern) ───────────────────────

    def step(self, axis: str, direction: int) -> None:
        """Step axis by step_v volts in direction (+1 or -1)."""
        current = self.get_voltage(axis)
        target = current + direction * self._step_v
        self.set_voltage(axis, target)

    @property
    def step_v(self) -> float:
        return self._step_v

    @step_v.setter
    def step_v(self, value: float) -> None:
        self._step_v = max(0.1, float(value))

    # ── convenience ───────────────────────────────────────────────────────────

    def get_all(self) -> dict[str, float]:
        """Return {'x': v, 'y': v, 'z': v}."""
        return {ax: self.get_voltage(ax) for ax in ('x', 'y', 'z')}

    def center(self, axis: str) -> None:
        """Set axis to mid-range (75 V for 0-150 V unit)."""
        mid = (VOLTAGE_MIN + VOLTAGE_MAX) / 2
        self.set_voltage(axis, mid)
        log.info('MDT693B %s → center (%.1f V)', axis.upper(), mid)

    def shutdown(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass
