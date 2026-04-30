"""
hardware/laser.py — laser control.

Laser        — legacy analog control via DAQC2 DAC (0–4.095 V modulation input).
MCLS1        — Thorlabs MCLS1 4-channel fiber-coupled laser source over USB serial.
               Communicates via ASCII commands; controls one channel by power (mW).
MCLS1Channel — Additional channel on an existing MCLS1, sharing its serial port.
"""

import logging
import threading
import time
import serial
from .daqc2 import DAQC2Board

log = logging.getLogger(__name__)

DAC_MAX_V = 4.095   # DAQC2 DAC ceiling


class Laser:
    """Controls laser intensity via a single DAQC2 DAC channel (0–4.095 V)."""

    def __init__(self, board: DAQC2Board, dac_channel: int = 0):
        self.board = board
        self.dac_channel = dac_channel
        self._voltage = 0.0
        self.off()

    def set_voltage(self, voltage: float):
        voltage = max(0.0, min(DAC_MAX_V, voltage))
        self.board.set_dac(self.dac_channel, voltage)
        self._voltage = voltage
        log.info('Laser set to %.3f V', voltage)

    def off(self):
        self.set_voltage(0.0)

    @property
    def voltage(self) -> float:
        return self._voltage


class MCLS1:
    """Thorlabs MCLS1 4-channel fiber-coupled laser source over USB serial.

    Sends ASCII commands over USB CDC (virtual COM port, 115200 8N1).
    Controls a single channel by optical power setpoint (mW).
    Calling set_power() also enables the channel; off()/disable() disables it
    without changing the setpoint so enable() can restore it.

    Args:
        port:         Serial port, e.g. '/dev/ttyUSB2'.
        channel:      Laser channel to control (1–4).
        baud:         Must match device default (115200).
        timeout:      Serial read timeout in seconds.
        current_cal:  Optional (min_ma, max_ma, min_mw, max_mw) calibration
                      tuple.  When provided, set_power() converts mW→mA and
                      drives the laser via ``current=`` rather than ``power=``.
                      Use this when the unit ignores ``power=`` and is
                      current-controlled from the front panel.
    """

    def __init__(self, port: str, channel: int = 1,
                 baud: int = 115200, timeout: float = 1.0,
                 current_cal: tuple[float, float, float, float] | None = None):
        self._ser          = serial.Serial(port, baud, timeout=timeout)
        self._lock         = threading.Lock()
        self._channel      = channel
        self._power_mw     = 0.0
        self._enabled      = False
        self._current_cal  = current_cal   # (min_ma, max_ma, min_mw, max_mw)
        # flush any startup banner, then select channel and ensure output off
        time.sleep(0.2)
        self._ser.reset_input_buffer()
        self._cmd(f'channel={channel}')
        self.disable()
        log.info('MCLS1 on %s ch%d initialised', port, channel)

    # ── serial protocol ───────────────────────────────────────────────────────

    def _cmd(self, cmd: str) -> str:
        """Send a command; return the response value (stripped of echo/prompt)."""
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write((cmd + '\r').encode())
            # Device responds with: echo \r\n [value \r\n] >
            raw   = self._ser.read_until(b'> ')
            text  = raw.decode(errors='replace').replace('\r', '\n')
            lines = [l.strip() for l in text.split('\n')
                     if l.strip() and l.strip() not in ('>', cmd)]
            resp = lines[0] if lines else ''
            if resp.upper().startswith('CMD_NOT_DEFINED'):
                log.warning('MCLS1: unrecognised command %r', cmd)
            return resp

    # ── public interface ──────────────────────────────────────────────────────

    def set_power(self, power_mw: float):
        """Set channel power (mW) and enable output.

        If a current_cal was supplied at construction the command is translated
        to a ``current=`` setpoint via linear interpolation; otherwise the
        standard ``power=`` command is used.
        """
        power_mw = max(0.0, power_mw)
        if self._current_cal:
            min_ma, max_ma, min_mw, max_mw = self._current_cal
            if power_mw == 0.0:
                self.disable()
                self._power_mw = 0.0
                return
            t  = (power_mw - min_mw) / (max_mw - min_mw)
            ma = min_ma + t * (max_ma - min_ma)
            ma = min(max_ma, ma)
            self.set_current_ma(ma)
            self._power_mw = power_mw   # store requested value, not back-calc
        else:
            self._cmd(f'channel={self._channel}')
            self._cmd(f'power={power_mw:.3f}')
            self._power_mw = power_mw
            self._cmd('enable=1')
            self._enabled = True
            log.info('MCLS1 ch%d → %.3f mW (enabled)', self._channel, power_mw)

    def set_current_ma(self, ma: float):
        """Set channel drive current (mA) directly and enable output.

        Use when the unit is current-controlled (front-panel knob) and the
        ``power=`` command has no effect.  Back-calculates displayed power_mw
        from current_cal if available.
        """
        self._cmd(f'channel={self._channel}')
        self._cmd(f'current={ma:.1f}')
        self._cmd('enable=1')
        self._enabled = True
        if self._current_cal:
            min_ma, max_ma, min_mw, max_mw = self._current_cal
            t = (ma - min_ma) / (max_ma - min_ma)
            self._power_mw = min_mw + t * (max_mw - min_mw)
        log.info('MCLS1 ch%d → %.1f mA (enabled, ~%.3f mW)',
                 self._channel, ma, self._power_mw)

    def enable(self):
        """Enable output at the current power setpoint."""
        self._cmd(f'channel={self._channel}')
        self._cmd('enable=1')
        self._enabled = True
        log.info('MCLS1 ch%d enabled (%.3f mW)', self._channel, self._power_mw)

    def disable(self):
        """Disable channel output (power setpoint unchanged)."""
        self._cmd(f'channel={self._channel}')
        self._cmd('enable=0')
        self._enabled = False
        log.info('MCLS1 ch%d disabled', self._channel)

    def off(self):
        """Alias for disable()."""
        self.disable()

    @property
    def power_mw(self) -> float:
        return self._power_mw

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    def shutdown(self):
        """Disable channel and close serial port."""
        try:
            self.disable()
        except Exception:
            pass
        self._ser.close()


class MCLS1Channel:
    """An additional channel on an existing MCLS1, sharing its serial connection.

    Instantiate after the parent MCLS1.  All serial access goes through the
    parent's _cmd(), which holds a lock, so concurrent use from separate
    threads is safe.  shutdown() only disables the channel output; it does not
    close the serial port (the parent owns that).

    Args:
        parent:       The MCLS1 instance that owns the serial connection.
        channel:      Channel number to control (1–4).
        current_cal:  Same semantics as MCLS1 current_cal.
    """

    def __init__(self, parent: 'MCLS1', channel: int,
                 current_cal: tuple[float, float, float, float] | None = None):
        self._parent      = parent
        self._channel     = channel
        self._power_mw    = 0.0
        self._enabled     = False
        self._current_cal = current_cal
        self._cmd(f'channel={channel}')
        self._cmd('enable=0')
        log.info('MCLS1Channel ch%d initialised', channel)

    def _cmd(self, cmd: str) -> str:
        return self._parent._cmd(cmd)

    def set_power(self, power_mw: float):
        power_mw = max(0.0, power_mw)
        if self._current_cal:
            min_ma, max_ma, min_mw, max_mw = self._current_cal
            if power_mw == 0.0:
                self.disable()
                self._power_mw = 0.0
                return
            t  = (power_mw - min_mw) / (max_mw - min_mw)
            ma = min(max_ma, min_ma + t * (max_ma - min_ma))
            self.set_current_ma(ma)
            self._power_mw = power_mw
        else:
            self._cmd(f'channel={self._channel}')
            self._cmd(f'power={power_mw:.3f}')
            self._power_mw = power_mw
            self._cmd('enable=1')
            self._enabled = True
            log.info('MCLS1Channel ch%d → %.3f mW (enabled)', self._channel, power_mw)

    def set_current_ma(self, ma: float):
        self._cmd(f'channel={self._channel}')
        self._cmd(f'current={ma:.1f}')
        self._cmd('enable=1')
        self._enabled = True
        if self._current_cal:
            min_ma, max_ma, min_mw, max_mw = self._current_cal
            t = (ma - min_ma) / (max_ma - min_ma)
            self._power_mw = min_mw + t * (max_mw - min_mw)
        log.info('MCLS1Channel ch%d → %.1f mA (enabled, ~%.3f mW)',
                 self._channel, ma, self._power_mw)

    def enable(self):
        self._cmd(f'channel={self._channel}')
        self._cmd('enable=1')
        self._enabled = True
        log.info('MCLS1Channel ch%d enabled (%.3f mW)', self._channel, self._power_mw)

    def disable(self):
        self._cmd(f'channel={self._channel}')
        self._cmd('enable=0')
        self._enabled = False
        log.info('MCLS1Channel ch%d disabled', self._channel)

    def off(self):
        self.disable()

    @property
    def power_mw(self) -> float:
        return self._power_mw

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    def shutdown(self):
        try:
            self.disable()
        except Exception:
            pass
