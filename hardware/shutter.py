"""
hardware/shutter.py — shutter abstractions.

RetroShutter — controls a shutter via a DAQC2 DOUT bit (e.g. Thorlabs SC10
               TTL input).
Shutter      — controls a servo via an Arduino Leonardo running
               arduino/servo_driver/servo_driver.ino.  Communicates over
               USB serial.  The Arduino generates jitter-free 50 Hz PWM;
               the servo is detached (PWM stopped) after each move.
"""

import logging
import serial
import time

log = logging.getLogger(__name__)


class RetroShutter:
    """Controls a pulse-triggered shutter via a DAQC2 DOUT bit.

    The shutter changes state on each TTL pulse; open/closed state is not
    tracked since there is no feedback.

    Args:
        board:      DAQC2Board instance.
        bit:        DOUT bit number (0–7).
        pulse_s:    Pulse duration in seconds (default 20 ms).
    """

    def __init__(self, board, bit: int, pulse_s: float = 0.02):
        self.board = board
        self.bit = bit
        self.pulse_s = pulse_s
        # ensure pin starts low
        self.board.set_dout(bit, state=False)

    def trigger(self):
        """Send a TTL pulse to toggle the shutter state."""
        self.board.set_dout(self.bit, state=True)
        time.sleep(self.pulse_s)
        self.board.set_dout(self.bit, state=False)
        log.info('RetroShutter (bit %d) triggered', self.bit)


class Shutter:
    """Controls a servo-based shutter via an Arduino Leonardo running servo_driver.ino.

    Sends pulse-width commands over USB serial.  The Arduino generates
    jitter-free 50 Hz PWM and detaches the servo (stops PWM) on request,
    eliminating idle jitter.

    Args:
        port:         Serial port, e.g. '/dev/ttyACM1'.
        open_pw_us:   Pulse width in µs for the open position.
        close_pw_us:  Pulse width in µs for the closed position.
        baud:         Must match sketch (default 115200).
        timeout:      Serial read timeout in seconds.
    """

    def __init__(self, port: str, open_pw_us: int = 2000,
                 close_pw_us: int = 1000, baud: int = 115200,
                 timeout: float = 2.0):
        self.open_pw_us = open_pw_us
        self.close_pw_us = close_pw_us
        self._open = False
        self._ser = serial.Serial(port, baud, timeout=timeout)
        # wait for Arduino to reset and send READY; an empty response just means
        # the sketch was already running and didn't reset on reconnect
        ready = self._ser.readline().decode().strip()
        if ready == 'READY' or ready == '':
            pass
        else:
            log.warning('Shutter: expected READY, got %r', ready)
        log.info('Shutter on %s initialised', port)

    def _send(self, cmd: str) -> bool:
        self._ser.write((cmd + '\n').encode())
        resp = self._ser.readline().decode().strip()
        if resp != 'OK':
            log.error('Shutter: unexpected response %r to %r', resp, cmd)
            return False
        return True

    def set_position(self, pulse_us: int):
        """Command the servo to a position by pulse width in microseconds."""
        self._send(str(pulse_us))
        log.info('Shutter → %d µs', pulse_us)

    def detach(self):
        """Stop PWM output (eliminates idle jitter)."""
        self._send('DETACH')
        log.info('Shutter detached')

    def open(self):
        self.set_position(self.open_pw_us)
        self._open = True

    def close(self):
        self.set_position(self.close_pw_us)
        self._open = False

    def shutdown(self):
        """Detach servo and close serial port."""
        try:
            self.detach()
        except Exception:
            pass
        self._ser.close()

    @property
    def is_open(self) -> bool:
        return self._open
