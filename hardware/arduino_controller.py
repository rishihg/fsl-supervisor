"""
hardware/arduino_controller.py — Driver for the HST Arduino Leonardo controller.

Controls the S1FC635 laser interlock and primary mirror shutter
via relay outputs on the Arduino.

The Arduino appears as /dev/ttyACM1 (or ttyACM0 if MDT693B is absent).
Check with: ls /dev/ttyACM*
"""

import logging
import serial
import time

log = logging.getLogger(__name__)


class ArduinoController:
    """Driver for the HST Arduino Leonardo auxiliary controller.

    Args:
        port:    Serial port, e.g. '/dev/ttyACM1'.
        timeout: Serial read/write timeout in seconds.
    """

    def __init__(self, port: str = '/dev/ttyACM1', timeout: float = 2.0):
        self._ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=timeout,
            write_timeout=timeout,
        )
        # Leonardo resets on serial open, wait for it to boot
        time.sleep(2.0)
        self._ser.flushInput()
        # Read the ready message
        ready = self._ser.readline().decode('utf-8', errors='replace').strip()
        if 'ready' not in ready.lower():
            log.warning('Arduino: unexpected greeting: %r', ready)
        # Sync state from hardware
        self._laser_state   = False
        self._shutter_state = False
        self._adr_state     = False
        self._sync()
        log.info('ArduinoController on %s initialised — laser=%s shutter=%s adr=%s',
                 port, self._laser_state, self._shutter_state, self._adr_state)

    def _send(self, cmd: str) -> str:
        self._ser.write((cmd + '\n').encode('utf-8'))
        resp = self._ser.readline().decode('utf-8', errors='replace').strip()
        return resp

    def _sync(self) -> None:
        resp = self._send('?')
        # Expected: "L0 S1" etc.
        try:
            parts = resp.split()
            for p in parts:
                if p.startswith('L'):
                    self._laser_state = p[1] == '1'
                elif p.startswith('S'):
                    self._shutter_state = p[1] == '1'
                elif p.startswith('A'):
                    self._adr_state = p[1] == '1'
        except Exception:
            log.warning('ArduinoController: could not parse status %r', resp)

    # ── laser ─────────────────────────────────────────────────────────────────

    @property
    def laser_on(self) -> bool:
        return self._laser_state

    def set_laser(self, on: bool) -> None:
        cmd  = 'L1' if on else 'L0'
        resp = self._send(cmd)
        if 'OK' in resp:
            self._laser_state = on
            log.info('Laser → %s', 'ON' if on else 'OFF')
        else:
            log.error('Laser command failed: %r', resp)

    def laser_toggle(self) -> None:
        self.set_laser(not self._laser_state)

    # ── shutter ───────────────────────────────────────────────────────────────

    @property
    def shutter_open(self) -> bool:
        return self._shutter_state

    def set_shutter(self, open: bool) -> None:
        cmd  = 'S1' if open else 'S0'
        resp = self._send(cmd)
        if 'OK' in resp:
            self._shutter_state = open
            log.info('Shutter → %s', 'OPEN' if open else 'CLOSED')
        else:
            log.error('Shutter command failed: %r', resp)

    def shutter_toggle(self) -> None:
        self.set_shutter(not self._shutter_state)

    # ── status ────────────────────────────────────────────────────────────────

    def status(self) -> dict:
        self._sync()
        return {
            'laser':   self._laser_state,
            'shutter': self._shutter_state,
            'adr':     self._adr_state,
        }

    # ── ADR-1805 interlock ───────────────────────────────────────────────────

    @property
    def adr_enabled(self) -> bool:
        return self._adr_state

    def set_adr(self, enabled: bool) -> None:
        cmd  = 'A1' if enabled else 'A0'
        resp = self._send(cmd)
        if 'OK' in resp:
            self._adr_state = enabled
            log.info('ADR-1805 -> %s', 'ENABLED' if enabled else 'DISABLED')
        else:
            log.error('ADR command failed: %r', resp)

    def adr_toggle(self) -> None:
        self.set_adr(not self._adr_state)

    def shutdown(self) -> None:
        """Safe state: all outputs off."""
        try:
            self.set_laser(False)
            self.set_shutter(False)
            self.set_adr(False)
            self._ser.close()
        except Exception:
            pass
