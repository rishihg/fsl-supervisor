"""
hardware/daqc2.py — thin wrapper around the DAQC2plate via BRIDGEplate.

On a non-Raspberry-Pi host the piplates library is accessed through the
BRIDGEplate USB bridge.  If the library is not installed (e.g. during
development without hardware) the module falls back to a stub that logs
every call instead of raising an ImportError.

Usage:
    board = DAQC2Board(addr=0)
    volts = board.read_adc(channel=1)
    board.set_dac(channel=1, voltage=2.5)
    board.set_dout(bit=0, state=True)   # pull output low (transistor ON)
"""

import logging

log = logging.getLogger(__name__)

# ── import Pi-Plates via BRIDGEplate ────────────────────────────────────────
try:
    from piplates.BRIDGEplate import DAQC2 as _DAQC2
    _HARDWARE = True
    log.debug('piplates BRIDGEplate loaded')
except ImportError:
    _DAQC2 = None
    _HARDWARE = False
    log.warning('piplates not available — DAQC2Board running in stub mode')


class DAQC2Board:
    """Interface to one DAQC2plate at the given board address (0–7).

    ADC channels 0–7: ±12 V, 16-bit (~366 µV/bit).
    DAC channels 1–4: 0–4.095 V, 12-bit (1 mV/step).
    DOUT bits  0–7:   open-drain, max 3 A / 30 VDC sink.
    DIN  bits  0–7:   3.3 V / 5 V compatible, pulled up.
    """

    def __init__(self, addr: int = 0):
        self.addr = addr

    # ── ADC ─────────────────────────────────────────────────────────────────

    def read_adc(self, channel: int) -> float:
        """Read one ADC channel. Returns volts in ±12 V range."""
        if _HARDWARE:
            return _DAQC2.getADC(self.addr, channel)
        log.debug('STUB read_adc(addr=%d, ch=%d) → 0.0', self.addr, channel)
        return 0.0

    def read_adc_all(self) -> list:
        """Read all 8 ADC channels. Returns list of 8 floats in volts."""
        if _HARDWARE:
            return _DAQC2.getADCall(self.addr)
        log.debug('STUB read_adc_all(addr=%d) → [0.0]*8', self.addr)
        return [0.0] * 8

    # ── DAC ─────────────────────────────────────────────────────────────────

    def set_dac(self, channel: int, voltage: float):
        """Set a DAC channel output (0–4.095 V, channels 1–4)."""
        voltage = max(0.0, min(4.095, voltage))
        if _HARDWARE:
            _DAQC2.setDAC(self.addr, channel, voltage)
        else:
            log.debug('STUB set_dac(addr=%d, ch=%d, v=%.3f)', self.addr, channel, voltage)

    def get_dac(self, channel: int) -> float:
        """Read back the current DAC output voltage."""
        if _HARDWARE:
            return _DAQC2.getDAC(self.addr, channel)
        log.debug('STUB get_dac(addr=%d, ch=%d) → 0.0', self.addr, channel)
        return 0.0

    # ── Digital output ───────────────────────────────────────────────────────

    def set_dout(self, bit: int, state: bool):
        """Drive a DOUT bit.  state=True pulls the output low (transistor ON)."""
        if _HARDWARE:
            if state:
                _DAQC2.setDOUTbit(self.addr, bit)
            else:
                _DAQC2.clrDOUTbit(self.addr, bit)
        else:
            log.debug('STUB set_dout(addr=%d, bit=%d, state=%s)', self.addr, bit, state)

    def toggle_dout(self, bit: int):
        if _HARDWARE:
            _DAQC2.toggleDOUTbit(self.addr, bit)
        else:
            log.debug('STUB toggle_dout(addr=%d, bit=%d)', self.addr, bit)

    # ── Digital input ────────────────────────────────────────────────────────

    def read_din(self, bit: int) -> bool:
        """Read a DIN bit. Returns True if high."""
        if _HARDWARE:
            return bool(_DAQC2.getDINbit(self.addr, bit))
        log.debug('STUB read_din(addr=%d, bit=%d) → False', self.addr, bit)
        return False

    def read_din_all(self) -> int:
        """Read all 8 DIN bits as a byte."""
        if _HARDWARE:
            return _DAQC2.getDINall(self.addr)
        log.debug('STUB read_din_all(addr=%d) → 0', self.addr)
        return 0
