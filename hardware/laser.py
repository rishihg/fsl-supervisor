"""
hardware/laser.py — laser intensity control via DAQC2 DAC.

The laser accepts a single 0–5 V analog input:
  0 V   = off / minimum power
  ~4 V  = maximum practical intensity (DAQC2 DAC ceiling)

Set intensity in volts with set_voltage(); call off() to disable.
The dead-zone near 0 V is laser-model-dependent — characterise before
running automated sequences.
"""

import logging
from .daqc2 import DAQC2Board

log = logging.getLogger(__name__)

DAC_MAX_V = 4.095   # DAQC2 DAC ceiling


class Laser:
    """Controls laser intensity via a single DAC channel.

    Args:
        board:       DAQC2Board instance.
        dac_channel: DAC channel wired to the laser's modulation input.
    """

    def __init__(self, board: DAQC2Board, dac_channel: int = 0):
        self.board = board
        self.dac_channel = dac_channel
        self._voltage = 0.0
        self.off()

    def set_voltage(self, voltage: float):
        """Set laser intensity (0–4.095 V). 0 V disables the laser."""
        voltage = max(0.0, min(DAC_MAX_V, voltage))
        self.board.set_dac(self.dac_channel, voltage)
        self._voltage = voltage
        log.info('Laser set to %.3f V', voltage)

    def off(self):
        """Set DAC to 0 V (laser off)."""
        self.set_voltage(0.0)

    @property
    def voltage(self) -> float:
        return self._voltage
