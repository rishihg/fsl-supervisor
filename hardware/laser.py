"""
hardware/laser.py — laser enable and intensity control via DAQC2.

Enable/disable: one DOUT bit (drives a relay or MOSFET gate).
Intensity:      one DAC channel (0–4.095 V analog modulation input).
                Leave intensity_channel=None if the laser has no analog
                modulation input.
"""

import logging
from .daqc2 import DAQC2Board

log = logging.getLogger(__name__)

DAC_MAX_V = 4.095   # DAQC2 DAC ceiling


class Laser:
    """Controls laser enable (digital) and intensity (DAC).

    Args:
        board:             DAQC2Board instance.
        enable_bit:        DOUT bit that enables the laser (0–7).
        intensity_channel: DAC channel for analog intensity (1–4), or None.
        active_low:        If True, pulling the enable bit low turns the
                           laser on.  If False, pulling it low turns it off.
    """

    def __init__(self, board: DAQC2Board, enable_bit: int,
                 intensity_channel: int = None, active_low: bool = True):
        self.board = board
        self.enable_bit = enable_bit
        self.intensity_channel = intensity_channel
        self.active_low = active_low
        self._enabled = False
        self._intensity_v = 0.0
        # ensure laser starts off
        self.disable()

    def enable(self):
        self.board.set_dout(self.enable_bit, state=self.active_low)
        self._enabled = True
        log.info('Laser enabled')

    def disable(self):
        self.board.set_dout(self.enable_bit, state=not self.active_low)
        self._enabled = False
        log.info('Laser disabled')

    def set_intensity(self, voltage: float):
        """Set laser intensity via DAC (0–4.095 V).

        The meaning of the voltage depends on the laser's modulation input
        spec; document the calibration in the system notes.
        """
        if self.intensity_channel is None:
            raise RuntimeError('No intensity DAC channel configured for this laser')
        voltage = max(0.0, min(DAC_MAX_V, voltage))
        self.board.set_dac(self.intensity_channel, voltage)
        self._intensity_v = voltage
        log.info('Laser intensity set to %.3f V', voltage)

    def get_intensity(self) -> float:
        return self._intensity_v

    @property
    def is_enabled(self) -> bool:
        return self._enabled
