"""
hardware/shutter.py — shutter abstraction over a DAQC2 digital output.

The DAQC2 DOUT bits are open-drain: setDOUTbit pulls the pin low
(transistor ON), clrDOUTbit releases it (transistor OFF).  Whether that
means the shutter opens or closes depends on the wiring; set active_low
accordingly.
"""

import logging
from .daqc2 import DAQC2Board

log = logging.getLogger(__name__)


class Shutter:
    """Controls one shutter via a DAQC2 DOUT bit.

    Args:
        board:      DAQC2Board instance.
        bit:        DOUT bit number (0–7).
        active_low: If True (default), pulling the bit low opens the shutter.
                    If False, pulling the bit low closes the shutter.
    """

    def __init__(self, board: DAQC2Board, bit: int, active_low: bool = True):
        self.board = board
        self.bit = bit
        self.active_low = active_low
        self._open = False
        # ensure shutter starts closed
        self.close()

    def open(self):
        self.board.set_dout(self.bit, state=self.active_low)
        self._open = True
        log.info('Shutter (bit %d) opened', self.bit)

    def close(self):
        self.board.set_dout(self.bit, state=not self.active_low)
        self._open = False
        log.info('Shutter (bit %d) closed', self.bit)

    @property
    def is_open(self) -> bool:
        return self._open
