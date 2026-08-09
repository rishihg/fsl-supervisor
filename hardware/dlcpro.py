"""
hardware/dlcpro.py — Toptica DLC pro tunable diode laser driver.

Communicates over Ethernet via the DeCoP protocol using the Toptica Python
Laser SDK (toptica-lasersdk), talking to the decop tree generated for
firmware v2.2.0 (toptica.lasersdk.dlcpro.v2_2_0). Install with:
    pip install --user toptica-lasersdk

Only the parameters this rig actually needs are wired up: emission status,
laser1 diode current setpoint (mA), laser1 diode temperature (C), and the
laser1 diode current-controller enable bit (laser1.dl.cc.enabled) — the
software on/off switch for that channel. The DLC pro's wavelength-tuning/
lock machinery is intentionally not exposed here. Add more parameters
if/when the rig needs them.

Note `emission` (read-only, whole-device) and `cc_enabled` (this channel's
software enable) are related but distinct: emission also reflects the key
switch and interlock state, so it can read False even with cc_enabled set
True (interlock open, key off) — don't assume they always agree.

Robustness notes:
  - Every decop get()/set() goes through _ask(), which retries once via
    _reopen() on failure. Unlike the CLD1010's usbtmc transport, there's no
    known "wedged endpoint" failure mode here — DeCoP over TCP just drops
    the socket on a network blip or a DLC pro reboot, so _reopen() is a
    plain close-and-reconnect (new NetworkConnection + DLCpro session), no
    USBDEVFS_RESET analogue needed. See hardware/cld1010.py for that
    driver's version of this same retry pattern.
  - Run only ONE process against a given DLC pro's command line at a time.
    The command line is a single logical connection per device — a second
    process (e.g. Toptica's own DLC pro GUI, or another operator's script)
    talking to it concurrently can change the setpoint out from under this
    driver's cached view of it, same failure mode as two processes sharing
    the CLD1010's usbtmc device.
"""

import logging
import time

from toptica.lasersdk.client import NetworkConnection, DecopError
from toptica.lasersdk.dlcpro.v2_2_0 import DLCpro as _DLCproAPI

log = logging.getLogger(__name__)


class DLCpro:
    """Driver for a Toptica DLC pro over the network (DeCoP protocol).

    Args:
        host:    IP address or hostname of the DLC pro, e.g. '10.2.245.161'.
        serial:  Expected serial number (e.g. 'DLC PRO_041413'). If given
                 and it doesn't match what the device reports, a warning is
                 logged — this is a sanity check only, not enforced.
        timeout: Seconds to wait for a response before raising.
    """

    def __init__(self, host: str, serial: str | None = None, timeout: float = 5.0):
        self._host    = host
        self._timeout = timeout
        self._conn, self._dlc = self._connect()

        actual_serial = self._ask(lambda: self._dlc.serial_number.get())
        if serial and actual_serial != serial:
            log.warning('DLC pro serial mismatch: expected %s, got %s',
                        serial, actual_serial)
        log.info('DLC pro connected: %s @ %s', actual_serial, host)

        self._emission_on = self._ask(lambda: self._dlc.emission.get())
        self._current_sp  = self._ask(lambda: self._dlc.laser1.dl.cc.current_set.get())
        self._cc_enabled  = self._ask(lambda: self._dlc.laser1.dl.cc.enabled.get())
        log.info('DLC pro state: emission=%s current_sp=%.2f mA cc_enabled=%s',
                 self._emission_on, self._current_sp, self._cc_enabled)

    # ── low-level ─────────────────────────────────────────────────────────────

    def _connect(self):
        # NetworkConnection is the async connection primitive; DLCpro wraps
        # it in a Client that owns opening/closing it via its own
        # background event loop thread. Don't call conn.open()/conn.close()
        # directly here — those are coroutines on the raw object and doing
        # so is a silent no-op (confirmed: it produces a "coroutine was
        # never awaited" RuntimeWarning and does nothing), not an error, so
        # this is easy to get wrong without it looking broken.
        conn = NetworkConnection(self._host, timeout=self._timeout)
        dlc = _DLCproAPI(conn)
        dlc.open()
        return conn, dlc

    def _reopen(self):
        log.warning('DLC pro: reconnecting to %s after a communication error',
                    self._host)
        try:
            self._dlc.close()
        except Exception:
            pass
        time.sleep(0.5)
        self._conn, self._dlc = self._connect()

    def _ask(self, fn):
        """Run a decop get()/set() call with one automatic reconnect-and-
        retry on failure."""
        try:
            return fn()
        except (DecopError, OSError) as exc:
            log.warning('DLC pro: command failed (%s); retrying once', exc)
            self._reopen()
            return fn()

    # ── status ────────────────────────────────────────────────────────────────

    @property
    def emission_on(self) -> bool:
        """Cached emission status, refreshed each poll()."""
        return self._emission_on

    @property
    def current_sp(self) -> float:
        """Diode current setpoint in mA (cached; refreshed on set_current())."""
        return self._current_sp

    @property
    def cc_enabled(self) -> bool:
        """Cached laser1 diode current-controller enable bit, refreshed each
        poll() (and immediately after set_enabled())."""
        return self._cc_enabled

    def poll(self) -> dict:
        """Live read of emission + diode temperature + cc_enabled.
        Deliberately never touches current_set — that's cached and only
        re-read after an explicit set_current(), same convention as
        CLD1010.current_sp."""
        self._emission_on = self._ask(lambda: self._dlc.emission.get())
        self._cc_enabled  = self._ask(lambda: self._dlc.laser1.dl.cc.enabled.get())
        temp = self._ask(lambda: self._dlc.laser1.dl.tc.temp_act.get())
        return {'emission_on': self._emission_on, 'cc_enabled': self._cc_enabled,
                'temp_act': temp}

    def measure_temp(self) -> float:
        """Live read of the laser1 diode temperature in Celsius."""
        return self._ask(lambda: self._dlc.laser1.dl.tc.temp_act.get())

    # ── current setpoint ─────────────────────────────────────────────────────

    def set_current(self, ma: float) -> None:
        """Set the laser1 diode current setpoint in mA."""
        self._ask(lambda: self._dlc.laser1.dl.cc.current_set.set(ma))
        self._current_sp = self._ask(lambda: self._dlc.laser1.dl.cc.current_set.get())
        log.info('DLC pro current setpoint -> %.2f mA', self._current_sp)

    # ── output enable ────────────────────────────────────────────────────────

    def set_enabled(self, on: bool) -> None:
        """Set the laser1 diode current-controller enable bit — this is the
        software on/off switch for the diode current (does not touch the
        setpoint). Does not guarantee emission_on flips too: emission also
        depends on the key switch/interlock (see module docstring)."""
        self._ask(lambda: self._dlc.laser1.dl.cc.enabled.set(on))
        self._cc_enabled = self._ask(lambda: self._dlc.laser1.dl.cc.enabled.get())
        log.info('DLC pro cc_enabled -> %s', self._cc_enabled)

    def enable_toggle(self) -> None:
        """Toggle the laser1 diode current-controller enable bit."""
        self.set_enabled(not self._cc_enabled)

    # ── shutdown ──────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        """Close the connection (dlc.close() also tears down the wrapped
        NetworkConnection — see _connect()). Deliberately does NOT disable
        the diode current controller on shutdown: this is a supervisor
        process exiting, not an operator request to turn the laser off, and
        the DLC pro keeps running its own state independent of this
        connection (same convention as CLD1010, which is self-powered)."""
        try:
            self._dlc.close()
        except Exception:
            pass
