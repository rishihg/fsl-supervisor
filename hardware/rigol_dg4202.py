"""
hardware/rigol_dg4202.py — Rigol DG4202 function/arbitrary waveform generator.

Talks directly to /dev/rigol_dg4202 (raw usbtmc file), same proven pattern
as hardware/cld1010.py. This sidesteps a real conflict: the kernel's usbtmc
driver already claims the device (creating /dev/usbtmc*), which blocks
pyvisa-py's USB backend from also claiming it via libusb — the two cannot
both hold the device at once. Going through pyvisa/QNCP for THIS specific
instrument therefore doesn't work on this machine's driver stack.

The SCPI commands sent here match QNCP's gen.Rigol_DG4000
(https://github.com/frozenbooger/QNCP) so behaviour is consistent with the
rest of the lab's tooling — only the transport differs.

Udev rule (fixes the /dev/usbtmc0 vs /dev/usbtmc3 vs ... numbering, which
otherwise depends on plug order — see /etc/udev/rules.d/99-hst-devices.rules):
    SUBSYSTEM=="usbmisc", ATTRS{idVendor}=="1ab1", ATTRS{idProduct}=="0641", \\
        SYMLINK+="rigol_dg4202", MODE="0666"
After adding: sudo udevadm control --reload-rules && sudo udevadm trigger,
then replug. Device will always appear at /dev/rigol_dg4202 regardless of
enumeration order.
"""

import logging
import time

log = logging.getLogger(__name__)


class RigolDG4202:
    """Driver for the Rigol DG4202, channel 1.

    Args:
        device:  Path to the usbtmc device, e.g. '/dev/rigol_dg4202'.
        timeout: Seconds to wait for a response before raising TimeoutError.
    """

    def __init__(self, device: str = '/dev/rigol_dg4202', timeout: float = 2.0,
                 channel: int = 1):
        self._device_path = device
        self._timeout = timeout
        self._ch = channel
        self._dev = open(device, 'r+b', buffering=0)

        idn = self._ask('*IDN?')
        log.info('Rigol DG4202 connected: %s', idn)

        self._freq_hz    = float(self._ask(f'SOUR{self._ch}:FREQ?'))
        self._duty_pct   = float(self._ask(f'SOUR{self._ch}:FUNC:SQU:DCYC?'))
        self._ampl_vpp   = float(self._ask(f'SOUR{self._ch}:VOLT?'))
        self._offset_v   = float(self._ask(f'SOUR{self._ch}:VOLT:OFFS?'))
        self._output_on  = self._ask(f'OUTP{self._ch}?').strip() in ('1', 'ON')
        self._phase_deg  = float(self._ask(f'SOUR{self._ch}:PHAS?'))
        self._waveform   = self._ask(f'SOUR{self._ch}:FUNC?').strip()
        self._pulse_width_s = 0.0
        self._pulse_delay_s = 0.0
        if 'PULS' in self._waveform.upper():
            try:
                self._pulse_width_s = float(self._ask(f'SOUR{self._ch}:FUNC:PULS:WIDT?'))
                self._pulse_delay_s = float(self._ask(f'SOUR{self._ch}:FUNC:PULS:DEL?'))
            except Exception:
                pass
        log.info('Rigol DG4202 state: waveform=%s freq=%.1fHz duty=%.1f%% '
                 'ampl=%.2fVpp offset=%.2fV phase=%.1fdeg output=%s',
                 self._waveform, self._freq_hz, self._duty_pct, self._ampl_vpp,
                 self._offset_v, self._phase_deg, self._output_on)

    # ── low-level (same resilient pattern as CLD1010) ────────────────────────

    def _reopen(self):
        log.warning('Rigol: reopening %s after a communication error',
                    self._device_path)
        try:
            self._dev.close()
        except Exception:
            pass
        time.sleep(0.2)
        self._dev = open(self._device_path, 'r+b', buffering=0)

    def _send_raw(self, cmd: str) -> None:
        self._dev.write((cmd + '\n').encode())

    def _read_raw(self) -> str:
        # NOTE: deliberately NOT using select() here. The kernel's usbtmc
        # driver does not reliably implement poll()/select() readiness on
        # this instrument — select() reports "not ready" even when a
        # response is actually pending, causing false timeouts. A plain
        # blocking read (relying on the kernel's own usbtmc read timeout)
        # is the proven-working approach, matching how CLD1010 worked for
        # months before select() was added there too.
        data = self._dev.read(256)
        if not data:
            raise TimeoutError('Rigol: empty response')
        return data.decode('utf-8', errors='replace').strip()

    def _send(self, cmd: str) -> None:
        try:
            self._send_raw(cmd)
        except OSError:
            self._reopen()
            self._send_raw(cmd)

    def _ask(self, cmd: str) -> str:
        try:
            self._send_raw(cmd)
            time.sleep(0.1)
            return self._read_raw()
        except (OSError, TimeoutError) as exc:
            log.warning('Rigol: command %r failed (%s); retrying once', cmd, exc)
            self._reopen()
            self._send_raw(cmd)
            time.sleep(0.1)
            return self._read_raw()

    # ── waveform / frequency / duty cycle ────────────────────────────────────

    @property
    def frequency_hz(self) -> float:
        return self._freq_hz

    def set_frequency(self, hz: float) -> None:
        self._send(f'SOUR{self._ch}:FREQ {hz:.6f}')
        time.sleep(0.05)
        self._freq_hz = float(self._ask(f'SOUR{self._ch}:FREQ?'))
        log.info('Rigol CH%d frequency -> %.3f Hz', self._ch, self._freq_hz)

    @property
    def duty_cycle_pct(self) -> float:
        return self._duty_pct

    def set_duty_cycle(self, pct: float) -> None:
        """Duty cycle in percent (0-100). Only meaningful for square wave."""
        pct = max(0.0, min(100.0, pct))
        self._send(f'SOUR{self._ch}:FUNC:SQU:DCYC {pct:.2f}')
        time.sleep(0.05)
        self._duty_pct = float(self._ask(f'SOUR{self._ch}:FUNC:SQU:DCYC?'))
        log.info('Rigol CH%d duty cycle -> %.1f%%', self._ch, self._duty_pct)

    def set_waveform(self, shape: str = 'SQU') -> None:
        """shape: SQU, SIN, PULS, RAMP, USER (arbitrary), etc."""
        self._send(f'SOUR{self._ch}:FUNC {shape}')
        self._waveform = shape
        log.info('Rigol CH%d waveform -> %s', self._ch, shape)

    @property
    def waveform(self) -> str:
        return self._waveform

    # ── amplitude / offset ────────────────────────────────────────────────────

    @property
    def amplitude_vpp(self) -> float:
        return self._ampl_vpp

    def set_amplitude(self, vpp: float) -> None:
        self._send(f'SOUR{self._ch}:VOLT {vpp:.4f}')
        time.sleep(0.05)
        self._ampl_vpp = float(self._ask(f'SOUR{self._ch}:VOLT?'))
        log.info('Rigol CH%d amplitude -> %.3f Vpp', self._ch, self._ampl_vpp)

    @property
    def offset_v(self) -> float:
        return self._offset_v

    def set_offset(self, volts: float) -> None:
        self._send(f'SOUR{self._ch}:VOLT:OFFS {volts:.4f}')
        time.sleep(0.05)
        self._offset_v = float(self._ask(f'SOUR{self._ch}:VOLT:OFFS?'))
        log.info('Rigol CH%d offset -> %.3f V', self._ch, self._offset_v)

    # ── phase / delay ─────────────────────────────────────────────────────────
    # A repeating waveform has no standalone "delay" control on this
    # instrument — delay is implemented as a phase shift, computed from
    # the current frequency: phase_deg = (delay_s / period_s) * 360.

    @property
    def phase_deg(self) -> float:
        return self._phase_deg

    @property
    def delay_s(self) -> float:
        if self._freq_hz <= 0:
            return 0.0
        return (self._phase_deg / 360.0) / self._freq_hz

    def set_phase(self, degrees: float) -> None:
        degrees = degrees % 360.0
        self._send(f'SOUR{self._ch}:PHAS {degrees:.3f}')
        time.sleep(0.05)
        self._phase_deg = float(self._ask(f'SOUR{self._ch}:PHAS?'))
        log.info('Rigol CH%d phase -> %.2f deg (%.3g s delay @ %.1f Hz)',
                 self._ch, self._phase_deg, self.delay_s, self._freq_hz)

    def set_delay(self, seconds: float) -> None:
        """Delay the waveform by `seconds`, converted to phase at the
        CURRENT frequency. Call this AFTER set_frequency()/apply_square()
        if you are changing frequency too — the conversion depends on it."""
        if self._freq_hz <= 0:
            log.error('Cannot set delay: frequency is 0 Hz')
            return
        degrees = (seconds * self._freq_hz) * 360.0
        self.set_phase(degrees)

    # ── output ────────────────────────────────────────────────────────────────

    @property
    def output_on(self) -> bool:
        return self._output_on

    def set_output(self, on: bool) -> None:
        self._send(f'OUTP{self._ch} {"ON" if on else "OFF"}')
        time.sleep(0.1)
        resp = self._ask(f'OUTP{self._ch}?').strip()
        self._output_on = resp in ('1', 'ON')
        log.info('Rigol CH%d output -> %s (confirmed: %s)',
                 self._ch, 'ON' if on else 'OFF', resp)

    def output_toggle(self) -> None:
        self.set_output(not self._output_on)

    # ── convenience: set everything at once (like SOUR:APPL:SQU) ────────────

    def apply_square(self, freq_hz, ampl_vpp, offset_v, duty_pct=None,
                     delay_s=None):
        """Set frequency/amplitude/offset in one SCPI call, then duty
        cycle and delay (delay is applied last since it depends on the
        just-set frequency)."""
        self._send(
            f'SOUR{self._ch}:APPL:SQU {freq_hz:.6f},{ampl_vpp:.4f},{offset_v:.4f}')
        time.sleep(0.1)
        self._waveform = 'SQU'
        if duty_pct is not None:
            self.set_duty_cycle(duty_pct)
        self._freq_hz  = float(self._ask(f'SOUR{self._ch}:FREQ?'))
        self._ampl_vpp = float(self._ask(f'SOUR{self._ch}:VOLT?'))
        self._offset_v = float(self._ask(f'SOUR{self._ch}:VOLT:OFFS?'))
        if delay_s is not None:
            self.set_delay(delay_s)
        log.info('Rigol CH%d applied square: %.1fHz %.2fVpp %.2fVoff delay=%.3gs',
                 self._ch, self._freq_hz, self._ampl_vpp, self._offset_v,
                 delay_s or 0.0)

    def apply_sine(self, freq_hz, ampl_vpp, offset_v=0.0):
        """Continuous sine wave at a fixed frequency."""
        self._send(
            f'SOUR{self._ch}:APPL:SIN {freq_hz:.6f},{ampl_vpp:.4f},{offset_v:.4f}')
        time.sleep(0.1)
        self._waveform = 'SIN'
        self._freq_hz  = float(self._ask(f'SOUR{self._ch}:FREQ?'))
        self._ampl_vpp = float(self._ask(f'SOUR{self._ch}:VOLT?'))
        self._offset_v = float(self._ask(f'SOUR{self._ch}:VOLT:OFFS?'))
        log.info('Rigol CH%d applied sine: %.1fHz %.2fVpp %.2fVoff',
                 self._ch, self._freq_hz, self._ampl_vpp, self._offset_v)

    def apply_dc(self, offset_v: float):
        """Constant/unmodulated output — what "CW" means for a laser
        driven through a MOD IN port: no modulation at all, laser stays
        continuously on at whatever level `offset_v` sets. Matches QNCP's
        own gen.Rigol_DG4000.DC() semantics (FUNC DC + offset)."""
        self._send(f'SOUR{self._ch}:FUNC DC')
        time.sleep(0.05)
        self._send(f'SOUR{self._ch}:VOLT:OFFS {offset_v:.4f}')
        time.sleep(0.05)
        self._waveform = 'DC'
        self._offset_v = float(self._ask(f'SOUR{self._ch}:VOLT:OFFS?'))
        log.info('Rigol CH%d applied DC (CW): %.3f V', self._ch, self._offset_v)

    # ── pulse: off -> on -> off, once per period, real delay+width in seconds ──
    # This is what you want for "off, then on for a bit, then off again" —
    # a genuine PULSe waveform with its own DELay/WIDTh, NOT a phase-shifted
    # continuous square wave (that's what set_delay()/set_phase() above do,
    # useful for syncing two continuous square waves against each other,
    # but not for a single delayed pulse within each period).

    @property
    def pulse_width_s(self) -> float:
        return self._pulse_width_s

    @property
    def pulse_delay_s(self) -> float:
        return self._pulse_delay_s

    def set_pulse_width(self, seconds: float) -> None:
        self._send(f'SOUR{self._ch}:FUNC:PULS:WIDT {seconds:.9f}')
        time.sleep(0.05)
        self._pulse_width_s = float(self._ask(f'SOUR{self._ch}:FUNC:PULS:WIDT?'))
        log.info('Rigol CH%d pulse width -> %.6g s', self._ch, self._pulse_width_s)

    def set_pulse_delay(self, seconds: float) -> None:
        """Real delay before the pulse's leading edge, in seconds — not a
        phase shift. This is the "off, then on, then off" delay."""
        self._send(f'SOUR{self._ch}:FUNC:PULS:DEL {seconds:.9f}')
        time.sleep(0.05)
        self._pulse_delay_s = float(self._ask(f'SOUR{self._ch}:FUNC:PULS:DEL?'))
        log.info('Rigol CH%d pulse delay -> %.6g s', self._ch, self._pulse_delay_s)

    def apply_pulse(self, freq_hz, width_s, delay_s, ampl_vpp, offset_v=0.0):
        """One pulse per period: off for `delay_s`, on for `width_s`, off
        for the rest of the period (1/freq_hz total)."""
        period_s = 1.0 / freq_hz if freq_hz > 0 else 1.0
        if delay_s + width_s > period_s:
            log.warning('Rigol CH%d: delay+width (%.6gs) exceeds period '
                       '(%.6gs) — pulse will be clipped or wrap',
                       self._ch, delay_s + width_s, period_s)
        self._send(f'SOUR{self._ch}:FUNC PULS')
        time.sleep(0.05)
        self._waveform = 'PULS'
        self._send(f'SOUR{self._ch}:FREQ {freq_hz:.6f}')
        time.sleep(0.05)
        self._send(f'SOUR{self._ch}:VOLT {ampl_vpp:.4f}')
        time.sleep(0.05)
        self._send(f'SOUR{self._ch}:VOLT:OFFS {offset_v:.4f}')
        time.sleep(0.05)
        self.set_pulse_width(width_s)
        self.set_pulse_delay(delay_s)
        self._freq_hz  = float(self._ask(f'SOUR{self._ch}:FREQ?'))
        self._ampl_vpp = float(self._ask(f'SOUR{self._ch}:VOLT?'))
        self._offset_v = float(self._ask(f'SOUR{self._ch}:VOLT:OFFS?'))
        log.info('Rigol CH%d applied pulse: %.1fHz width=%.6gs delay=%.6gs '
                 '%.2fVpp %.2fVoff',
                 self._ch, self._freq_hz, self._pulse_width_s,
                 self._pulse_delay_s, self._ampl_vpp, self._offset_v)

    # ── status ────────────────────────────────────────────────────────────────

    def status(self) -> dict:
        return {
            'waveform':       self._waveform,
            'frequency_hz':   self._freq_hz,
            'duty_cycle_pct': self._duty_pct,
            'amplitude_vpp':  self._ampl_vpp,
            'offset_v':       self._offset_v,
            'phase_deg':      self._phase_deg,
            'delay_s':        self.delay_s,
            'pulse_width_s':  self._pulse_width_s,
            'pulse_delay_s':  self._pulse_delay_s,
            'output_on':      self._output_on,
        }

    def shutdown(self) -> None:
        """Safe state: output off."""
        try:
            self.set_output(False)
            self._dev.close()
        except Exception:
            pass
