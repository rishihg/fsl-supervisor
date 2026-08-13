#!/usr/bin/env python3
"""
hst_supervisor.py — Operator TUI for the HST free-space optical terminal.

Navigation:
    Tab / Shift+Tab    cycle active panel
    Arrow keys         context-sensitive within active panel
    Enter              toggle / activate
    . / ,              double / halve step size
    q                  quit
"""

import contextlib
import json
import select
import logging
import os
import subprocess
import sys
import socket
import threading
import time

with contextlib.suppress(Exception):
    with open(os.devnull, 'w') as _devnull:
        with contextlib.redirect_stderr(_devnull):
            import piplates  # noqa: F401

from textual import events
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.screen import ModalScreen
from textual.widgets import Footer, Header, Input, Label, RichLog, Static

from flip_mount import FlipMount
from hst_config import (
    ANYLOOP_BINARY, VIEWER_SCRIPT, CAMERA_CONFIGS,
    FIBER_TIC_SERIALS, FIBER_TIC_STEP_SIZE,
    MDT693B_PORT, MDT693B_STEP_V,
    ARDUINO_PORT, CLD1010_DEVICE,
    RIGOL_DEVICE,
    QED_VNC_HOST,
    FLIP_MOUNT_SERIAL,
    DLCPRO_HOST, DLCPRO_SERIAL,
)
from hardware.mdt693b import MDT693B
from hardware.arduino_controller import ArduinoController
from hardware.cld1010 import CLD1010
from hardware.rigol_dg4202 import RigolDG4202
from hardware.dlcpro import DLCpro

log = logging.getLogger(__name__)

_FIBER_AXES   = list(FIBER_TIC_SERIALS.keys())   # ['x', 'y', 'z']
_PANELS       = ['cameras', 'fiber', 'piezo', 'lasers', 'shutters', 'funcgen']
_LASER_SLOTS  = ['cld1010', 's1fc635', 'adr1805', 'dlcpro']
_SHUTTER_SLOTS = ['shutter', 'flipmount1']


# ── logging bridge ────────────────────────────────────────────────────────────

_LEVEL_STYLE = {'WARNING': 'yellow', 'ERROR': 'red', 'CRITICAL': 'bold red'}

class _TUIHandler(logging.Handler):
    def __init__(self, app):
        super().__init__()
        self._app = app
    def emit(self, record):
        msg   = self.format(record)
        style = _LEVEL_STYLE.get(record.levelname, '')
        with contextlib.suppress(Exception):
            if threading.current_thread() is threading.main_thread():
                self._app._append_log(msg, style)
            else:
                self._app.call_from_thread(self._app._append_log, msg, style)


# ── camera viewer ─────────────────────────────────────────────────────────────

class AnyloopProcess:
    """Manages one anyloop subprocess. Matches BNL fsl-supervisor's design."""

    def __init__(self, binary):
        self.binary = binary
        self._proc = None

    def start(self, config):
        if self.running:
            raise RuntimeError('anyloop is already running')
        self._proc = subprocess.Popen(
            [self.binary, config],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        log.info('anyloop started: pid=%d config=%s', self._proc.pid, config)

    def stop(self, timeout=2.0):
        if not self.running:
            return
        self._proc.terminate()
        try:
            self._proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            log.warning('anyloop did not terminate cleanly; killing')
            self._proc.kill()
            self._proc.wait()
        log.info('anyloop stopped')

    @property
    def running(self):
        return self._proc is not None and self._proc.poll() is None


class CameraViewer:
    """One camera = one anyloop subprocess + one MJPEG viewer subprocess.

    Opened and closed explicitly (Enter key) — a camera you have not
    opened has NO process and NO thread. This matches BNL's proven
    fsl-supervisor design and avoids any background scanning that could
    disrupt another camera sharing the USB bus.
    """

    def __init__(self, slot, config, udp_port, name, http_port):
        self.slot        = slot
        self.config      = config
        self.udp_port    = udp_port
        self.name        = name
        self.http_port   = http_port
        self.exposure_us, self.gain = self._read_exposure_and_gain()
        self._anyloop    = AnyloopProcess(ANYLOOP_BINARY)
        self._viewer     = None
        self._ae_busy    = False   # guards against overlapping auto-expose runs

    def _read_exposure_and_gain(self):
        try:
            with open(self.config) as f:
                cfg = json.load(f)
            for dev in cfg.get('pipeline', []):
                if 'aylp_asi' in dev.get('uri', ''):
                    p = dev.get('params', {})
                    return int(p.get('exposure_us', 10000)), int(p.get('gain', 0))
        except Exception:
            pass
        return 10000, 0

    def set_exposure(self, us, gain=None):
        """Patch exposure (and optionally gain) in the config JSON. If the
        camera is currently open, restart it so the change takes effect —
        this is a deliberate stop/restart (like BNL's viewer), not a live
        tweak, and is the reliable way to apply ASI exposure changes."""
        with open(self.config) as f:
            cfg = json.load(f)
        for dev in cfg.get('pipeline', []):
            if 'aylp_asi' in dev.get('uri', ''):
                dev['params']['exposure_us'] = int(us)
                if gain is not None:
                    dev['params']['gain'] = int(gain)
        with open(self.config, 'w') as f:
            json.dump(cfg, f, indent=4)
        self.exposure_us = int(us)
        if gain is not None:
            self.gain = int(gain)
        log.info('%s: exposure -> %d us%s', self.name, int(us),
                 f' gain={gain}' if gain is not None else '')
        if self.is_open:
            self.close()
            self.open()

    def fetch_stats(self, timeout=1.0):
        """Read live min/max/mean/frames from the running viewer's /stats
        endpoint. Returns None if the camera isn't open or isn't responding
        yet (e.g. just restarted and hasn't produced a frame)."""
        if not self.is_open:
            return None
        import urllib.request
        try:
            with urllib.request.urlopen(
                    f'http://127.0.0.1:{self.http_port}/stats',
                    timeout=timeout) as r:
                return json.loads(r.read().decode())
        except Exception:
            return None

    def auto_expose(self, target_mean=130, max_iterations=10,
                    min_exposure_us=5, max_exposure_us=200_000,
                    max_gain=400, settle_s=1.5, log_fn=None):
        """Automatically converge on a good exposure AND gain, the way
        camera software does. Priority order, same as most auto-exposure
        algorithms (keeps noise down): raise EXPOSURE first, up to
        `max_exposure_us` (default 200ms — past that, live view gets
        sluggish). Only once exposure is maxed out does it start raising
        GAIN to get the rest of the brightness needed. Coming down from
        too bright works the same way in reverse: cut gain to zero first,
        then cut exposure.

        Each iteration restarts the camera (this pipeline can't tweak
        exposure/gain live), waits for a frame, reads min/max/mean, and
        adjusts. Stops once the frame is well-exposed (max in a safe
        headroom band, not clipped) or after max_iterations.
        """
        def _log(msg):
            if log_fn:
                log_fn(msg)

        if not self.is_open:
            _log(f'{self.name}: camera must be open to auto-expose')
            return

        if self._ae_busy:
            _log(f'{self.name}: auto-expose already running, ignoring '
                f'extra press (this is likely what caused the flicker '
                f'— multiple overlapping runs fighting each other)')
            return
        self._ae_busy = True

        try:
            exposure = self.exposure_us
            gain     = self.gain
            prev_exposure, prev_gain = None, None

            for i in range(max_iterations):
                self.set_exposure(exposure, gain=gain)
                time.sleep(settle_s)   # let the restarted camera produce a frame
                stats = self.fetch_stats()
                if not stats or stats.get('frames', 0) == 0:
                    time.sleep(settle_s)
                    stats = self.fetch_stats()
                    if not stats:
                        _log(f'{self.name}: auto-expose gave up (camera not responding)')
                        return

                mn, mx, mean = stats['min'], stats['max'], stats['mean']
                _log(f'{self.name}: auto-expose iter {i+1}: exp={exposure}us '
                    f'gain={gain} min={mn} max={mx} mean={mean:.1f}')

                # If this iteration landed on the exact same settings as the
                # last one, nothing will change by continuing — stop instead
                # of oscillating forever at a noisy boundary.
                if (exposure, gain) == (prev_exposure, prev_gain):
                    _log(f'{self.name}: auto-expose settled (no further '
                        f'change) at exp={exposure}us gain={gain}')
                    return
                prev_exposure, prev_gain = exposure, gain

                if mx >= 254:
                    # Saturated. Cut gain first (it's "free" — doesn't cost
                    # frame rate), then exposure if gain is already at 0.
                    if gain > 0:
                        gain = max(0, int(gain * 0.3))
                    else:
                        exposure = max(min_exposure_us, int(exposure * 0.3))
                    continue

                if 170 <= mx <= 250:
                    _log(f'{self.name}: auto-expose converged: exp={exposure}us '
                        f'gain={gain} (max={mx}, mean={mean:.1f})')
                    return

                if mean < 3:
                    # Essentially black — big jump, exposure first
                    if exposure < max_exposure_us:
                        exposure = min(max_exposure_us, int(exposure * 8))
                    else:
                        gain = min(max_gain, max(gain * 2, 20))
                    continue

                # Proportional scaling toward target, clamped so one step
                # can't overshoot wildly (also dampened vs. a full jump to
                # reduce boundary oscillation). Exposure does the work
                # until it hits the cap; gain only kicks in once exposure
                # is maxed out.
                scale = target_mean / max(mean, 1.0)
                scale = max(0.5, min(scale, 3.0))
                # Dampen: move 70% of the way toward the proportional
                # target instead of the full jump, to settle rather than
                # bounce past and back.
                scale = 1.0 + (scale - 1.0) * 0.7

                if exposure < max_exposure_us or scale < 1.0:
                    exposure = int(exposure * scale)
                    exposure = max(min_exposure_us, min(max_exposure_us, exposure))
                else:
                    gain = int(gain * scale) if gain > 0 else 20
                    gain = max(0, min(max_gain, gain))

            _log(f'{self.name}: auto-expose stopped after {max_iterations} '
                f'iterations at exp={exposure}us gain={gain} (best effort — check image)')
        finally:
            self._ae_busy = False

    def _evict_port(self, port):
        """Kill any stale process already holding our HTTP port (e.g. from
        a previous crashed session)."""
        result = subprocess.run(
            ['ss', '-tlnpH', f'sport = :{port}'],
            capture_output=True, text=True,
        )
        for line in result.stdout.splitlines():
            m = __import__('re').search(r'pid=(\d+)', line)
            if m:
                with contextlib.suppress(ProcessLookupError):
                    os.kill(int(m.group(1)), 15)
        if result.stdout.strip():
            time.sleep(0.5)

    def open(self):
        self._evict_port(self.http_port)
        self._viewer = subprocess.Popen(
            [sys.executable, VIEWER_SCRIPT,
             str(self.udp_port), str(self.http_port), self.name],
            stdout=subprocess.PIPE, stderr=None,
        )
        readable, _, _ = select.select([self._viewer.stdout], [], [], 10.0)
        if not readable:
            self._viewer.terminate()
            raise RuntimeError(f'{self.name} viewer did not signal READY')
        line = self._viewer.stdout.readline().strip()
        if line != b'READY':
            self._viewer.terminate()
            raise RuntimeError(f'{self.name} viewer failed to start ({line!r})')
        self._anyloop.start(self.config)
        log.info('%s stream at http://localhost:%d/', self.name, self.http_port)

    def close(self):
        if self._viewer and self._viewer.poll() is None:
            self._viewer.terminate()
        self._viewer = None
        self._anyloop.stop()
        log.info('%s closed', self.name)

    @property
    def is_open(self):
        return self._anyloop.running

# ── Tic axis ──────────────────────────────────────────────────────────────────

class TicAxis:
    def __init__(self, serial_number, step_size=5):
        import subprocess
        from ticlib import TicUSB
        self._serial    = serial_number
        # Reset clears any brown-out or driver error state
        subprocess.run(['ticcmd', '-d', serial_number, '--reset'],
                       capture_output=True)
        time.sleep(0.5)
        self._tic       = TicUSB(serial_number=serial_number)
        self._step_size = max(1, int(step_size))
        self._tic.energize()
        self._tic.exit_safe_start()
        log.info('TicAxis serial=%s step=%d initialised', serial_number, step_size)

    def step(self, direction):
        import subprocess
        subprocess.run(['ticcmd', '-d', self._serial, '--reset-command-timeout'],
                       capture_output=True)
        self._tic.energize()
        self._tic.exit_safe_start()
        pos = self._tic.get_current_position()
        self._tic.set_target_position(pos + direction * self._step_size)
        log.info('Fiber %s -> %+d', self._serial, direction * self._step_size)

    @property
    def step_size(self):
        return self._step_size

    @step_size.setter
    def step_size(self, v):
        self._step_size = max(1, int(v))

    def shutdown(self):
        with contextlib.suppress(Exception):
            self._tic.deenergize()


# ── modals ────────────────────────────────────────────────────────────────────

class ExposureModal(ModalScreen):
    DEFAULT_CSS = """
    ExposureModal { align: center middle; }
    ExposureModal > Vertical {
        width: 60; height: auto; border: round $primary;
        background: $surface; padding: 1 2;
    }
    ExposureModal Input { margin-top: 1; }
    """
    BINDINGS = [
        Binding('escape', 'cancel',  'Cancel'),
        Binding('enter',  'confirm', 'OK', priority=True),
    ]

    def __init__(self, cameras):
        super().__init__()
        self._cameras = cameras

    def compose(self):
        with Vertical():
            yield Label('[bold]Camera settings[/bold]\n')
            yield Label('[dim]slot  exposure_us  [gain]   e.g. 1 20000 200[/dim]')
            for slot, v in sorted(self._cameras.items()):
                exp = f'{v.exposure_us} us' if v.is_open else '[dim]closed[/dim]'
                yield Label(f'  {slot+1}  {v.name:<12}  {exp}')
            yield Input(placeholder='slot  exposure_us  [gain]', id='exp-input')

    def on_mount(self):
        self.query_one('#exp-input', Input).focus()

    def action_confirm(self):
        inp = self.query_one('#exp-input', Input)
        self.dismiss(inp.value.strip())

    def on_input_submitted(self, event):
        self.dismiss(event.value.strip())

    def action_cancel(self):
        self.dismiss(None)


class CLD1010Modal(ModalScreen):
    DEFAULT_CSS = """
    CLD1010Modal { align: center middle; }
    CLD1010Modal > Vertical {
        width: 56; height: auto; border: round $primary;
        background: $surface; padding: 1 2;
    }
    """
    BINDINGS = [Binding('escape', 'cancel', 'Cancel')]

    def __init__(self, laser, tec_on):
        super().__init__()
        self._laser  = laser
        self._tec_on = tec_on

    def compose(self):
        laser = self._laser
        lst = 'ON' if laser.laser_on else 'OFF'
        tst = 'ON' if self._tec_on else 'OFF'
        with Vertical():
            yield Label('[bold]CLD1010[/bold]  [dim](785 nm)[/dim]\n')
            yield Label(f'  Laser: {lst}   TEC: {tst}   '
                        f'setpoint: {laser.current_sp*1000:.1f} mA')
            yield Label('')
            yield Label('  l = toggle laser        k = toggle TEC')
            yield Label('  type a number = set current in mA')
            yield Label('[dim]Sequence: TEC on -> wait ~30s -> laser on[/dim]')
            yield Input(placeholder='mA, or leave blank and press l/k', id='cld-input')

    def on_mount(self):
        self.query_one('#cld-input', Input).focus()

    def on_input_submitted(self, event):
        # NOTE: the input box has focus, so 'l'/'k' are typed as text into
        # it (Textual's Input consumes plain character keys before a
        # separate on_key handler would ever see them) — interpret them
        # here on submit instead of trying to intercept the keypress.
        v = event.value.strip()
        if not v:
            self.dismiss(None)
        elif v.lower() == 'l':
            self.dismiss(('laser_toggle', None))
        elif v.lower() == 'k':
            self.dismiss(('tec_toggle', None))
        else:
            self.dismiss(('set_current', v))

    def action_cancel(self):
        self.dismiss(None)


class DLCproModal(ModalScreen):
    DEFAULT_CSS = """
    DLCproModal { align: center middle; }
    DLCproModal > Vertical {
        width: 56; height: auto; border: round $primary;
        background: $surface; padding: 1 2;
    }
    """
    BINDINGS = [Binding('escape', 'cancel', 'Cancel')]

    def __init__(self, laser):
        super().__init__()
        self._laser = laser

    def compose(self):
        laser = self._laser
        est = 'ON' if laser.emission_on else 'OFF'
        cst = 'ENABLED' if laser.cc_enabled else 'DISABLED'
        with Vertical():
            yield Label('[bold]Toptica DLC pro[/bold]\n')
            yield Label(f'  Emission: {est}   Diode: {cst}   '
                        f'setpoint: {laser.current_sp:.2f} mA')
            yield Label('')
            yield Label('  e = toggle diode enable/disable')
            yield Label('  type a number = set diode current in mA')
            yield Input(placeholder='mA, or leave blank and press e', id='dlc-input')

    def on_mount(self):
        self.query_one('#dlc-input', Input).focus()

    def on_input_submitted(self, event):
        # NOTE: same reasoning as CLD1010Modal above — the Input has focus,
        # so 'e' arrives as text here rather than a separate keypress.
        v = event.value.strip()
        if not v:
            self.dismiss(None)
        elif v.lower() == 'e':
            self.dismiss(('enable_toggle', None))
        else:
            self.dismiss(('set_current', v))

    def action_cancel(self):
        self.dismiss(None)


class FuncGenModal(ModalScreen):
    DEFAULT_CSS = """
    FuncGenModal { align: center middle; }
    FuncGenModal > Vertical {
        width: 62; height: auto; border: round $primary;
        background: $surface; padding: 1 2;
    }
    FuncGenModal Input { margin-top: 1; }
    """
    BINDINGS = [Binding('escape', 'cancel', 'Cancel')]

    def __init__(self, fg):
        super().__init__()
        self._fg = fg

    def compose(self):
        fg = self._fg
        with Vertical():
            yield Label('[bold]Function Generator[/bold]  [dim](Rigol DG4202 CH1)[/dim]\n')
            yield Label(f'  Now: [{fg.waveform}]  {fg.frequency_hz:,.1f} Hz   '
                        f'{fg.duty_cycle_pct:.1f}% duty   '
                        f'{fg.amplitude_vpp:.2f} Vpp   '
                        f'{fg.offset_v:.2f} V offset   '
                        f'phase {fg.phase_deg:.1f}deg   '
                        f'{"ON" if fg.output_on else "OFF"}')
            yield Label('')
            yield Label('[dim]Square (default):  freq_hz duty_% ampl_vpp offset_v [delay_s] [on|off][/dim]')
            yield Label('[dim]  e.g.  1000000 50 2.0 0.0 0 on[/dim]')
            yield Label('[dim]Sine:  sin freq_hz ampl_vpp [offset_v] [on|off][/dim]')
            yield Label('[dim]  e.g.  sin 1000000 1.0 0 on[/dim]')
            yield Label('[dim]DC / CW (constant, unmodulated — laser stays continuously on):[/dim]')
            yield Label('[dim]  dc offset_v [on|off]   e.g.  dc 2.5 on[/dim]')
            yield Label('[dim]Pulse (off, then on for width_s, then off — real delay in seconds):[/dim]')
            yield Label('[dim]  pulse freq_hz width_s delay_s ampl_vpp [offset_v] [on|off][/dim]')
            yield Label('[dim]  e.g.  pulse 1000 0.0001 0.00005 2.0 0 on[/dim]')
            yield Label('[dim]Press o on the panel (no dialog) to just toggle output.[/dim]')
            yield Input(placeholder='square: freq duty ampl offset [delay] [on|off]', id='fg-input')

    def on_mount(self):
        self.query_one('#fg-input', Input).focus()

    def on_input_submitted(self, event):
        self.dismiss(event.value.strip())

    def action_cancel(self):
        self.dismiss(None)


# ── main application ──────────────────────────────────────────────────────────

class HSTSupervisorApp(App):

    TITLE = 'HST Supervisor'

    DEFAULT_CSS = """
    Screen { background: $surface; }

    #status { height: 1; padding: 0 1; background: $primary-darken-3; }

    .panel {
        border: round $primary; padding: 0 2;
        height: auto; margin: 0 1 1 0;
    }
    .panel--active { border: round $success; }

    #row-top    { height: auto; margin: 1 0 0 1; }
    #row-bottom { height: auto; margin: 0 0 0 1; }

    #p-cameras  { width: 1fr; }
    #p-fiber    { width: 30; }
    #p-piezo    { width: 26; }
    #p-lasers   { width: 1fr; }
    #col-side   { width: 40; height: auto; }
    #p-shutters { width: 1fr; }
    #p-funcgen  { width: 1fr; }

    #log { border: round $primary; margin: 1; height: 10; }
    """

    BINDINGS = [
        Binding('ctrl+n',     'next_panel',   'Next panel'),
        Binding('ctrl+b',     'prev_panel',   'Prev panel'),
        Binding('up',         'panel_up',     'Up/+',    show=False, priority=True),
        Binding('down',       'panel_down',   'Down/-',  show=False, priority=True),
        Binding('left',       'panel_left',   'Left',    show=False, priority=True),
        Binding('right',      'panel_right',  'Right',   show=False, priority=True),
        Binding('enter',      'panel_enter',  'Toggle',  show=False),
        Binding('full_stop',  'step_finer',   'Step x2'),
        Binding('comma',      'step_coarser', 'Step /2'),
        Binding('e',          'set_exposure', 'Exposure...'),
        Binding('o',          'fg_output_toggle', 'FuncGen Out', show=False),
        Binding('a',          'auto_expose',  'Auto-expose', show=False),
        Binding('v',          'open_qed_vnc', 'quED VNC'),
        Binding('q',          'app_quit',     'Quit'),
    ]

    def __init__(self):
        super().__init__()
        self._cameras        = {}
        self.fiber_axes      = {}
        self.piezo           = None
        self.arduino         = None
        self.laser           = None
        self.dlcpro          = None
        self.funcgen         = None

        # navigation state
        self._panel          = 'cameras'   # active panel
        self._cam_slot       = 0
        self._fiber_axis     = 0           # index into _FIBER_AXES
        self._laser_slot     = 0           # index into _LASER_SLOTS
        self._shutter_slot   = 0           # index into _SHUTTER_SLOTS

        # laser state
        self._tec_on         = False
        self.flip_mount      = None
        self._flip_in        = True   # True = IN position
        self._current_step   = 1.0         # mA

        # piezo cached voltages
        self._pv             = {'x': 0.0, 'y': 0.0}
        # cached CLD1010 temperature — NEVER call measure_temperature()
        # directly from a render method (main thread); it's a blocking
        # hardware read and can freeze the whole TUI if the device hangs.
        self._cld_temp_str   = ''
        # CLD1010 auto-reconnect: a dropped connection previously stayed
        # dropped until the whole supervisor was restarted (and even then
        # would fail again if the USB endpoint was still wedged). _poll()
        # now retries CLD1010(...) periodically whenever self.laser is
        # None, on a cooldown so a genuinely unplugged laser doesn't spam
        # retries every 3s.
        self._laser_reconnect_next = 0.0
        self._laser_reconnecting   = False
        # Guards every self.laser device I/O call. Without this, the 3s
        # poll thread (temperature reads), the reconnect thread, and any
        # UI-triggered laser command thread can all touch the same usbtmc
        # fd concurrently and interleave commands/responses on the wire —
        # this looks exactly like random hardware failures but is
        # self-inflicted (no second process or second person needed).
        self._laser_lock = threading.Lock()

        # DLC pro state — same cache/lock/reconnect pattern as the CLD1010
        # above, just over a network (DeCoP) connection instead of usbtmc.
        self._dlcpro_current_step  = 1.0    # mA
        self._dlcpro_temp_str      = ''
        self._dlcpro_reconnect_next = 0.0
        self._dlcpro_reconnecting   = False
        self._dlcpro_lock = threading.Lock()

        # function generator step sizes (for future arrow-key nudging)
        self._fg_freq_step  = 100.0    # Hz
        self._fg_ampl_step  = 0.1      # Vpp

    # ── startup ───────────────────────────────────────────────────────────────

    def on_mount(self):
        handler = _TUIHandler(self)
        handler.setFormatter(logging.Formatter(
            '%(asctime)s  %(levelname)-8s  %(name)s: %(message)s',
            datefmt='%H:%M:%S'))
        logging.getLogger().addHandler(handler)
        logging.getLogger().setLevel(logging.INFO)

        for slot, (config, udp, name, http) in CAMERA_CONFIGS.items():
            self._cameras[slot] = CameraViewer(slot, config, udp, name, http)

        for axis, serial in FIBER_TIC_SERIALS.items():
            try:
                self.fiber_axes[axis] = TicAxis(serial, FIBER_TIC_STEP_SIZE)
            except Exception as exc:
                log.error('Fiber axis %s failed: %s', axis, exc)

        try:
            self.piezo = MDT693B(MDT693B_PORT, MDT693B_STEP_V)
            self._pv   = {'x': self.piezo.get_voltage('x'),
                          'y': self.piezo.get_voltage('y')}
        except Exception as exc:
            log.error('MDT693B failed: %s', exc)

        try:
            self.arduino = ArduinoController(ARDUINO_PORT)
        except Exception as exc:
            log.error('Arduino failed: %s', exc)

        # Flip mount
        try:
            self.flip_mount = FlipMount(FLIP_MOUNT_SERIAL)
            log.info('FlipMount serial=%s initialised', FLIP_MOUNT_SERIAL)
        except Exception as exc:
            log.error('FlipMount failed: %s', exc)

        try:
            with self._laser_lock:
                self.laser = CLD1010(CLD1010_DEVICE)
        except Exception as exc:
            log.error('CLD1010 failed: %s', exc)

        try:
            with self._dlcpro_lock:
                self.dlcpro = DLCpro(DLCPRO_HOST, serial=DLCPRO_SERIAL)
        except Exception as exc:
            log.error('DLC pro failed: %s', exc)

        try:
            self.funcgen = RigolDG4202(RIGOL_DEVICE)
        except Exception as exc:
            log.error('Rigol DG4202 failed: %s', exc)

        self._refresh()
        self.set_interval(3.0, self._poll)

    def _poll(self):
        # All blocking hardware I/O for periodic display happens in a
        # background thread. The main thread (this method, and every
        # render_* method) must only ever read cached instance variables —
        # a hardware read that hangs must never be able to freeze the TUI.
        def _fetch():
            if self.piezo:
                try:
                    self._pv = {'x': self.piezo.get_voltage('x'),
                                 'y': self.piezo.get_voltage('y')}
                except Exception:
                    pass
            if self.laser:
                try:
                    with self._laser_lock:
                        temp = self.laser.measure_temperature()
                    self._cld_temp_str = f'  T={temp:.1f}C'
                except Exception:
                    self._cld_temp_str = ''
            elif not self._laser_reconnecting and time.time() >= self._laser_reconnect_next:
                self._try_reconnect_laser()
            if self.dlcpro:
                try:
                    with self._dlcpro_lock:
                        st = self.dlcpro.poll()
                    self._dlcpro_temp_str = f'  T={st["temp_act"]:.2f}C'
                except Exception:
                    self._dlcpro_temp_str = ''
            elif not self._dlcpro_reconnecting and time.time() >= self._dlcpro_reconnect_next:
                self._try_reconnect_dlcpro()
            self.call_from_thread(self._refresh)
        threading.Thread(target=_fetch, daemon=True).start()

    def _try_reconnect_dlcpro(self):
        """Runs on the _poll background thread; mirrors
        _try_reconnect_laser below for the CLD1010. A dropped DLC pro
        connection (network blip, DLC pro reboot) retries here every 20s
        rather than requiring a supervisor restart."""
        self._dlcpro_reconnecting = True
        try:
            with self._dlcpro_lock:
                self.dlcpro = DLCpro(DLCPRO_HOST, serial=DLCPRO_SERIAL)
            log.info('DLC pro reconnected')
        except Exception as exc:
            log.debug('DLC pro reconnect attempt failed: %s', exc)
            self._dlcpro_reconnect_next = time.time() + 20.0
        finally:
            self._dlcpro_reconnecting = False

    def _try_reconnect_laser(self):
        """Runs on the _poll background thread. CLD1010(...) does its own
        blocking I/O (incl. a possible USB reset + retry, see
        hardware/cld1010.py), so this can take several seconds — the
        _laser_reconnecting guard stops overlapping attempts if a retry
        is still in flight when the next 3s poll tick fires."""
        self._laser_reconnecting = True
        try:
            with self._laser_lock:
                self.laser = CLD1010(CLD1010_DEVICE)
            log.info('CLD1010 reconnected')
        except Exception as exc:
            log.debug('CLD1010 reconnect attempt failed: %s', exc)
            self._laser_reconnect_next = time.time() + 20.0
        finally:
            self._laser_reconnecting = False

    # ── layout ────────────────────────────────────────────────────────────────

    def compose(self):
        yield Header()
        yield Static('', id='status')
        with Horizontal(id='row-top'):
            yield Static('', id='p-cameras',  classes='panel')
            yield Static('', id='p-fiber',    classes='panel')
            yield Static('', id='p-piezo',    classes='panel')
        with Horizontal(id='row-bottom'):
            yield Static('', id='p-lasers',   classes='panel')
            with Vertical(id='col-side'):
                yield Static('', id='p-funcgen',  classes='panel')
                yield Static('', id='p-shutters', classes='panel')
        yield RichLog(id='log', highlight=True, markup=True)
        yield Footer()

    def _refresh(self):
        # Update active panel border
        for pid in _PANELS:
            w = self.query_one(f'#p-{pid}', Static)
            if pid == self._panel:
                w.add_class('panel--active')
            else:
                w.remove_class('panel--active')

        self.query_one('#status',     Static).update(self._render_status())
        self.query_one('#p-cameras',  Static).update(self._render_cameras())
        self.query_one('#p-fiber',    Static).update(self._render_fiber())
        self.query_one('#p-piezo',    Static).update(self._render_piezo())
        self.query_one('#p-lasers',   Static).update(self._render_lasers())
        self.query_one('#p-shutters', Static).update(self._render_shutters())
        self.query_one('#p-funcgen',  Static).update(self._render_funcgen())

    def _append_log(self, msg, style=''):
        w = self.query_one('#log', RichLog)
        w.write(f'[{style}]{msg}[/{style}]' if style else msg)

    # ── render ────────────────────────────────────────────────────────────────

    def _render_status(self):
        hints = {
            'cameras':  'Enter=toggle  Up/Down=exposure  a=auto-expose',
            'fiber':    'Left/Right=axis  Up/Down=step  ./,=size',
            'piezo':    'Left/Right=X  Up/Down=Y  ./,=size',
            'lasers':   'Left/Right=select  Enter=toggle  Up/Down=current (CLD1010/DLC pro)',
            'shutters': 'Left/Right=select  Enter=toggle',
            'funcgen':  'Enter=settings dialog  o=quick output toggle',
        }
        panel_label = self._panel.upper()
        hint = hints.get(self._panel, '')
        return f'[bold cyan]{panel_label}[/bold cyan]  [dim]{hint}[/dim]   Click a panel, or Ctrl+N/Ctrl+B  q=quit'

    def _render_cameras(self):
        active = self._panel == 'cameras'
        lines = ['[bold]Cameras[/bold]  [dim](Enter=open/close, e=exposure/gain, a=auto-expose)[/dim]\n']
        for slot, v in sorted(self._cameras.items()):
            sel = slot == self._cam_slot and active
            marker = '[bold cyan]>[/bold cyan]' if sel else ' '
            if v.is_open:
                state = f'[green]OPEN[/green]  http://localhost:{v.http_port}/  {v.exposure_us}us  gain{v.gain}'
            else:
                state = '[dim]CLOSED[/dim]'
            lines.append(f' {marker} [{slot+1}] {v.name:<12} {state}')
        return '\n'.join(lines)

    def _render_fiber(self):
        active = self._panel == 'fiber'
        ax_name = _FIBER_AXES[self._fiber_axis]
        tobj    = self.fiber_axes.get(ax_name)
        step    = tobj.step_size if tobj else FIBER_TIC_STEP_SIZE
        lines   = [f'[bold]Fiber Stage[/bold]\n',
                   f'  step = {step} usteps\n']
        for i, ax in enumerate(_FIBER_AXES):
            t      = self.fiber_axes.get(ax)
            sel    = i == self._fiber_axis and active
            marker = '[bold cyan]>[/bold cyan]' if sel else ' '
            status = f'step={t.step_size}' if t else '[red]ERR[/red]'
            label  = f'[cyan]{ax}[/cyan]' if sel else ax
            lines.append(f' {marker} {label}  {status}')
        return '\n'.join(lines)

    def _render_piezo(self):
        active = self._panel == 'piezo'
        step   = self.piezo.step_v if self.piezo else MDT693B_STEP_V
        lines  = [f'[bold]Piezo Mirror[/bold]\n',
                  f'  step = {step:.1f} V\n']
        for ax in ('x', 'y'):
            v   = self._pv.get(ax, 0.0)
            if v is None:
                v = 0.0
            bar = self._vbar(v, 0, 150, 10)
            sel = (ax == 'x' and active) or (ax == 'y' and active)
            lbl = f'[cyan]{ax.upper()}[/cyan]' if active else ax.upper()
            key = 'Left/Right' if ax == 'x' else 'Up/Down'
            lines.append(f'  {lbl} [{key}]  {bar}  {v:6.1f} V')
        if not self.piezo:
            lines.append('\n  [red]NOT CONNECTED[/red]')
        return '\n'.join(lines)

    def _render_lasers(self):
        active = self._panel == 'lasers'
        lines  = ['[bold]Lasers[/bold]  [dim](Enter=open dialog for selected laser)[/dim]\n']

        items = [
            ('cld1010',  'CLD1010',   '785 nm'),
            ('s1fc635',  'S1FC635',   '635 nm'),
            ('adr1805',  'ADR-1805',  '795 nm'),
            ('dlcpro',   'DLC pro',   'tunable'),
        ]
        for i, (key, name, wl) in enumerate(items):
            sel    = i == self._laser_slot and active
            marker = '[bold cyan]>[/bold cyan]' if sel else ' '
            label  = f'[cyan]{name}[/cyan]' if sel else name

            if key == 'cld1010':
                if self.laser:
                    lst  = '[green]ON[/green]'  if self.laser.laser_on else '[dim]OFF[/dim]'
                    tst  = '[green]ON[/green]'  if self._tec_on        else '[dim]OFF[/dim]'
                    # cached value only — see _poll(), never a live read here
                    temp_str = self._cld_temp_str
                    lines.append(f' {marker} {label} [dim]({wl})[/dim]  Output:{lst}  TEC:{tst}  {self.laser.current_sp*1000:.1f}mA  step={self._current_step:.1f}mA{temp_str}')
                    if sel:
                        lines.append('         [dim]Enter=open CLD1010 dialog (l=laser k=TEC or type mA)[/dim]')
                else:
                    lines.append(f' {marker} {label} [dim]({wl})[/dim]  [red]NOT CONNECTED[/red]')

            elif key == 's1fc635':
                if self.arduino:
                    st = '[green]ENABLED[/green]' if self.arduino.laser_on else '[dim]DISABLED[/dim]'
                    lines.append(f' {marker} {label} [dim]({wl})[/dim]  Interlock:{st}')
                else:
                    lines.append(f' {marker} {label} [dim]({wl})[/dim]  [red]NOT CONNECTED[/red]')

            elif key == 'adr1805':
                if self.arduino:
                    st = '[green]ENABLED[/green]' if self.arduino.adr_enabled else '[dim]DISABLED[/dim]'
                    lines.append(f' {marker} {label} [dim]({wl})[/dim]  Interlock:{st}')
                else:
                    lines.append(f' {marker} {label} [dim]({wl})[/dim]  [red]NOT CONNECTED[/red]')

            elif key == 'dlcpro':
                if self.dlcpro:
                    est = '[green]ON[/green]' if self.dlcpro.emission_on else '[dim]OFF[/dim]'
                    dst = '[green]ENABLED[/green]' if self.dlcpro.cc_enabled else '[dim]DISABLED[/dim]'
                    # cached value only — see _poll(), never a live read here
                    temp_str = self._dlcpro_temp_str
                    lines.append(f' {marker} {label} [dim]({wl})[/dim]  Emission:{est}  Diode:{dst}  '
                                f'{self.dlcpro.current_sp:.1f}mA  step={self._dlcpro_current_step:.1f}mA{temp_str}')
                    if sel:
                        lines.append('         [dim]Enter=open DLC pro dialog (e=toggle diode or type mA)[/dim]')
                else:
                    lines.append(f' {marker} {label} [dim]({wl})[/dim]  [red]NOT CONNECTED[/red]')

            lines.append('')

        return '\n'.join(lines)

    def _render_shutters(self):
        active = self._panel == 'shutters'
        lines  = ['[bold]Shutters / Flip Mounts[/bold]  [dim](Enter=toggle)[/dim]\n']

        items = [
            ('shutter',    'Primary shutter'),
            ('flipmount1', 'Flip mount 1'),
        ]
        for i, (key, name) in enumerate(items):
            sel    = i == self._shutter_slot and active
            marker = '[bold cyan]>[/bold cyan]' if sel else ' '
            label  = f'[cyan]{name}[/cyan]' if sel else name

            if key == 'shutter':
                if self.arduino:
                    st = '[green]OPEN[/green]' if self.arduino.shutter_open else '[dim]CLOSED[/dim]'
                    lines.append(f' {marker} {label}   {st}')
                else:
                    lines.append(f' {marker} {label}   [red]NOT CONNECTED[/red]')
            else:
                if self.flip_mount:
                    fm_state = '[green]IN[/green]' if self._flip_in else '[dim]OUT[/dim]'
                    lines.append(f' {marker} {label}   {fm_state}')
                else:
                    lines.append(f' {marker} {label}   [red]NOT CONNECTED[/red]')

        return '\n'.join(lines)

    def _render_funcgen(self):
        lines = ['[bold]Function Generator[/bold]  [dim](Rigol DG4202 CH1)[/dim]\n']
        if self.funcgen:
            fg  = self.funcgen
            out = '[green]OUTPUT ON[/green]' if fg.output_on else '[dim]OUTPUT OFF[/dim]'
            lines.append(f'  {out}   [dim](o = quick toggle)[/dim]')
            lines.append(f'  Waveform   {fg.waveform}')
            if fg.waveform == 'DC':
                lines.append(f'  Level      {fg.offset_v:.3f} V  [dim](CW / unmodulated)[/dim]')
            else:
                lines.append(f'  Freq       {fg.frequency_hz:,.1f} Hz')
                if fg.waveform == 'SQU':
                    lines.append(f'  Duty cycle {fg.duty_cycle_pct:.1f} %')
                lines.append(f'  Amplitude  {fg.amplitude_vpp:.3f} Vpp')
                lines.append(f'  Offset     {fg.offset_v:.3f} V')
                if fg.waveform == 'SQU':
                    lines.append(f'  Phase delay {fg.delay_s*1e6:.2f} us  ({fg.phase_deg:.1f} deg)')
                elif 'PULS' in fg.waveform:
                    lines.append(f'  Pulse width {fg.pulse_width_s*1e6:.2f} us')
                    lines.append(f'  Pulse delay {fg.pulse_delay_s*1e6:.2f} us')
            lines.append('')
            lines.append('  [dim]Enter to edit settings[/dim]')
        else:
            lines.append('  [red]NOT CONNECTED[/red]')
        return '\n'.join(lines)

    @staticmethod
    def _vbar(v, vmin, vmax, width=10):
        if v is None:
            v = vmin
        frac   = max(0.0, min(1.0, (v - vmin) / (vmax - vmin)))
        filled = int(frac * width)
        return '[cyan]' + '█' * filled + '[/cyan]' + '░' * (width - filled)

    # ── flip mount state query ───────────────────────────────────────────────

    def _ask_flip_state(self):
        """Open dialog to choose where to move the flip mount."""
        current = 'IN' if self._flip_in else 'OUT'

        class FlipMoveModal(ModalScreen):
            DEFAULT_CSS = """
            FlipMoveModal { align: center middle; }
            FlipMoveModal > Vertical {
                width: 50; height: auto; border: round $primary;
                background: $surface; padding: 1 2;
            }
            """
            BINDINGS = [Binding('escape', 'cancel', 'Cancel')]

            def compose(self):
                with Vertical():
                    yield Label(f'[bold]Flip mount[/bold]  [dim](currently {current})[/dim]\n')
                    yield Label('  i = Move to IN   (element enters beam path)')
                    yield Label('  o = Move to OUT  (beam path clear, 180 deg)')
                    yield Label('')
                    yield Input(placeholder='i or o', id='flip-input')

            def on_mount(self):
                self.query_one('#flip-input', Input).focus()

            def on_input_submitted(self, event):
                v = event.value.strip().lower()
                if v in ('i', 'in'):
                    self.dismiss('in')
                elif v in ('o', 'out'):
                    self.dismiss('out')
                else:
                    self.dismiss(None)

            def action_cancel(self):
                self.dismiss(None)

        def apply(dest):
            if dest is None:
                return
            if not self.flip_mount:
                log.error('Flip mount not connected')
                return
            def _move():
                try:
                    if dest == 'in':
                        self.flip_mount.go_in()
                        self._flip_in = True
                        log.info('Flip mount -> IN')
                    else:
                        self.flip_mount.go_out()
                        self._flip_in = False
                        log.info('Flip mount -> OUT (180 deg)')
                except Exception as exc:
                    log.error('Flip mount move failed: %s', exc)
                self.call_from_thread(self._refresh)
            import threading
            threading.Thread(target=_move, daemon=True).start()

        self.push_screen(FlipMoveModal(), apply)

    # ── panel navigation ──────────────────────────────────────────────────────

    def on_click(self, event: events.Click) -> None:
        """Click anywhere on a panel to make it active — same effect as
        cycling to it with Ctrl+N/Ctrl+B."""
        widget = event.widget
        # Walk up from whatever was clicked to find a panel Static (id 'p-*')
        while widget is not None:
            wid = getattr(widget, 'id', None)
            if wid and wid.startswith('p-'):
                panel = wid[2:]
                if panel in _PANELS and panel != self._panel:
                    self._panel = panel
                    self._refresh()
                return
            widget = widget.parent

    def action_next_panel(self):
        idx = _PANELS.index(self._panel)
        self._panel = _PANELS[(idx + 1) % len(_PANELS)]
        self._refresh()

    def action_prev_panel(self):
        idx = _PANELS.index(self._panel)
        self._panel = _PANELS[(idx - 1) % len(_PANELS)]
        self._refresh()

    # ── arrow key routing ─────────────────────────────────────────────────────

    def _nudge_cld1010_current(self, delta_ma: float):
        """Runs the current-setpoint write on a background thread, guarded
        by _laser_lock like every other self.laser device I/O — this used
        to run inline on the main/UI thread with no lock, which both froze
        the TUI on a slow write and could race the poll thread."""
        def _run():
            try:
                with self._laser_lock:
                    new = max(0.0, self.laser.current_sp * 1000 + delta_ma)
                    self.laser.set_current(new / 1000.0)
                log.info('CLD1010 current -> %.1f mA', new)
            except Exception as exc:
                log.error('CLD1010 current adjust failed: %s', exc)
            self.call_from_thread(self._refresh)
        threading.Thread(target=_run, daemon=True).start()

    def _nudge_dlcpro_current(self, delta_ma: float):
        """Same pattern as _nudge_cld1010_current above, over _dlcpro_lock."""
        def _run():
            try:
                with self._dlcpro_lock:
                    new = max(0.0, self.dlcpro.current_sp + delta_ma)
                    self.dlcpro.set_current(new)
                log.info('DLC pro current -> %.2f mA', new)
            except Exception as exc:
                log.error('DLC pro current adjust failed: %s', exc)
            self.call_from_thread(self._refresh)
        threading.Thread(target=_run, daemon=True).start()

    def action_panel_up(self):
        if self._panel == 'cameras':
            v = self._cameras.get(self._cam_slot)
            if v and v.is_open:
                new_exp = min(1000000, int(v.exposure_us * 2))
                threading.Thread(target=v.set_exposure, args=(new_exp,),
                                 daemon=True).start()
        elif self._panel == 'fiber':
            self._fiber_step(1)
        elif self._panel == 'piezo':
            self._piezo_step('y', 1)
        elif self._panel == 'lasers':
            if self._laser_slot == 0 and self.laser:
                self._nudge_cld1010_current(+self._current_step)
            elif self._laser_slot == 3 and self.dlcpro:
                self._nudge_dlcpro_current(+self._dlcpro_current_step)

    def action_panel_down(self):
        if self._panel == 'cameras':
            v = self._cameras.get(self._cam_slot)
            if v and v.is_open:
                new_exp = max(10, int(v.exposure_us / 2))
                threading.Thread(target=v.set_exposure, args=(new_exp,),
                                 daemon=True).start()
        elif self._panel == 'fiber':
            self._fiber_step(-1)
        elif self._panel == 'piezo':
            self._piezo_step('y', -1)
        elif self._panel == 'lasers':
            if self._laser_slot == 0 and self.laser:
                self._nudge_cld1010_current(-self._current_step)
            elif self._laser_slot == 3 and self.dlcpro:
                self._nudge_dlcpro_current(-self._dlcpro_current_step)

    def action_panel_left(self):
        if self._panel == 'cameras':
            self._cam_slot = (self._cam_slot - 1) % max(1, len(self._cameras))
        elif self._panel == 'fiber':
            self._fiber_axis = (self._fiber_axis - 1) % len(_FIBER_AXES)
        elif self._panel == 'piezo':
            self._piezo_step('x', -1)
        elif self._panel == 'lasers':
            self._laser_slot = (self._laser_slot - 1) % len(_LASER_SLOTS)
        elif self._panel == 'shutters':
            self._shutter_slot = (self._shutter_slot - 1) % len(_SHUTTER_SLOTS)
        self._refresh()

    def action_panel_right(self):
        if self._panel == 'cameras':
            self._cam_slot = (self._cam_slot + 1) % max(1, len(self._cameras))
        elif self._panel == 'fiber':
            self._fiber_axis = (self._fiber_axis + 1) % len(_FIBER_AXES)
        elif self._panel == 'piezo':
            self._piezo_step('x', 1)
        elif self._panel == 'lasers':
            self._laser_slot = (self._laser_slot + 1) % len(_LASER_SLOTS)
        elif self._panel == 'shutters':
            self._shutter_slot = (self._shutter_slot + 1) % len(_SHUTTER_SLOTS)
        self._refresh()

    def action_panel_enter(self):
        if self._panel == 'cameras':
            v = self._cameras.get(self._cam_slot)
            if v:
                if v.is_open:
                    v.close()
                else:
                    try:
                        v.open()
                    except Exception as exc:
                        log.error('Camera open failed: %s', exc)
        elif self._panel == 'lasers':
            slot = _LASER_SLOTS[self._laser_slot]
            if slot == 'cld1010':
                if not self.laser:
                    log.error('CLD1010 not connected')
                    return
                self._open_cld1010_dialog()
                return
            elif slot == 's1fc635':
                if not self.arduino:
                    log.error('Arduino not connected')
                    return
                try:
                    self.arduino.laser_toggle()
                    log.info('S1FC635 -> %s', 'ENABLED' if self.arduino.laser_on else 'DISABLED')
                except Exception as exc:
                    log.error('S1FC635 toggle failed: %s', exc)
            elif slot == 'adr1805':
                if not self.arduino:
                    log.error('Arduino not connected')
                    return
                try:
                    self.arduino.adr_toggle()
                    log.info('ADR-1805 -> %s', 'ENABLED' if self.arduino.adr_enabled else 'DISABLED')
                except Exception as exc:
                    log.error('ADR toggle failed: %s', exc)
            elif slot == 'dlcpro':
                if not self.dlcpro:
                    log.error('DLC pro not connected')
                    return
                self._open_dlcpro_dialog()
                return
        elif self._panel == 'shutters':
            slot = _SHUTTER_SLOTS[self._shutter_slot]
            if slot == 'shutter':
                if not self.arduino:
                    log.error('Arduino not connected')
                    return
                try:
                    self.arduino.shutter_toggle()
                    log.info('Shutter -> %s', 'OPEN' if self.arduino.shutter_open else 'CLOSED')
                except Exception as exc:
                    log.error('Shutter toggle failed: %s', exc)
            else:
                if not self.flip_mount:
                    log.error('Flip mount not connected')
                    return
                self._ask_flip_state()
        elif self._panel == 'funcgen':
            self._open_funcgen_dialog()
        self._refresh()

    def _open_cld1010_dialog(self):
        def apply(result):
            if not result:
                return
            action, value = result
            def _run():
                try:
                    with self._laser_lock:
                        if action == 'laser_toggle':
                            if not self.laser.laser_on and not self._tec_on:
                                log.warning('TEC is OFF — turn TEC on first and wait ~30s')
                            self.laser.laser_toggle()
                            log.info('CLD1010 laser -> %s',
                                    'ON' if self.laser.laser_on else 'OFF')
                        elif action == 'tec_toggle':
                            self._tec_on = not self._tec_on
                            self.laser.set_tec(self._tec_on)
                            log.info('TEC -> %s', 'ON' if self._tec_on else 'OFF')
                        elif action == 'set_current':
                            ma = float(value)
                            self.laser.set_current(ma / 1000.0)
                            log.info('CLD1010 current -> %.1f mA', ma)
                except Exception as exc:
                    log.error('CLD1010 action failed: %s', exc)
                self.call_from_thread(self._refresh)
            threading.Thread(target=_run, daemon=True).start()

        self.push_screen(CLD1010Modal(self.laser, self._tec_on), apply)

    def _open_dlcpro_dialog(self):
        def apply(result):
            if not result:
                return
            action, value = result if isinstance(result, tuple) else ('set_current', result)
            def _run():
                try:
                    with self._dlcpro_lock:
                        if action == 'enable_toggle':
                            self.dlcpro.enable_toggle()
                            log.info('DLC pro diode -> %s',
                                    'ENABLED' if self.dlcpro.cc_enabled else 'DISABLED')
                        elif action == 'set_current':
                            ma = float(value)
                            self.dlcpro.set_current(ma)
                except Exception as exc:
                    log.error('DLC pro action failed: %s', exc)
                self.call_from_thread(self._refresh)
            threading.Thread(target=_run, daemon=True).start()

        self.push_screen(DLCproModal(self.dlcpro), apply)

    def _open_funcgen_dialog(self):
        if not self.funcgen:
            log.error('Rigol DG4202 not connected')
            return

        def apply(value):
            if not value:
                return
            parts = value.split()
            if not parts:
                return
            keyword = parts[0].lower()

            try:
                if keyword == 'sin':
                    # sin freq_hz ampl_vpp [offset_v] [on|off]
                    rest = parts[1:]
                    if len(rest) < 2:
                        log.error('Expected "sin freq_hz ampl_vpp [offset_v] [on|off]"')
                        return
                    freq   = float(rest[0])
                    ampl   = float(rest[1])
                    offset = float(rest[2]) if len(rest) >= 3 and rest[2].lower() not in ('on', 'off') else 0.0
                    out    = next((t.lower() for t in rest if t.lower() in ('on', 'off')), None)
                    def _run():
                        try:
                            self.funcgen.apply_sine(freq, ampl, offset)
                            if out:
                                self.funcgen.set_output(out == 'on')
                        except Exception as exc:
                            log.error('Function generator set failed: %s', exc)
                        self.call_from_thread(self._refresh)
                    threading.Thread(target=_run, daemon=True).start()

                elif keyword == 'pulse':
                    # pulse freq_hz width_s delay_s ampl_vpp [offset_v] [on|off]
                    rest = parts[1:]
                    if len(rest) < 4:
                        log.error('Expected "pulse freq_hz width_s delay_s ampl_vpp [offset_v] [on|off]"')
                        return
                    freq   = float(rest[0])
                    width  = float(rest[1])
                    delay  = float(rest[2])
                    ampl   = float(rest[3])
                    remainder = rest[4:]
                    offset = next((float(t) for t in remainder
                                   if t.lower() not in ('on', 'off')), 0.0)
                    out    = next((t.lower() for t in remainder if t.lower() in ('on', 'off')), None)
                    def _run():
                        try:
                            self.funcgen.apply_pulse(freq, width, delay, ampl, offset)
                            if out:
                                self.funcgen.set_output(out == 'on')
                        except Exception as exc:
                            log.error('Function generator set failed: %s', exc)
                        self.call_from_thread(self._refresh)
                    threading.Thread(target=_run, daemon=True).start()

                elif keyword == 'dc':
                    # dc offset_v [on|off]
                    rest = parts[1:]
                    if len(rest) < 1:
                        log.error('Expected "dc offset_v [on|off]"')
                        return
                    offset = float(rest[0])
                    out    = rest[1].lower() if len(rest) >= 2 else None
                    def _run():
                        try:
                            self.funcgen.apply_dc(offset)
                            if out in ('on', 'off'):
                                self.funcgen.set_output(out == 'on')
                        except Exception as exc:
                            log.error('Function generator set failed: %s', exc)
                        self.call_from_thread(self._refresh)
                    threading.Thread(target=_run, daemon=True).start()

                else:
                    # square (default): freq duty ampl offset [delay_s] [on|off]
                    if len(parts) < 4:
                        log.error('Expected "freq_hz duty_%% ampl_vpp offset_v [delay_s] [on|off]"')
                        return
                    freq   = float(parts[0])
                    duty   = float(parts[1])
                    ampl   = float(parts[2])
                    offset = float(parts[3])
                    out    = next((t.lower() for t in parts[4:] if t.lower() in ('on', 'off')), None)
                    numeric_rest = [t for t in parts[4:] if t.lower() not in ('on', 'off')]
                    delay  = float(numeric_rest[0]) if numeric_rest else None
                    def _run():
                        try:
                            self.funcgen.apply_square(freq, ampl, offset,
                                                      duty_pct=duty, delay_s=delay)
                            if out:
                                self.funcgen.set_output(out == 'on')
                        except Exception as exc:
                            log.error('Function generator set failed: %s', exc)
                        self.call_from_thread(self._refresh)
                    threading.Thread(target=_run, daemon=True).start()
            except ValueError:
                log.error('Invalid input: %r', value)
                return

        self.push_screen(FuncGenModal(self.funcgen), apply)

    # ── step size ─────────────────────────────────────────────────────────────

    def action_step_finer(self):
        if self._panel == 'fiber':
            ax = _FIBER_AXES[self._fiber_axis]
            t  = self.fiber_axes.get(ax)
            if t:
                t.step_size = t.step_size * 2
        elif self._panel == 'piezo' and self.piezo:
            self.piezo.step_v = self.piezo.step_v * 2
        elif self._panel == 'lasers' and self._laser_slot == 0:
            self._current_step = min(50.0, self._current_step * 2)
        elif self._panel == 'lasers' and self._laser_slot == 3:
            self._dlcpro_current_step = min(50.0, self._dlcpro_current_step * 2)
        self._refresh()

    def action_step_coarser(self):
        if self._panel == 'fiber':
            ax = _FIBER_AXES[self._fiber_axis]
            t  = self.fiber_axes.get(ax)
            if t:
                t.step_size = max(1, t.step_size // 2)
        elif self._panel == 'piezo' and self.piezo:
            self.piezo.step_v = max(0.1, self.piezo.step_v / 2)
        elif self._panel == 'lasers' and self._laser_slot == 0:
            self._current_step = max(0.1, self._current_step / 2)
        elif self._panel == 'lasers' and self._laser_slot == 3:
            self._dlcpro_current_step = max(0.1, self._dlcpro_current_step / 2)
        self._refresh()

    # ── motor helpers ─────────────────────────────────────────────────────────

    def _fiber_step(self, direction):
        ax   = _FIBER_AXES[self._fiber_axis]
        tobj = self.fiber_axes.get(ax)
        if not tobj:
            log.error('Fiber axis %s not initialised', ax)
            return
        try:
            tobj.step(direction)
        except Exception as exc:
            log.error('Fiber step failed: %s', exc)
        self._refresh()

    def _piezo_step(self, axis, direction):
        if not self.piezo:
            log.error('MDT693B not connected')
            return
        try:
            current = self._pv.get(axis, 75.0)
            target  = max(0.0, min(150.0, current + direction * self.piezo.step_v))
            self.piezo.set_voltage(axis, target)
            self._pv[axis] = target
        except Exception as exc:
            log.error('Piezo step failed: %s', exc)
        self._refresh()

    # ── global actions ────────────────────────────────────────────────────────

    def action_fg_output_toggle(self):
        if self._panel != 'funcgen':
            return
        if not self.funcgen:
            log.error('Rigol DG4202 not connected')
            return
        def _run():
            try:
                self.funcgen.output_toggle()
                log.info('Rigol output -> %s', 'ON' if self.funcgen.output_on else 'OFF')
            except Exception as exc:
                log.error('Output toggle failed: %s', exc)
            self.call_from_thread(self._refresh)
        threading.Thread(target=_run, daemon=True).start()

    def action_open_qed_vnc(self):
        """Quick-launch the quED control unit's VNC desktop as its own
        window. Not embedded in the TUI (Textual renders text, not a video
        framebuffer) — this just saves remembering the IP and typing the
        vncviewer command each time."""
        try:
            subprocess.Popen(
                ['vncviewer', f'{QED_VNC_HOST}:5900'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            log.info('Launching quED VNC (%s:5900)...', QED_VNC_HOST)
        except FileNotFoundError:
            log.error('vncviewer not found. Install with: '
                     'sudo apt install tigervnc-viewer')
        except Exception as exc:
            log.error('Failed to launch quED VNC: %s', exc)

    def action_auto_expose(self):
        if self._panel != 'cameras':
            return
        v = self._cameras.get(self._cam_slot)
        if not v:
            return
        if not v.is_open:
            log.error('%s: open the camera first (Enter), then auto-expose', v.name)
            return
        if v._ae_busy:
            log.warning('%s: auto-expose already running, please wait for it '
                       'to finish (pressing it again mid-run causes flicker)',
                       v.name)
            return
        log.info('%s: starting auto-expose...', v.name)
        def _run():
            v.auto_expose(log_fn=log.info)
            self.call_from_thread(self._refresh)
        threading.Thread(target=_run, daemon=True).start()

    def action_set_exposure(self):
        def apply(value):
            if not value:
                return
            parts = value.split()
            if len(parts) < 2:
                log.error('Expected "slot exposure_us [gain]"')
                return
            try:
                slot = int(parts[0]) - 1
                us   = int(parts[1])
                gain = int(parts[2]) if len(parts) >= 3 else None
            except ValueError:
                log.error('Invalid input: %r', value)
                return
            v = self._cameras.get(slot)
            if not v:
                log.error('Unknown slot %d', slot + 1)
                return
            def _run():
                v.set_exposure(us, gain)
                self.call_from_thread(self._refresh)
            threading.Thread(target=_run, daemon=True).start()
        self.push_screen(ExposureModal(self._cameras), apply)

    def action_app_quit(self):
        log.info('Shutting down...')
        # Cameras are subprocesses of THIS supervisor (matching BNL's
        # fsl-supervisor design). Closing them here is correct — the way
        # to keep a stream alive across sessions is to NOT actually quit:
        # detach the tmux session instead, and reattach with `tmux attach`.
        for v in self._cameras.values():
            with contextlib.suppress(Exception): v.close()

        for t in self.fiber_axes.values():
            with contextlib.suppress(Exception): t.shutdown()
        if self.piezo:
            with contextlib.suppress(Exception): self.piezo.shutdown()
        if self.laser:
            with self._laser_lock, contextlib.suppress(Exception):
                self.laser.shutdown()
        if self.dlcpro:
            with self._dlcpro_lock, contextlib.suppress(Exception):
                self.dlcpro.shutdown()
        if self.funcgen:
            with contextlib.suppress(Exception): self.funcgen.shutdown()
        if self.arduino:
            with contextlib.suppress(Exception): self.arduino.shutdown()
        if self.flip_mount:
            with contextlib.suppress(Exception): self.flip_mount._deenergize()
        self.exit()


# ── entry point ───────────────────────────────────────────────────────────────

def _acquire_single_instance_lock():
    """Refuse to start a second supervisor process. Two instances opening
    the same hardware (CLD1010, Arduino, MDT693B, Tics) at once causes
    interleaved/garbled communication that looks like random hardware
    failures after a while. If you want a second terminal on this session,
    use: tmux attach -t hst  — do NOT run this script again directly."""
    import fcntl
    lock_path = '/tmp/hst_supervisor.lock'
    # Open with 'a+' (not 'w') so a failed flock attempt below does NOT
    # truncate the file — 'w' truncates on open regardless of whether the
    # lock is subsequently acquired, which wipes out the PID left behind
    # by the process that actually holds the lock, breaking the "who has
    # it" diagnostic just when it's needed most (a second copy starting
    # while the first is running).
    lock_file = open(lock_path, 'a+')
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        lock_file.seek(0)
        held_pid = lock_file.read().strip()
        print()
        print('=' * 70)
        print('ERROR: hst_supervisor.py is already running'
              + (f' (PID {held_pid})' if held_pid else '') + ' (lock held).')
        print()
        print('Running a second copy will cause the CLD1010, Arduino, and')
        print('Tic controllers to receive interleaved commands from both')
        print('processes — this is very likely what caused hardware to')
        print('show as "NOT CONNECTED" after a while.')
        print()
        print('Instead, attach to the already-running session:')
        print('    tmux attach -t hst')
        print('=' * 70)
        sys.exit(1)
    lock_file.seek(0)
    lock_file.truncate()
    lock_file.write(str(os.getpid()))
    lock_file.flush()
    return lock_file  # keep a reference so the lock isn't released by GC


if __name__ == '__main__':
    _lock = _acquire_single_instance_lock()
    HSTSupervisorApp().run()
