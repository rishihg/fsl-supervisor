"""
tui.py — Textual TUI for the FSL supervisor.

Launched by supervisor.py --tui (or by default).
"""

import logging
import threading
import time
from datetime import datetime

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.screen import ModalScreen
from textual.widgets import Footer, Header, Input, Label, RichLog, Static

from anyloop_manager import AnyloopProcess, TelemetryReceiver, CommandSender
from hardware import DAQC2Board, RetroShutter, Shutter, MCLS1, TicAxis, KDC101Axis
from alignment import AlignmentStateMachine, State
from supervisor import (
    ANYLOOP_BINARY, ANYLOOP_CONFIG,
    TELEMETRY_PORT, COMMAND_PORT, N_COMMAND,
    DAQC2_ADDR, SHUTTER_BIT,
    SHUTTER_PORT, SHUTTER_OPEN_PW_US, SHUTTER_CLOSE_PW_US,
    MCLS1_PORT, MCLS1_CHANNEL, MCLS1_CURRENT_CAL, MCLS1_MAX_POWER_MW,
    TIC_SERIAL, TIC_STEP_SIZE, KDC101_PORT, KDC101_CHANNEL, KDC101_STEP_SIZE,
    CameraViewer, CAMERA_CONFIGS,
)

log = logging.getLogger(__name__)

# ── logging → TUI bridge ─────────────────────────────────────────────────────

_LEVEL_STYLE = {
    'WARNING':  'yellow',
    'ERROR':    'red',
    'CRITICAL': 'bold red',
}

class _TUIHandler(logging.Handler):
    """Forwards log records into the RichLog widget."""

    def __init__(self, app: 'FSLSupervisorApp') -> None:
        super().__init__()
        self._app = app

    def emit(self, record: logging.LogRecord) -> None:
        msg = self.format(record)
        style = _LEVEL_STYLE.get(record.levelname, '')
        try:
            if threading.current_thread() is threading.main_thread():
                self._app._append_log(msg, style)
            else:
                self._app.call_from_thread(self._app._append_log, msg, style)
        except Exception:
            pass


# ── state machine state → display ────────────────────────────────────────────

_STATE_STYLE: dict[State, tuple[str, str]] = {
    State.IDLE:         ('dim',      'IDLE'),
    State.COARSE_ALIGN: ('yellow',   'COARSE ALIGN'),
    State.FIBER_COUPLE: ('yellow',   'FIBER COUPLE'),
    State.LOCK_ACQUIRE: ('cyan',     'LOCK ACQUIRE'),
    State.RUNNING:      ('green',    'RUNNING'),
    State.STOPPING:     ('magenta',  'STOPPING'),
    State.FAULT:        ('bold red', 'FAULT'),
}


# ── laser voltage modal ───────────────────────────────────────────────────────

class LaserModal(ModalScreen):
    """Floating dialog for setting laser voltage."""

    DEFAULT_CSS = """
    LaserModal {
        align: center middle;
    }
    LaserModal > Vertical {
        width: 50;
        height: 9;
        border: round $primary;
        background: $surface;
        padding: 1 2;
    }
    LaserModal Label {
        margin-bottom: 1;
    }
    """

    BINDINGS = [Binding('escape', 'cancel', 'Cancel')]

    def compose(self) -> ComposeResult:
        with Vertical():
            yield Label(f'Set laser power (mW, {MCLS1_CURRENT_CAL[2]:.2f}–{MCLS1_CURRENT_CAL[3]:.1f}), or "off":')
            yield Input(placeholder=f'e.g.  {MCLS1_CURRENT_CAL[3]/2:.2f}', id='laser-input')

    def on_mount(self) -> None:
        self.query_one('#laser-input', Input).focus()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        self.dismiss(event.value.strip())

    def action_cancel(self) -> None:
        self.dismiss(None)


# ── exposure modal ────────────────────────────────────────────────────────────

class ExposureModal(ModalScreen):
    """Floating dialog for setting camera exposure."""

    DEFAULT_CSS = """
    ExposureModal {
        align: center middle;
    }
    ExposureModal > Vertical {
        width: 54;
        height: auto;
        border: round $primary;
        background: $surface;
        padding: 1 2;
    }
    ExposureModal Label {
        margin-bottom: 0;
    }
    ExposureModal #exp-header {
        margin-bottom: 1;
    }
    ExposureModal Input {
        margin-top: 1;
    }
    """

    BINDINGS = [Binding('escape', 'cancel', 'Cancel')]

    def __init__(self, cameras: dict) -> None:
        super().__init__()
        self._cameras = cameras

    def compose(self) -> ComposeResult:
        with Vertical():
            yield Label('[dim]slot  exposure_µs   (e.g. 0 20000)[/dim]',
                        id='exp-header')
            for idx, viewer in sorted(self._cameras.items()):
                exp_str = f'{viewer.exposure_us} µs' if viewer.is_open else '[dim]closed[/dim]'
                yield Label(f'  [dim]{idx}[/dim]  {viewer.name:<12}  {exp_str}')
            yield Input(placeholder='slot  exposure_µs', id='exp-input')

    def on_mount(self) -> None:
        self.query_one('#exp-input', Input).focus()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        self.dismiss(event.value.strip())

    def action_cancel(self) -> None:
        self.dismiss(None)


# ── main application ──────────────────────────────────────────────────────────

class FSLSupervisorApp(App):

    TITLE = 'FSL Supervisor'

    DEFAULT_CSS = """
    Screen {
        background: $surface;
    }

    #status {
        height: 1;
        padding: 0 2;
        background: $primary-darken-3;
    }

    #hardware {
        height: 12;
        border: round $primary;
        padding: 0 2;
        margin: 1 1 0 1;
    }

    #log {
        border: round $primary;
        margin: 1;
    }
    """

    BINDINGS = [
        Binding('s', 'start_loop',        'Start loop'),
        Binding('S', 'stop_loop',         'Stop loop'),
        Binding('o', 'open_shutter',      'Open shutter',  show=False),
        Binding('c', 'close_shutter',     'Close shutter', show=False),
        Binding('r', 'trigger_retro',     'Retro',         show=False),
        Binding('l', 'set_laser',         'Laser…',        show=False),
        Binding('1', 'camera_toggle(0)',  'Cam 290',       show=False),
        Binding('2', 'camera_toggle(1)',  'Cam 662a',      show=False),
        Binding('3', 'camera_toggle(2)',  'Cam 662b',      show=False),
        Binding('e', 'set_exposure',      'Exposure…',     show=False),
        # Steering mirror — arrow keys; priority so RichLog doesn't consume them
        Binding('left',  'mirror_az(-1)', 'Az−',   show=False, priority=True),
        Binding('right', 'mirror_az(1)',  'Az+',   show=False, priority=True),
        Binding('up',    'mirror_alt(1)', 'Alt+',  show=False, priority=True),
        Binding('down',  'mirror_alt(-1)','Alt−',  show=False, priority=True),
        Binding('[',     'mirror_step_coarser', 'Step÷2', show=False),
        Binding(']',     'mirror_step_finer',   'Step×2', show=False),
        Binding('a', 'abort',             'Abort'),
        Binding('q', 'quit',              'Quit'),
    ]

    def __init__(self, config: str = ANYLOOP_CONFIG) -> None:
        super().__init__()
        self._config = config
        self._loop_started_at: datetime | None = None

        # hardware (populated in on_mount; may remain None on init failure)
        self.anyloop:   AnyloopProcess | None       = None
        self.telemetry: TelemetryReceiver | None     = None
        self.commander: CommandSender | None         = None
        self.board:      DAQC2Board | None            = None
        self.retro:      RetroShutter | None         = None
        self.shutter:    Shutter | None              = None
        self.laser:      MCLS1 | None                = None
        self.mirror_az:  TicAxis | None              = None
        self.mirror_alt: KDC101Axis | None           = None
        self.sm:         AlignmentStateMachine | None = None
        self._cameras: dict[int, CameraViewer] = {}
        self._laser_pending: bool = False

    # ── layout ───────────────────────────────────────────────────────────────

    def compose(self) -> ComposeResult:
        yield Header()
        yield Static('', id='status')
        yield Static('', id='hardware', markup=True)
        yield RichLog(id='log', highlight=True, markup=True)
        yield Footer()

    # ── startup / shutdown ───────────────────────────────────────────────────

    def on_mount(self) -> None:
        # Redirect all logging into the RichLog widget.
        handler = _TUIHandler(self)
        handler.setFormatter(logging.Formatter(
            '%(asctime)s  %(levelname)-8s  %(name)s: %(message)s', '%H:%M:%S'
        ))
        logging.getLogger().addHandler(handler)

        # Hardware init — failures are logged but don't crash the TUI.
        try:
            self.anyloop   = AnyloopProcess(ANYLOOP_BINARY)
            self.telemetry = TelemetryReceiver(TELEMETRY_PORT)
            self.commander = CommandSender(COMMAND_PORT, N_COMMAND)
            self.board     = DAQC2Board(addr=DAQC2_ADDR)
            self.retro     = RetroShutter(self.board, bit=SHUTTER_BIT)
            self.shutter   = Shutter(SHUTTER_PORT,
                                     open_pw_us=SHUTTER_OPEN_PW_US,
                                     close_pw_us=SHUTTER_CLOSE_PW_US)
            self.laser     = MCLS1(MCLS1_PORT, channel=MCLS1_CHANNEL,
                                   current_cal=MCLS1_CURRENT_CAL)
        except Exception:
            log.exception('Hardware initialisation failed')

        try:
            self.mirror_az  = TicAxis(step_size=TIC_STEP_SIZE,
                                          serial_number=TIC_SERIAL)
        except Exception:
            log.exception('Mirror Az (Tic) init failed')

        try:
            self.mirror_alt = KDC101Axis(KDC101_PORT, channel=KDC101_CHANNEL,
                                         step_size=KDC101_STEP_SIZE)
        except Exception:
            log.exception('Mirror Alt (KDC101) init failed')

        try:
            self.sm = AlignmentStateMachine(
                anyloop   = self.anyloop,
                commander = self.commander,
                shutter   = self.retro,
                laser     = self.laser,
                telemetry = self.telemetry,
                config    = self._config,
            )
        except Exception:
            log.exception('Hardware initialisation failed')

        self._cameras = {
            idx: CameraViewer(idx, config, port, name, http_port)
            for idx, (config, port, name, http_port) in CAMERA_CONFIGS.items()
        }

        if self.telemetry:
            self.telemetry.start()

        threading.Thread(
            target=self._sm_loop, daemon=True, name='sm-stepper'
        ).start()

        self.set_interval(0.5, self._refresh_display)
        self._refresh_display()

    def on_unmount(self) -> None:
        if self.sm:
            self.sm.abort()
        if self.telemetry:
            self.telemetry.stop()
        if self.commander:
            self.commander.close()
        if self.shutter:
            self.shutter.shutdown()
        if self.laser:
            self.laser.shutdown()
        if self.mirror_az:
            self.mirror_az.shutdown()
        if self.mirror_alt:
            self.mirror_alt.shutdown()
        for viewer in self._cameras.values():
            viewer.close()

    # ── display refresh ───────────────────────────────────────────────────────

    def _refresh_display(self) -> None:
        self._update_status()
        self._update_hardware()

    def _update_status(self) -> None:
        state = self.sm.state if self.sm else State.FAULT
        s_style, s_label = _STATE_STYLE.get(state, ('white', state.value.upper()))

        if self.anyloop and self.anyloop.running:
            if self._loop_started_at:
                elapsed = int((datetime.now() - self._loop_started_at).total_seconds())
                h, rem = divmod(elapsed, 3600)
                m, s_  = divmod(rem, 60)
                uptime = f'  up {h:02d}:{m:02d}:{s_:02d}'
            else:
                uptime = ''
            loop_part = f'[green]RUNNING[/green] pid={self.anyloop.pid}{uptime}'
        else:
            loop_part = '[dim]stopped[/dim]'

        self.query_one('#status', Static).update(
            f'State: [{s_style}]{s_label}[/{s_style}]    '
            f'Loop: {loop_part}'
        )

    def _update_hardware(self) -> None:
        lines = []

        # Laser — power bar
        mw = self.laser.power_mw if self.laser else 0.0
        enabled = self.laser.is_enabled if self.laser else False
        bar_len = 20
        filled = int(round(mw / MCLS1_MAX_POWER_MW * bar_len))
        filled = max(0, min(bar_len, filled))
        bar = '█' * filled + '░' * (bar_len - filled)
        if self._laser_pending:
            lines.append(f'[dim]\\[l][/dim] Laser   [yellow]{bar}  …[/yellow]')
        elif enabled:
            lines.append(f'[dim]\\[l][/dim] Laser   [green]{bar}[/green]  {mw:.1f} mW')
        else:
            lines.append(f'[dim]\\[l][/dim] Laser   [dim]{bar}  OFF[/dim]')

        # Servo shutter
        if self.shutter:
            sh = '[green]OPEN[/green]' if self.shutter.is_open else '[dim]CLOSED[/dim]'
        else:
            sh = '[dim]—[/dim]'
        lines.append(f'[dim]\\[o/c][/dim] Shutter  {sh}')

        # Retro (pulse-toggled, no readable state)
        lines.append('[dim]\\[r][/dim] Retro    [dim]—[/dim]')

        # Steering mirror axes
        az_step  = self.mirror_az.step_size  if self.mirror_az  else None
        alt_step = self.mirror_alt.step_size if self.mirror_alt else None
        az_str  = f'[dim]step {az_step}[/dim]'  if az_step  is not None else '[dim]—[/dim]'
        alt_str = f'[dim]step {alt_step}[/dim]' if alt_step is not None else '[dim]—[/dim]'
        lines.append(f'[dim]\\[←→][/dim] Mirror Az   {az_str}')
        lines.append(f'[dim]\\[↑↓][/dim] Mirror Alt  {alt_str}')

        # Camera viewers — numbered 1/2/3 to match keybindings
        for idx, viewer in self._cameras.items():
            cam_num = idx + 1
            if viewer.is_open:
                url = f'http://localhost:{viewer.http_port}/'
                exp_str = f'[dim]{viewer.exposure_us} µs[/dim]'
                cam_str = f'[green]OPEN[/green]  [dim]\\[e][/dim]xposure = {exp_str}  [link="{url}"]{url}[/link]'
            else:
                cam_str = '[dim]NOT OPEN[/dim]'
            lines.append(
                f'[dim]\\[{cam_num}][/dim] Cam {cam_num}  {viewer.name}  {cam_str}'
            )

        self.query_one('#hardware', Static).update('\n'.join(lines))

    # ── log ───────────────────────────────────────────────────────────────────

    def _append_log(self, msg: str, style: str) -> None:
        widget = self.query_one('#log', RichLog)
        if style:
            widget.write(f'[{style}]{msg}[/{style}]')
        else:
            widget.write(msg)

    # ── key actions ───────────────────────────────────────────────────────────

    def action_start_loop(self) -> None:
        if not self.anyloop:
            log.error('anyloop not initialised')
            return
        try:
            self.anyloop.start(self._config)
            self._loop_started_at = datetime.now()
            log.info('anyloop started (pid %d)', self.anyloop.pid)
        except Exception as exc:
            log.error('start: %s', exc)
        self._refresh_display()

    def action_stop_loop(self) -> None:
        if not self.anyloop:
            return
        self.anyloop.stop()
        self._loop_started_at = None
        log.info('anyloop stopped')
        self._refresh_display()

    def action_open_shutter(self) -> None:
        if not self.shutter:
            log.error('shutter not initialised')
            return
        self.shutter.open()
        log.info('Shutter → open')
        self._refresh_display()

    def action_close_shutter(self) -> None:
        if not self.shutter:
            log.error('shutter not initialised')
            return
        self.shutter.close()
        log.info('Shutter → closed')
        self._refresh_display()

    def action_trigger_retro(self) -> None:
        if not self.retro:
            log.error('retro not initialised')
            return
        self.retro.trigger()
        log.info('Retro triggered')

    def action_mirror_az(self, direction: int) -> None:
        if not self.mirror_az:
            log.error('Mirror Az (Tic) not initialised')
            return
        threading.Thread(
            target=self.mirror_az.step, args=(direction,), daemon=True
        ).start()

    def action_mirror_alt(self, direction: int) -> None:
        if not self.mirror_alt:
            log.error('Mirror Alt (KDC101) not initialised')
            return
        threading.Thread(
            target=self.mirror_alt.step, args=(direction,), daemon=True
        ).start()

    def action_mirror_step_finer(self) -> None:
        if self.mirror_az:
            self.mirror_az.step_size = self.mirror_az.step_size * 2
        if self.mirror_alt:
            self.mirror_alt.step_size = self.mirror_alt.step_size * 2
        self._refresh_display()

    def action_mirror_step_coarser(self) -> None:
        if self.mirror_az:
            self.mirror_az.step_size = max(1, self.mirror_az.step_size // 2)
        if self.mirror_alt:
            self.mirror_alt.step_size = max(1, self.mirror_alt.step_size // 2)
        self._refresh_display()

    def action_set_laser(self) -> None:
        if not self.laser:
            log.error('laser not initialised')
            return

        def apply(value: str | None) -> None:
            if not value:
                return
            self._laser_pending = True
            self._refresh_display()
            def _run():
                if value.lower() == 'off':
                    self.laser.off()
                    log.info('Laser off')
                else:
                    try:
                        self.laser.set_power(float(value))
                        log.info('Laser → %.3f mW', self.laser.power_mw)
                    except ValueError:
                        log.error('Invalid power: %r', value)
                self._laser_pending = False
                self.call_from_thread(self._refresh_display)
            threading.Thread(target=_run, daemon=True, name='laser-set').start()

        self.push_screen(LaserModal(), apply)

    def action_camera_toggle(self, idx: int) -> None:
        viewer = self._cameras.get(idx)
        if not viewer:
            return
        if viewer.is_open:
            viewer.close()
            log.info('Camera %d (%s) closed', idx, viewer.name)
        else:
            if idx == 1 and self.anyloop and self.anyloop.running:
                log.warning('ASI662MM (cam 1) in use by steering loop; stop the loop first')
                return
            try:
                viewer.open()
                log.info('Camera %d (%s) stream at http://localhost:%d/',
                         idx, viewer.name, viewer.http_port)
            except Exception as exc:
                log.error('Camera %d open failed: %s', idx, exc)
        self._refresh_display()

    def action_set_exposure(self) -> None:
        def apply(value: str | None) -> None:
            if not value:
                return
            parts = value.split()
            if len(parts) != 2:
                log.error('Exposure: expected "slot exposure_µs", got %r', value)
                return
            try:
                idx = int(parts[0])
                us  = int(parts[1])
            except ValueError:
                log.error('Exposure: invalid input %r', value)
                return
            viewer = self._cameras.get(idx)
            if not viewer:
                log.error('Exposure: unknown camera slot %d', idx)
                return
            def _run():
                try:
                    viewer.set_exposure(us)
                except Exception as exc:
                    log.error('Exposure set failed: %s', exc)
                self.call_from_thread(self._refresh_display)
            threading.Thread(target=_run, daemon=True, name='exposure-set').start()

        self.push_screen(ExposureModal(self._cameras), apply)

    def action_abort(self) -> None:
        if not self.sm:
            return
        self.sm.abort()
        self._loop_started_at = None
        log.warning('ABORT')
        self._refresh_display()

    # ── background state-machine stepper ─────────────────────────────────────

    def _sm_loop(self) -> None:
        while True:
            try:
                if self.sm:
                    self.sm.step()
            except Exception:
                log.exception('State machine error')
            time.sleep(0.1)
