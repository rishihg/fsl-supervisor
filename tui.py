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
from textual.containers import Horizontal, Vertical
from textual.screen import ModalScreen
from textual.widgets import Footer, Header, Input, Label, RichLog, Static

from anyloop_manager import AnyloopProcess, TelemetryReceiver, CommandSender
from hardware import DAQC2Board, RetroShutter, Shutter, MCLS1, MCLS1Channel, TicAxis, KDC101Axis
from alignment import AlignmentStateMachine, State
from supervisor import (
    ANYLOOP_BINARY, ANYLOOP_CONFIG,
    TELEMETRY_PORT, COMMAND_PORT, N_COMMAND,
    DAQC2_ADDR, SHUTTER_BIT,
    SHUTTER_PORT, SHUTTER_OPEN_PW_US, SHUTTER_CLOSE_PW_US,
    MCLS1_PORT, MCLS1_CHANNEL, MCLS1_CURRENT_CAL, MCLS1_MAX_POWER_MW,
    MCLS1_CHANNEL_3, MCLS1_CURRENT_CAL_3, MCLS1_MAX_POWER_MW_3,
    TIC_SERIAL, TIC_STEP_SIZE, KDC101_PORT, KDC101_CHANNEL, KDC101_STEP_SIZE,
    FIBER_TIC_SERIALS, FIBER_TIC_STEP_SIZE,
    CameraViewer, CAMERA_CONFIGS,
)

log = logging.getLogger(__name__)

_FIBER_LABEL = {'x': 'x', 'y': 'y', 'z': 'z', 'theta': 'θ', 'phi': 'φ'}
_FIBER_AXES  = list(FIBER_TIC_SERIALS.keys())

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


# ── laser modal ───────────────────────────────────────────────────────────────

class LaserModal(ModalScreen):
    """Floating dialog for setting power on any laser channel."""

    DEFAULT_CSS = """
    LaserModal {
        align: center middle;
    }
    LaserModal > Vertical {
        width: 54;
        height: auto;
        border: round $primary;
        background: $surface;
        padding: 1 2;
    }
    LaserModal Label {
        margin-bottom: 0;
    }
    LaserModal #laser-header {
        margin-bottom: 1;
    }
    LaserModal Input {
        margin-top: 1;
    }
    """

    BINDINGS = [Binding('escape', 'cancel', 'Cancel')]

    def __init__(self, lasers: list[tuple[int, str, float, bool, float, float]]) -> None:
        super().__init__()
        self._lasers = lasers  # [(channel, name, power_mw, is_enabled, min_mw, max_mw), ...]

    def compose(self) -> ComposeResult:
        with Vertical():
            yield Label('[dim]channel  power_mw   (or "off")[/dim]',
                        id='laser-header')
            for ch, name, mw, enabled, min_mw, max_mw in self._lasers:
                state = f'{mw:.2f} mW  [green]on[/green]' if enabled else f'{mw:.2f} mW  [dim]off[/dim]'
                yield Label(f'  [dim]{ch}[/dim]  {name:<10}  {state}  [dim]({min_mw}–{max_mw} mW)[/dim]')
            yield Input(placeholder='channel  power_mw', id='laser-input')

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
            yield Label('[dim]cam  exposure_µs   (e.g. 1 20000)[/dim]',
                        id='exp-header')
            for idx, viewer in sorted(self._cameras.items()):
                exp_str = f'{viewer.exposure_us} µs' if viewer.is_open else '[dim]closed[/dim]'
                yield Label(f'  [dim]{idx + 1}[/dim]  {viewer.name:<12}  {exp_str}')
            yield Input(placeholder='cam  exposure_µs', id='exp-input')

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

    #hardware-panel {
        height: auto;
        margin: 1 1 0 1;
    }

    #hardware-left {
        width: 1fr;
        height: auto;
    }

    #hw-lasers, #hw-shutters, #hw-cameras {
        border: round $primary;
        padding: 0 2;
        height: auto;
        margin: 0 1 1 0;
    }

    #hardware-right {
        width: 34;
        height: auto;
        border: round $primary;
        padding: 0 2;
        margin: 0 0 1 0;
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
        Binding('r', 'trigger_retroreflector', 'Retroreflector', show=False),
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
        # Fiber stage
        Binding('f',   'fiber_cycle',        'Fiber axis', show=False),
        Binding('=',   'fiber_step(1)',      'Fiber+',     show=False),
        Binding('-',   'fiber_step(-1)',     'Fiber−',     show=False),
        Binding('comma',      'fiber_step_coarser', 'FiberStep÷2', show=False),
        Binding('full_stop',  'fiber_step_finer',   'FiberStep×2', show=False),
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
        self.retroreflector:      RetroShutter | None         = None
        self.shutter:    Shutter | None              = None
        self.laser:      MCLS1 | None                = None
        self.laser2:     MCLS1Channel | None         = None
        self.mirror_az:  TicAxis | None              = None
        self.mirror_alt: KDC101Axis | None           = None
        self.sm:         AlignmentStateMachine | None = None
        self._cameras: dict[int, CameraViewer] = {}
        self._laser_pending:    bool = False
        self._laser2_pending:   bool = False
        self._mirror_highlight: str | None = None
        self.fiber_axes: dict[str, TicAxis | None] = {k: None for k in _FIBER_AXES}
        self._fiber_active:    str       = _FIBER_AXES[0]
        self._fiber_highlight: str | None = None

    # ── layout ───────────────────────────────────────────────────────────────

    def compose(self) -> ComposeResult:
        yield Header()
        yield Static('', id='status')
        with Horizontal(id='hardware-panel'):
            with Vertical(id='hardware-left'):
                yield Static('', id='hw-lasers',   markup=True)
                yield Static('', id='hw-shutters', markup=True)
                yield Static('', id='hw-cameras',  markup=True)
            yield Static('', id='hardware-right', markup=True)
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
            self.retroreflector     = RetroShutter(self.board, bit=SHUTTER_BIT)
            self.shutter   = Shutter(SHUTTER_PORT,
                                     open_pw_us=SHUTTER_OPEN_PW_US,
                                     close_pw_us=SHUTTER_CLOSE_PW_US)
            self.laser     = MCLS1(MCLS1_PORT, channel=MCLS1_CHANNEL,
                                   current_cal=MCLS1_CURRENT_CAL)
            self.laser2    = MCLS1Channel(self.laser, channel=MCLS1_CHANNEL_3,
                                          current_cal=MCLS1_CURRENT_CAL_3)
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

        for axis, serial in FIBER_TIC_SERIALS.items():
            if not serial:
                continue
            try:
                self.fiber_axes[axis] = TicAxis(step_size=FIBER_TIC_STEP_SIZE,
                                                serial_number=serial)
            except Exception:
                log.exception('Fiber stage %s (Tic serial=%s) init failed', axis, serial)

        try:
            self.sm = AlignmentStateMachine(
                anyloop   = self.anyloop,
                commander = self.commander,
                shutter   = self.retroreflector,
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
        if self.laser2:
            self.laser2.shutdown()
        if self.laser:
            self.laser.shutdown()
        if self.mirror_az:
            self.mirror_az.shutdown()
        if self.mirror_alt:
            self.mirror_alt.shutdown()
        for ax in self.fiber_axes.values():
            if ax:
                ax.shutdown()
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

        # ── lasers box ────────────────────────────────────────────────────────
        def _laser_line(laser, pending, max_mw, label):
            mw      = laser.power_mw  if laser else 0.0
            enabled = laser.is_enabled if laser else False
            bar_len = 20
            filled  = max(0, min(bar_len, int(round(mw / max_mw * bar_len))))
            bar     = '█' * filled + '░' * (bar_len - filled)
            if pending:
                return f'{label}  [yellow]{bar}  …[/yellow]'
            elif enabled:
                return f'{label}  [green]{bar}[/green]  {mw:.1f} mW'
            else:
                return f'{label}  [dim]{bar}  OFF[/dim]'

        lasers = [
            '[dim]\\[l][/dim] Lasers:',
            '  ' + _laser_line(self.laser,  self._laser_pending,
                               MCLS1_MAX_POWER_MW,   'MCLS ch. 2 (635nm)'),
            '  ' + _laser_line(self.laser2, self._laser2_pending,
                               MCLS1_MAX_POWER_MW_3, 'MCLS ch. 3 (785nm)'),
        ]
        self.query_one('#hw-lasers', Static).update('\n'.join(lasers))

        # ── shutters box ──────────────────────────────────────────────────────
        if self.shutter:
            sh = '[green]OPEN[/green]' if self.shutter.is_open else '[dim]CLOSED[/dim]'
        else:
            sh = '[dim]—[/dim]'
        shutters = [
            f'[dim]\\[o/c][/dim] Shutter         {sh}',
            '[dim]\\[r][/dim] Retroreflector  [dim]—[/dim]',
        ]
        self.query_one('#hw-shutters', Static).update('\n'.join(shutters))

        # ── cameras box ───────────────────────────────────────────────────────
        cameras = []
        for idx, viewer in self._cameras.items():
            cam_num = idx + 1
            if cameras:
                cameras.append('')  # blank line between camera entries
            open_str = '[green]OPEN[/green]' if viewer.is_open else '[dim]NOT OPEN[/dim]'
            cameras.append(f'[dim]\\[{cam_num}][/dim] Cam {cam_num}  {viewer.name}  {open_str}')
            url = f'http://localhost:{viewer.http_port}/'
            if viewer.is_open:
                cameras.append(f'     [link="{url}"]{url}[/link]')
                cameras.append(f'     [dim]\\[e][/dim]xposure = {viewer.exposure_us} µs')
            else:
                cameras.append(f'     [dim]{url}[/dim]')
                cameras.append(f'     [dim]\\[e][/dim][dim]xposure = {viewer.exposure_us} µs[/dim]')
        self.query_one('#hw-cameras', Static).update('\n'.join(cameras))

        # ── right panel — mirror control cross ────────────────────────────────
        az_step  = self.mirror_az.step_size  if self.mirror_az  else None
        alt_step = self.mirror_alt.step_size if self.mirror_alt else None
        az_str  = str(az_step)  if az_step  is not None else '—'
        alt_str = str(alt_step) if alt_step is not None else '—'

        h = self._mirror_highlight

        def _k(key, active: bool) -> str:
            """Key hint bracket — bold green when active, dim otherwise."""
            if active:
                return f'[bold green]\\[{key}][/bold green]'
            return f'[dim]\\[{key}][/dim]'

        def _t(text: str, active: bool) -> str:
            """Plain text — bold green when active, unchanged otherwise."""
            return f'[bold green]{text}[/bold green]' if active else text

        up      = h == 'up'
        down    = h == 'down'
        left    = h == 'left'
        right_  = h == 'right'
        coarser = h == 'coarser'
        finer   = h == 'finer'
        step_hl = coarser or finer

        step_style = 'bold green' if step_hl else 'dim'

        # ── fiber stage section ───────────────────────────────────────────────
        fiber_lines: list[str] = [
            '',
            '─' * 28,
            ' Fiber Stage',
            '',
            f' {_k("f", False)} cycle   {_k("=", False)}+  {_k("-", False)}−',
            f' {_k(",", False)}÷2  step  ×2{_k(".", False)}',
            '',
        ]
        for ax in _FIBER_AXES:
            tobj  = self.fiber_axes.get(ax)
            label = _FIBER_LABEL[ax]
            active = ax == self._fiber_active
            flash  = self._fiber_highlight == ax
            if active:
                step_val = str(tobj.step_size) if tobj else '—'
                line = (f' [bold green]▶ {label}[/bold green]'
                        f'  [dim]step:[/dim] {step_val}')
            else:
                avail = '' if tobj else ' [dim](—)[/dim]'
                line = f'   {label}{avail}'
            if flash:
                line = f'[bold green]{line}[/bold green]'
            fiber_lines.append(line)

        right = '\n'.join([
            ' Optical Steering',
            '',
            _t('            ↑', up),
            f'         {_k("↑", up)} {_t("alt+", up)}',
            _t('            │', up),
            f' {_t("az−", left)} {_k("←", left)} {_t("───", left)}┼{_t("───", right_)} {_k("→", right_)} {_t("az+", right_)}',
            _t('            │', down),
            f'         {_k("↓", down)} {_t("alt−", down)}',
            _t('            ↓', down),
            '',
            f' az  step: [{step_style}]{az_str}[/{step_style}]',
            f' alt step: [{step_style}]{alt_str}[/{step_style}]',
            '',
            f' {_k("[", coarser)} {_t("÷2", coarser)}   step   {_t("×2", finer)} {_k("]", finer)}',
        ] + fiber_lines)
        self.query_one('#hardware-right', Static).update(right)

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

    def action_trigger_retroreflector(self) -> None:
        if not self.retroreflector:
            log.error('retroreflector not initialised')
            return
        self.retroreflector.trigger()
        log.info('Retroreflector triggered')

    def _flash_mirror(self, key: str, duration: float = 0.15) -> None:
        self._mirror_highlight = key
        self._refresh_display()
        self.set_timer(duration, self._clear_mirror_highlight)

    def _clear_mirror_highlight(self) -> None:
        self._mirror_highlight = None
        self._refresh_display()

    def action_mirror_az(self, direction: int) -> None:
        if not self.mirror_az:
            log.error('Mirror Az (Tic) not initialised')
            return
        self._flash_mirror('left' if direction < 0 else 'right')
        threading.Thread(
            target=self.mirror_az.step, args=(direction,), daemon=True
        ).start()

    def action_mirror_alt(self, direction: int) -> None:
        if not self.mirror_alt:
            log.error('Mirror Alt (KDC101) not initialised')
            return
        self._flash_mirror('up' if direction > 0 else 'down')
        threading.Thread(
            target=self.mirror_alt.step, args=(direction,), daemon=True
        ).start()

    def action_mirror_step_finer(self) -> None:
        if self.mirror_az:
            self.mirror_az.step_size = self.mirror_az.step_size * 2
        if self.mirror_alt:
            self.mirror_alt.step_size = self.mirror_alt.step_size * 2
        self._flash_mirror('finer', duration=0.3)

    def action_mirror_step_coarser(self) -> None:
        if self.mirror_az:
            self.mirror_az.step_size = max(1, self.mirror_az.step_size // 2)
        if self.mirror_alt:
            self.mirror_alt.step_size = max(1, self.mirror_alt.step_size // 2)
        self._flash_mirror('coarser', duration=0.3)

    # ── fiber stage actions ───────────────────────────────────────────────────

    def _flash_fiber(self, axis: str, duration: float = 0.15) -> None:
        self._fiber_highlight = axis
        self._refresh_display()
        self.set_timer(duration, self._clear_fiber_highlight)

    def _clear_fiber_highlight(self) -> None:
        self._fiber_highlight = None
        self._refresh_display()

    def action_fiber_cycle(self) -> None:
        idx = _FIBER_AXES.index(self._fiber_active)
        self._fiber_active = _FIBER_AXES[(idx + 1) % len(_FIBER_AXES)]
        self._refresh_display()

    def action_fiber_step(self, direction: int) -> None:
        ax = self._fiber_active
        tobj = self.fiber_axes.get(ax)
        if not tobj:
            log.error('Fiber axis %s not initialised', ax)
            return
        self._flash_fiber(ax)
        threading.Thread(target=tobj.step, args=(direction,), daemon=True).start()

    def action_fiber_step_finer(self) -> None:
        tobj = self.fiber_axes.get(self._fiber_active)
        if tobj:
            tobj.step_size = tobj.step_size * 2
        self._flash_fiber(self._fiber_active, duration=0.3)

    def action_fiber_step_coarser(self) -> None:
        tobj = self.fiber_axes.get(self._fiber_active)
        if tobj:
            tobj.step_size = max(1, tobj.step_size // 2)
        self._flash_fiber(self._fiber_active, duration=0.3)

    def action_set_laser(self) -> None:
        channel_map = {}
        lasers_info = []
        if self.laser:
            channel_map[MCLS1_CHANNEL]   = (self.laser,  '_laser_pending')
            lasers_info.append((MCLS1_CHANNEL,   'MCLS ch.2 635nm',
                                self.laser.power_mw,  self.laser.is_enabled,
                                MCLS1_CURRENT_CAL[2], MCLS1_CURRENT_CAL[3]))
        if self.laser2:
            channel_map[MCLS1_CHANNEL_3] = (self.laser2, '_laser2_pending')
            lasers_info.append((MCLS1_CHANNEL_3, 'MCLS ch.3 785nm',
                                self.laser2.power_mw, self.laser2.is_enabled,
                                MCLS1_CURRENT_CAL_3[2], MCLS1_CURRENT_CAL_3[3]))
        if not channel_map:
            log.error('no lasers initialised')
            return

        def apply(value: str | None) -> None:
            if not value:
                return
            parts = value.split()
            if len(parts) != 2:
                log.error('Laser: expected "channel power_mw", got %r', value)
                return
            try:
                ch = int(parts[0])
            except ValueError:
                log.error('Laser: invalid channel %r', parts[0])
                return
            if ch not in channel_map:
                log.error('Laser: unknown channel %d', ch)
                return
            laser, pending_attr = channel_map[ch]
            setattr(self, pending_attr, True)
            self._refresh_display()
            def _run():
                if parts[1].lower() == 'off':
                    laser.off()
                    log.info('Laser ch%d off', ch)
                else:
                    try:
                        laser.set_power(float(parts[1]))
                        log.info('Laser ch%d → %.3f mW', ch, laser.power_mw)
                    except ValueError:
                        log.error('Invalid power: %r', parts[1])
                setattr(self, pending_attr, False)
                self.call_from_thread(self._refresh_display)
            threading.Thread(target=_run, daemon=True,
                             name=f'laser{ch}-set').start()

        self.push_screen(LaserModal(lasers_info), apply)

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
                cam_num = int(parts[0])
                us      = int(parts[1])
            except ValueError:
                log.error('Exposure: invalid input %r', value)
                return
            idx = cam_num - 1
            viewer = self._cameras.get(idx)
            if not viewer:
                log.error('Exposure: unknown camera %d (expected 1–%d)', cam_num, len(self._cameras))
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
