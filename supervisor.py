#!/home/fsl/supervisor/.venv/bin/python3
"""
supervisor.py — operator interface for the optical alignment supervisor.

Run with:
    python supervisor.py [--config /path/to/steering.json]

Commands (type 'help' at the prompt):
    start / stop            Start or stop the anyloop control loop.
    setpoint <y> <x>        Send a setpoint command (normalised ±1 units).
    retro trigger           Trigger the retroreflector cover (toggles state).
    laser <V> | off         Set laser intensity or turn off.
    camera open/close <n>   Open or close a live camera view window (0=ASI662, 1=ASI290).
    align                   Run the automated alignment sequence.
    fiber_couple            Advance to fiber-coupling step.
    lock                    Advance to loop lock-acquire step.
    status                  Print current system status.
    abort                   Emergency stop (triggers retro, kills anyloop).
    quit / EOF              Shut down cleanly.
"""

import argparse
import cmd
import json
import logging
import select
import subprocess
import sys
import threading
import time

from anyloop_manager import AnyloopProcess, TelemetryReceiver, CommandSender
from hardware import DAQC2Board, RetroShutter, Shutter, MCLS1, TicAxis, KDC101Axis
from alignment import AlignmentStateMachine, State

# ── defaults (edit here or override on the command line) ───────────────────
ANYLOOP_BINARY  = '/home/fsl/bin/anyloop'
ANYLOOP_CONFIG  = '/home/fsl/steering.json'

# anyloop udp_sink port for CoM telemetry (must match steering.json)
TELEMETRY_PORT  = 64731

# anyloop udp_source port for supervisor commands (add udp_source to your
# pipeline config with this port before using 'setpoint' or 'lock')
COMMAND_PORT    = 64732
N_COMMAND       = 2   # [y, x] setpoint vector

# DAQC2 board address (0 = first board)
DAQC2_ADDR      = 0

# Retroreflector cover (Thorlabs pulse-triggered): DOUT bit on DAQC2
SHUTTER_BIT     = 0

# MCLS1: Thorlabs 4-channel fiber-coupled laser source over USB serial
MCLS1_PORT        = '/dev/supervisor/mcls1'
MCLS1_CHANNEL     = 2
# Measured calibration: front-panel current range → output power range.
# Used to drive the laser via current= instead of power= (the unit ignores
# power= and is current-controlled from the front panel).
MCLS1_CURRENT_CAL  = (43.0, 66.0, 0.32, 3.1)  # (min_mA, max_mA, min_mW, max_mW)
MCLS1_MAX_POWER_MW = 3.1

# Shutter: Arduino Leonardo on USB serial
SHUTTER_PORT        = '/dev/supervisor/shutter'
SHUTTER_OPEN_PW_US  = 2000
SHUTTER_CLOSE_PW_US = 1000

# Steering mirror: Tic T834 azimuth (USB, serial-locked) + KDC101 altitude
TIC_SERIAL        = '00485172'      # set to e.g. '00123456' to pin to a specific unit
TIC_STEP_SIZE     = 5         # microsteps per keypress — tune to taste
KDC101_PORT       = '/dev/supervisor/kdc101'
KDC101_CHANNEL    = 1
KDC101_STEP_SIZE  = 500       # encoder counts per keypress — tune to taste

# Camera viewers: open-loop anyloop configs + MJPEG viewer script
# ASI SDK enumeration on this machine: index 0 = ASI290MM, index 1 = ASI662MM
# Note: slot 0 (ASI662MM) shares hardware with the steering loop;
#       open only when the loop is stopped.
# Frames served as MJPEG at http://localhost:<http_port>/
# (forward with:  ssh -L <http_port>:localhost:<http_port> <host>)
VIEWER_SCRIPT  = '/home/fsl/supervisor/camera_view.py'
CAMERA_CONFIGS = {
    #  slot: (config,                           udp_port, name,       http_port)
    0: ('/home/fsl/camera1_asi290.json',  64741, 'CONFOCAL',  8290),
    1: ('/home/fsl/camera2_asi662.json',  64740, 'STEERING',  8662),
    2: ('/home/fsl/camera3_asi662b.json', 64742, 'FIBER',     8663),
}

# ── logging ─────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s  %(levelname)-8s  %(name)s: %(message)s',
    datefmt='%H:%M:%S',
    stream=sys.stdout,
)
log = logging.getLogger('supervisor')


class CameraViewer:
    """Manages one open-loop anyloop instance and an MJPEG HTTP viewer.

    Args:
        cam_index:  Supervisor slot index (0 = ASI662MM, 1 = ASI290MM).
        config:     Path to the camera-only anyloop JSON config.
        port:       UDP port the anyloop udp_sink sends frames to.
        name:       Human-readable camera name.
        http_port:  Port the MJPEG server listens on.
    """

    def __init__(self, cam_index: int, config: str, port: int, name: str,
                 http_port: int):
        self.cam_index   = cam_index
        self.config      = config
        self.port        = port
        self.name        = name
        self.http_port   = http_port
        self.exposure_us = self._read_exposure_us()
        self._anyloop    = AnyloopProcess(ANYLOOP_BINARY)
        self._viewer: subprocess.Popen | None = None

    def _read_exposure_us(self) -> int:
        """Read exposure_us from the config JSON (returns 10000 on any failure)."""
        try:
            with open(self.config) as f:
                cfg = json.load(f)
            for device in cfg.get('pipeline', []):
                if 'aylp_asi' in device.get('uri', ''):
                    return int(device.get('params', {}).get('exposure_us', 10000))
        except Exception:
            pass
        return 10000

    def set_exposure(self, exposure_us: int):
        """Patch exposure_us in the config JSON and restart the viewer if open."""
        with open(self.config) as f:
            cfg = json.load(f)
        for device in cfg.get('pipeline', []):
            if 'aylp_asi' in device.get('uri', ''):
                device['params']['exposure_us'] = exposure_us
        with open(self.config, 'w') as f:
            json.dump(cfg, f, indent=4)
        self.exposure_us = exposure_us
        log.info('Camera %d (%s): exposure → %d µs', self.cam_index, self.name, exposure_us)
        if self.is_open:
            self.close()
            self.open()

    def _evict_port(self, port: int):
        """Kill any process already listening on a TCP port (stale viewers)."""
        import signal as _signal
        result = subprocess.run(
            ['ss', '-tlnpH', f'sport = :{port}'],
            capture_output=True, text=True,
        )
        for line in result.stdout.splitlines():
            m = __import__('re').search(r'pid=(\d+)', line)
            if m:
                pid = int(m.group(1))
                try:
                    __import__('os').kill(pid, _signal.SIGTERM)
                    log.info('Camera %d: evicted stale viewer pid=%d on port %d',
                             self.cam_index, pid, port)
                except ProcessLookupError:
                    pass
        # give the evicted process a moment to release the port
        if result.stdout.strip():
            time.sleep(0.5)

    def open(self):
        # Evict any stale viewer process that survived a previous supervisor
        # session and is still holding our HTTP port.
        self._evict_port(self.http_port)
        # Start the viewer and wait for it to confirm its UDP socket is bound
        # before starting anyloop.  The viewer prints "READY\n" to stdout
        # after sock.bind().  stderr is inherited so errors appear in the
        # supervisor output.
        self._viewer = subprocess.Popen(
            [sys.executable, VIEWER_SCRIPT,
             str(self.port), str(self.http_port), self.name],
            stdout=subprocess.PIPE,
            stderr=None,   # inherit supervisor stderr so viewer crashes are visible
        )
        readable, _, _ = select.select([self._viewer.stdout], [], [], 10.0)
        if not readable:
            self._viewer.terminate()
            raise RuntimeError(
                f'camera {self.cam_index} viewer did not signal ready within 10 s'
            )
        line = self._viewer.stdout.readline().strip()
        if line != b'READY':
            self._viewer.terminate()
            raise RuntimeError(
                f'camera {self.cam_index} viewer failed to start (got {line!r})'
            )
        self._anyloop.start(self.config)
        log.info('Camera %d (%s) stream at http://localhost:%d/',
                 self.cam_index, self.name, self.http_port)

    def close(self):
        if self._viewer and self._viewer.poll() is None:
            self._viewer.terminate()
        self._viewer = None
        self._anyloop.stop()
        log.info('Camera %d (%s) viewer closed', self.cam_index, self.name)

    @property
    def is_open(self) -> bool:
        return self._anyloop.running


class SupervisorShell(cmd.Cmd):
    intro  = (
        '\n  FSL supervisor\n'
        '  Type  help  for a list of commands.\n'
    )
    prompt = '(supervisor) '

    def __init__(self, config: str):
        super().__init__()
        self.config = config

        # ── anyloop IPC ──────────────────────────────────────────────────────
        self.anyloop   = AnyloopProcess(ANYLOOP_BINARY)
        self.telemetry = TelemetryReceiver(TELEMETRY_PORT)
        self.commander = CommandSender(COMMAND_PORT, N_COMMAND)

        # ── hardware ─────────────────────────────────────────────────────────
        self.board         = DAQC2Board(addr=DAQC2_ADDR)
        self.retro         = RetroShutter(self.board, bit=SHUTTER_BIT)
        self.shutter       = Shutter(SHUTTER_PORT,
                                     open_pw_us=SHUTTER_OPEN_PW_US,
                                     close_pw_us=SHUTTER_CLOSE_PW_US)
        self.laser         = MCLS1(MCLS1_PORT, channel=MCLS1_CHANNEL,
                                    current_cal=MCLS1_CURRENT_CAL)

        # ── camera viewers ───────────────────────────────────────────────────
        self._cameras = {
            idx: CameraViewer(idx, config, port, name, http_port)
            for idx, (config, port, name, http_port) in CAMERA_CONFIGS.items()
        }

        # ── state machine ────────────────────────────────────────────────────
        self.sm = AlignmentStateMachine(
            anyloop   = self.anyloop,
            commander = self.commander,
            shutter   = self.retro,
            laser     = self.laser,
            telemetry = self.telemetry,
            config    = self.config,
        )

        # ── background state-machine stepper ─────────────────────────────────
        self._sm_thread = threading.Thread(target=self._sm_loop, daemon=True,
                                           name='sm-stepper')
        self.telemetry.start()
        self._sm_thread.start()

    # ── anyloop control ──────────────────────────────────────────────────────

    def do_start(self, arg):
        """Start the anyloop control loop.  Usage: start [config_file]"""
        config = arg.strip() or self.config
        try:
            self.anyloop.start(config)
            print(f'anyloop started (pid {self.anyloop.pid})')
        except Exception as e:
            print(f'Error: {e}')

    def do_stop(self, arg):
        """Stop the anyloop control loop."""
        self.anyloop.stop()
        print('anyloop stopped')

    # ── setpoint ─────────────────────────────────────────────────────────────

    def do_setpoint(self, arg):
        """Send a setpoint to anyloop's udp_source.  Usage: setpoint <y> <x>
        Values are normalised to [-1, 1] (same units as CoM output)."""
        parts = arg.split()
        if len(parts) != 2:
            print('Usage: setpoint <y> <x>')
            return
        try:
            y, x = float(parts[0]), float(parts[1])
        except ValueError:
            print('y and x must be numbers')
            return
        self.commander.send(y, x)
        print(f'Setpoint → y={y:.4f}  x={x:.4f}')

    # ── hardware ─────────────────────────────────────────────────────────────

    def do_retro(self, arg):
        """Trigger the retroreflector cover (Thorlabs, pulse-toggled).  Usage: retro trigger"""
        if arg.strip().lower() == 'trigger':
            self.retro.trigger()
            print('Retro triggered')
        else:
            print('Usage: retro trigger')

    def do_shutter(self, arg):
        """Control the shutter (Arduino Leonardo).
        Usage:
          shutter open              move to open position
          shutter close             move to closed position
          shutter pos <pulse_us>    move to arbitrary position (e.g. 1500)
          shutter detach            stop PWM output (eliminates idle jitter)"""
        parts = arg.strip().split()
        if not parts:
            print(self.do_shutter.__doc__)
            return
        if parts[0] == 'open':
            self.shutter.open()
            print(f'Shutter → open ({self.shutter.open_pw_us} µs)')
        elif parts[0] == 'close':
            self.shutter.close()
            print(f'Shutter → close ({self.shutter.close_pw_us} µs)')
        elif parts[0] == 'pos' and len(parts) == 2:
            try:
                pw = int(parts[1])
            except ValueError:
                print('pulse_us must be an integer')
                return
            self.shutter.set_position(pw)
            print(f'Shutter → {pw} µs')
        elif parts[0] == 'detach':
            self.shutter.detach()
            print('Shutter detached')
        else:
            print(self.do_shutter.__doc__)

    def do_camera(self, arg):
        """Open or close a live camera view window.
        Usage:
          camera open <index>   open viewer (0 = ASI662MM, 1 = ASI290MM)
          camera close <index>  close viewer
          camera status         show which cameras are open
        Note: cam 0 (ASI662MM) shares hardware with the steering loop;
              close the loop first with 'stop' before opening cam 0."""
        parts = arg.strip().split()
        if not parts or parts[0] == 'status':
            for idx, viewer in self._cameras.items():
                state = 'open' if viewer.is_open else 'closed'
                print(f'  cam {idx} ({viewer.name}): {state}')
            return
        if len(parts) < 2:
            print(self.do_camera.__doc__)
            return
        try:
            idx = int(parts[1])
        except ValueError:
            print('index must be 0 or 1')
            return
        if idx not in self._cameras:
            print(f'Unknown camera index {idx}. Available: {sorted(self._cameras)}')
            return
        viewer = self._cameras[idx]
        if parts[0] == 'open':
            if viewer.is_open:
                print(f'Camera {idx} ({viewer.name}) is already open')
                return
            if idx == 1 and self.anyloop.running:
                print('Error: ASI662MM (cam 1) is in use by the steering loop. '
                      'Run "stop" first.')
                return
            try:
                viewer.open()
                print(f'Camera {idx} ({viewer.name}) opened — '
                      f'stream at http://localhost:{viewer.http_port}/')
            except Exception as e:
                print(f'Error opening camera {idx}: {e}')
        elif parts[0] == 'close':
            viewer.close()
            print(f'Camera {idx} ({viewer.name}) closed')
        else:
            print(self.do_camera.__doc__)

    def do_laser(self, arg):
        """Set laser power.
        Usage:
          laser <mW>   set output power in milliwatts
          laser off    disable laser output"""
        parts = arg.split()
        if not parts:
            print(self.do_laser.__doc__)
            return
        if parts[0] == 'off':
            self.laser.off()
            print('Laser off')
        elif len(parts) == 1:
            try:
                mw = float(parts[0])
            except ValueError:
                print('Usage: laser <mW> | laser off')
                return
            self.laser.set_power(mw)
            print(f'Laser → {self.laser.power_mw:.3f} mW (enabled)')
        else:
            print(self.do_laser.__doc__)

    # ── alignment sequence ───────────────────────────────────────────────────

    def do_align(self, arg):
        """Start the automated alignment sequence from IDLE."""
        try:
            self.sm.start_sequence()
            print(f'Sequence started. State: {self.sm.state.value}')
        except (RuntimeError, NotImplementedError) as e:
            print(f'Error: {e}')

    def do_fiber_couple(self, arg):
        """Advance to the fiber-coupling step (from coarse_align state)."""
        if self.sm.state != State.COARSE_ALIGN:
            print(f'Cannot enter fiber_couple from state {self.sm.state.value}')
            return
        try:
            self.sm._to(State.FIBER_COUPLE)
            self.sm._run_fiber_couple()
        except NotImplementedError as e:
            print(f'Not yet implemented: {e}')

    def do_lock(self, arg):
        """Advance to loop lock-acquire step (starts anyloop)."""
        if self.sm.state not in (State.IDLE, State.COARSE_ALIGN, State.FIBER_COUPLE):
            print(f'Cannot enter lock_acquire from state {self.sm.state.value}')
            return
        self.sm._to(State.LOCK_ACQUIRE)
        self.sm._run_lock_acquire()
        print('anyloop started; waiting for lock...')

    def do_abort(self, arg):
        """Emergency stop: close shutter, disable laser, kill anyloop."""
        self.sm.abort()
        print('Aborted. State: idle')

    # ── status ────────────────────────────────────────────────────────────────

    def do_status(self, arg):
        """Print current system status."""
        pkt = self.telemetry.get()
        loop_str = (f'running (pid {self.anyloop.pid})'
                    if self.anyloop.running else 'stopped')
        com_str = (f'y={pkt.data[0]:+.4f}  x={pkt.data[1]:+.4f}'
                   if (pkt and len(pkt.data) >= 2) else 'no telemetry')
        print(
            f'  state    : {self.sm.state.value}\n'
            f'  anyloop  : {loop_str}\n'
            f'  CoM      : {com_str}\n'
            f'  retro    : state unknown (pulse-triggered)\n'
            f'  laser    : {self.laser.power_mw:.3f} mW'
            + (' (enabled)' if self.laser.is_enabled else ' (disabled)')
        )

    # ── quit ─────────────────────────────────────────────────────────────────

    def do_quit(self, arg):
        """Shut down the supervisor cleanly."""
        return self._shutdown()

    def do_EOF(self, arg):
        print()
        return self._shutdown()

    def _shutdown(self):
        print('Shutting down...')
        self.sm.abort()
        for viewer in self._cameras.values():
            viewer.close()
        self.telemetry.stop()
        self.commander.close()
        self.shutter.shutdown()
        self.laser.shutdown()
        return True

    # ── background state-machine loop ────────────────────────────────────────

    def _sm_loop(self):
        """Advance the state machine every 100 ms in a background thread."""
        while True:
            try:
                self.sm.step()
            except Exception:
                log.exception('Exception in state machine step')
            time.sleep(0.1)


# ── entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='FSL supervisor')
    parser.add_argument('--config', default=ANYLOOP_CONFIG,
                        help='anyloop pipeline config JSON (default: %(default)s)')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug logging')
    parser.add_argument('--shell', action='store_true',
                        help='Use the text shell instead of the TUI')
    args = parser.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    if args.shell:
        shell = SupervisorShell(config=args.config)
        try:
            shell.cmdloop()
        except KeyboardInterrupt:
            print()
            shell._shutdown()
    else:
        from tui import FSLSupervisorApp
        FSLSupervisorApp(config=args.config).run()


if __name__ == '__main__':
    main()
