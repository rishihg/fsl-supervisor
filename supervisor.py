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
    align                   Run the automated alignment sequence.
    fiber_couple            Advance to fiber-coupling step.
    lock                    Advance to loop lock-acquire step.
    status                  Print current system status.
    abort                   Emergency stop (triggers retro, kills anyloop).
    quit / EOF              Shut down cleanly.
"""

import argparse
import cmd
import logging
import sys
import threading
import time

from anyloop_manager import AnyloopProcess, TelemetryReceiver, CommandSender
from hardware import DAQC2Board, RetroShutter, Shutter, Laser
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

# Laser: single 0–5 V modulation input on DAC channel 0
# 0 V = off; ~4 V = max practical intensity (DAQC2 ceiling)
# Characterise dead-zone near 0 V before running automated sequences.
LASER_DAC_CH    = 0

# Shutter: Arduino Leonardo on USB serial
SHUTTER_PORT        = '/dev/ttyACM1'
SHUTTER_OPEN_PW_US  = 2000
SHUTTER_CLOSE_PW_US = 1000

# ── logging ─────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s  %(levelname)-8s  %(name)s: %(message)s',
    datefmt='%H:%M:%S',
    stream=sys.stdout,
)
log = logging.getLogger('supervisor')


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
        self.laser         = Laser(self.board, dac_channel=LASER_DAC_CH)

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

    def do_laser(self, arg):
        """Set laser intensity.
        Usage:
          laser <volts>   set output voltage (0–4.095 V)
          laser off       set output to 0 V"""
        parts = arg.split()
        if not parts:
            print(self.do_laser.__doc__)
            return
        if parts[0] == 'off':
            self.laser.off()
            print('Laser off (0.000 V)')
        elif len(parts) == 1:
            try:
                v = float(parts[0])
            except ValueError:
                print('Usage: laser <volts> | laser off')
                return
            self.laser.set_voltage(v)
            print(f'Laser → {self.laser.voltage:.3f} V')
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
            f'  laser    : {self.laser.voltage:.3f} V'
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
        self.telemetry.stop()
        self.commander.close()
        self.shutter.shutdown()
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
    args = parser.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    shell = SupervisorShell(config=args.config)
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        print()
        shell._shutdown()


if __name__ == '__main__':
    main()
