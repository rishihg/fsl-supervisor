"""
flip_mount.py — Tic T834 flip mount controller.

Uses relative moves so no calibration or absolute position is needed.
The motor moves a fixed number of steps (equivalent to 90 degrees) in
one direction to go OUT, and the same steps back to go IN.

On first use or after power cycle, the supervisor asks the user where
the motor currently is, then tracks state from there.

Configuration:
    STEPS_PER_REV   full steps per revolution of your motor (200 for NEMA8)
    TRAVEL_DEGREES  how many degrees to rotate between IN and OUT
    STEP_MODE       'full' or 'micro' (32 microsteps per full step)
"""

import argparse
import subprocess
import sys
import time

# ── Configuration ─────────────────────────────────────────────────────────────

STEPS_PER_REV  = 400     # measured: 100 steps = 90 deg → 400 steps/rev, 200 steps = 180 deg
TRAVEL_DEGREES = 180     # degrees between IN and OUT (180 so gravity holds at rest)
STEP_MODE      = 'full'  # 'full' or 'micro'

MICROSTEPS_PER_FULL = 32

def _steps_for_degrees(degrees):
    full_steps = round(degrees * STEPS_PER_REV / 360)
    if STEP_MODE == 'micro':
        return full_steps * MICROSTEPS_PER_FULL
    return full_steps

TRAVEL_STEPS = _steps_for_degrees(TRAVEL_DEGREES)
OVERSHOOT    = 20    # steps to overshoot then return, eliminates backlash

# ── Tic helpers ───────────────────────────────────────────────────────────────

def tic(serial, *args):
    result = subprocess.run(
        ['ticcmd', '-d', serial] + list(args),
        capture_output=True, text=True
    )
    if result.returncode != 0:
        print(f'ERROR: {result.stderr.strip()}', file=sys.stderr)
    return result

def get_position(serial):
    result = subprocess.run(
        ['ticcmd', '-d', serial, '--status'],
        capture_output=True, text=True
    )
    for line in result.stdout.splitlines():
        if 'Current position:' in line:
            return int(line.split(':')[1].strip())
    return None


# ── FlipMount class ───────────────────────────────────────────────────────────

class FlipMount:
    """Flip mount controller using relative moves.

    Args:
        serial:     Tic T834 serial number
        is_in:      Current state — True if beam is IN path, False if OUT
        direction:  +1 or -1, sign of steps to go from IN to OUT
    """

    def __init__(self, serial, is_in=True, direction=1):
        self.serial    = serial
        self._is_in    = is_in
        self._direction = direction
        self._pos      = get_position(serial) or 0

    def _prepare(self):
        tic(self.serial, '--reset-command-timeout')
        tic(self.serial, '--resume')
        tic(self.serial, '--exit-safe-start')

    def _deenergize(self):
        tic(self.serial, '--deenergize')

    def _move_relative(self, steps):
        self._prepare()
        target = self._pos + steps
        tic(self.serial, '--position', str(target))
        # Wait for move to complete
        deadline = time.time() + 10.0
        while time.time() < deadline:
            time.sleep(0.1)
            pos = get_position(self.serial)
            if pos is not None and abs(pos - target) <= 2:
                self._pos = pos
                break
        # Deliberately NOT deenergizing here — stay energized (holding
        # torque on) at rest so the mount doesn't sag/drift under gravity
        # between moves. Only deenergize explicitly (e.g. on shutdown) if
        # you want it free-spinning.

    @property
    def is_in(self):
        return self._is_in

    def go_in(self):
        if self._is_in:
            print('Already IN')
            return
        steps = -self._direction * TRAVEL_STEPS
        print(f'Moving IN ({steps:+d} steps)')
        self._move_relative(steps)
        self._is_in = True
        print('Done — flip mount IN')

    def go_out(self):
        if not self._is_in:
            print('Already OUT')
            return
        steps = self._direction * TRAVEL_STEPS
        print(f'Moving OUT ({steps:+d} steps)')
        self._move_relative(steps)
        self._is_in = False
        print('Done — flip mount OUT')

    def toggle(self):
        if self._is_in:
            self.go_out()
        else:
            self.go_in()

    def status(self):
        pos = get_position(self.serial)
        state = 'IN' if self._is_in else 'OUT'
        print(f'Flip mount: {state}  (motor position: {pos})')
        print(f'Travel: {TRAVEL_DEGREES} deg = {TRAVEL_STEPS} steps ({STEP_MODE} step mode)')


# ── Interactive startup ───────────────────────────────────────────────────────

def ask_current_state(serial):
    """Ask the user where the motor currently is."""
    print()
    print(f'Flip mount Tic {serial}')
    print(f'Travel = {TRAVEL_DEGREES} degrees = {TRAVEL_STEPS} steps')
    print()
    print('Where is the flip mount right now?')
    print('  1 = IN  (mirror/filter is in the beam path)')
    print('  2 = OUT (beam path is clear)')
    while True:
        ans = input('Enter 1 or 2: ').strip()
        if ans == '1':
            return True
        if ans == '2':
            return False
        print('Please enter 1 or 2')


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Flip mount controller')
    parser.add_argument('--serial',    required=True, help='Tic serial number')
    parser.add_argument('--action',    choices=['in', 'out', 'toggle', 'status'],
                        default='status')
    parser.add_argument('--state',     choices=['in', 'out'],
                        help='Current state (skip interactive prompt)')
    parser.add_argument('--direction', type=int, choices=[1, -1], default=1,
                        help='Direction: +1 or -1 for OUT move')
    parser.add_argument('--degrees',   type=float, default=TRAVEL_DEGREES,
                        help=f'Travel angle in degrees (default {TRAVEL_DEGREES})')
    args = parser.parse_args()

    # Override travel degrees if specified
    global TRAVEL_STEPS
    TRAVEL_STEPS = _steps_for_degrees(args.degrees)

    # Determine current state
    if args.state:
        is_in = args.state == 'in'
    elif args.action != 'status':
        is_in = ask_current_state(args.serial)
    else:
        is_in = True  # does not matter for status

    fm = FlipMount(args.serial, is_in=is_in, direction=args.direction)

    if args.action == 'in':
        fm.go_in()
    elif args.action == 'out':
        fm.go_out()
    elif args.action == 'toggle':
        fm.toggle()
    elif args.action == 'status':
        fm.status()


if __name__ == '__main__':
    main()
