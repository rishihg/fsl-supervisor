"""
alignment/state_machine.py — supervisory state machine for the full
optical alignment and beam-steering sequence.

States
------
IDLE            Nothing is running.  Safe to reconfigure hardware.
COARSE_ALIGN    Shutter open, laser on.  Slow steering mirror is commanded
                open-loop via udp_source to find the alignment beam on the
                camera.  Exits when CoM is within capture_radius of centre.
FIBER_COUPLE    ActiveFiberCoupling hill-climber is running on the
                translation stage to maximise fibre coupling.
LOCK_ACQUIRE    anyloop is started with PID enabled.  Waiting for CoM to
                settle within lock_radius.
RUNNING         Fast/slow closed-loop steering is active.  This is the
                normal operating state.
STOPPING        Orderly shutdown sequence: disabling integrators, stopping
                anyloop, closing shutter.
FAULT           An unrecoverable error occurred.  Human intervention needed.

Transitions are driven by:
  - supervisor CLI commands (start_sequence, abort)
  - telemetry from anyloop (CoM residual)
  - completion callbacks from ActiveFiberCoupling
"""

import logging
from enum import Enum, auto
from typing import Optional

log = logging.getLogger(__name__)


class State(Enum):
    IDLE          = 'idle'
    COARSE_ALIGN  = 'coarse_align'
    FIBER_COUPLE  = 'fiber_couple'
    LOCK_ACQUIRE  = 'lock_acquire'
    RUNNING       = 'running'
    STOPPING      = 'stopping'
    FAULT         = 'fault'


class AlignmentStateMachine:
    """Supervisory state machine.

    Args:
        anyloop:   AnyloopProcess instance.
        commander: CommandSender aimed at the udp_source port.
        shutter:   Shutter instance.
        laser:     Laser instance.
        telemetry: TelemetryReceiver for CoM data.
        stage:     StageDevices instance (ActiveFiberCoupling), or None if
                   fiber coupling is not yet connected.
        config:    Path to the anyloop JSON pipeline config.
    """

    # CoM residual thresholds (normalised ±1 units)
    CAPTURE_RADIUS = 0.1   # enter FIBER_COUPLE when |CoM| < this
    LOCK_RADIUS    = 0.05  # declare lock when residual < this

    def __init__(self, anyloop, commander, shutter, laser,
                 telemetry=None, stage=None,
                 config='/home/fsl/steering.json'):
        self.anyloop   = anyloop
        self.commander = commander
        self.shutter   = shutter
        self.laser     = laser
        self.telemetry = telemetry
        self.stage     = stage
        self.config    = config
        self.state     = State.IDLE

    # ── public interface ─────────────────────────────────────────────────────

    def start_sequence(self):
        """Begin the automated alignment sequence from IDLE."""
        if self.state != State.IDLE:
            raise RuntimeError(f'Cannot start sequence from state {self.state.value}')
        self._to(State.COARSE_ALIGN)
        self._run_coarse_align()

    def abort(self):
        """Emergency stop from any state."""
        log.warning('ABORT requested from state %s', self.state.value)
        self._safe_shutdown()
        self._to(State.IDLE)

    def step(self):
        """Advance the state machine based on current telemetry.

        Call this periodically from the supervisor main loop (e.g. every
        100 ms) to drive automatic state transitions without blocking.
        Returns the current state after any transitions.
        """
        if self.state == State.LOCK_ACQUIRE:
            self._check_lock()
        elif self.state == State.RUNNING:
            self._check_running()
        return self.state

    # ── state entry actions ──────────────────────────────────────────────────

    def _run_coarse_align(self):
        """Open shutter, enable laser, zero the setpoint.

        The operator (or a future automated routine) then uses 'setpoint'
        commands to steer the slow mirror until the beam is centred.
        Transition to FIBER_COUPLE must currently be triggered manually
        with 'fiber_couple'; automatic transition on CoM threshold is a
        TODO once we have open-loop mirror control.
        """
        self.shutter.open()
        self.laser.enable()
        # zero setpoint so anyloop (when started) aims for beam centre
        self.commander.send(0.0, 0.0)
        log.info('Coarse align: shutter open, laser on, setpoint zeroed.')
        log.info('Steer beam manually or use "setpoint y x" commands.')

    def _run_fiber_couple(self):
        """Run the ActiveFiberCoupling hill-climber."""
        if self.stage is None:
            raise NotImplementedError(
                'No stage connected — ActiveFiberCoupling not available yet.'
            )
        # TODO: import and call HillClimb.run() once AFC is integrated
        # from ActiveFiberCoupling.HillClimb import run as hill_climb_run
        # hill_climb_run(self.stage, movementType=..., exposureTime=...)
        raise NotImplementedError('Fiber coupling sequence not yet implemented')

    def _run_lock_acquire(self):
        """Start anyloop and wait for the loop to lock."""
        self.anyloop.start(self.config)
        log.info('anyloop started; waiting for lock (|CoM| < %.3f)...', self.LOCK_RADIUS)

    def _check_lock(self):
        """Transition LOCK_ACQUIRE → RUNNING when residual is small enough."""
        if self.telemetry is None:
            return
        pkt = self.telemetry.get()
        if pkt is None or not pkt.data:
            return
        residual = (pkt.data[0]**2 + pkt.data[1]**2) ** 0.5
        if residual < self.LOCK_RADIUS:
            log.info('Loop locked (residual=%.4f)', residual)
            self._to(State.RUNNING)

    def _check_running(self):
        """Fault if anyloop dies unexpectedly."""
        if not self.anyloop.running:
            log.error('anyloop exited unexpectedly')
            self._to(State.FAULT)

    def _safe_shutdown(self):
        """Stop anyloop, close shutter, disable laser."""
        try:
            self.anyloop.stop()
        except Exception:
            pass
        try:
            self.shutter.close()
        except Exception:
            pass
        try:
            self.laser.disable()
        except Exception:
            pass

    # ── helpers ──────────────────────────────────────────────────────────────

    def _to(self, new_state: State):
        log.info('State: %s → %s', self.state.value, new_state.value)
        self.state = new_state
