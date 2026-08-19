"""The simulated rig: a breathing phantom, two motors, two sensors, one contact model.

Sits behind a :class:`~ct.hw.bus.loopback.LoopbackBus` and speaks the real wire protocol,
so the controller cannot tell it apart from hardware — and neither can the codecs, which
are exercised here rather than bypassed.

**Nothing in ``control/``, ``hw/`` or ``rt/`` may import this module** (rule 3). It is the
hardware layer's equivalent of ``y_clean`` and ``truth``: a controller that can see
simulation ground truth proves nothing when you run it in simulation.
``test_boundaries.py`` enforces it.

What is modelled, and why each part earns its place:

- **The breathing surface** comes from any registered :class:`~ct.interfaces.SignalSource`,
  so ``sinusoid``, ``lujan``, ``rc_piecewise`` and ``csv`` all work unchanged — the same
  generators the estimator was validated against in session 001, and ``csv`` is the route
  for the collected recordings.
- **Motors** as impedance-controlled second-order systems, because that is what makes
  ``kp=0`` genuinely behave as float rather than as a special case in the simulation.
- **A contact model that clips.** The tactile sensor loses the skin at end-exhale when it
  is seated too shallow, and saturates when seated too deep. Without that, APPROACH's
  seating criterion would have nothing to search for and the state would be untested.
- **Tissue capture**, so a floating needle rides with the skin and ADVANCE's increments
  land where the state machine thinks they do.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

from ct.geometry import RigGeometry
from ct.hw.bus.mailbox import Frame
from ct.hw.config import MotorConfig, RigConfig, SensorConfig


def _due(t: float, period: float, last: float) -> bool:
    """Whether a periodic emitter should fire at ``t``, on a fixed grid.

    Not ``t - last >= period``. The loop's tick times are accumulated floats, so that
    comparison misses by an ulp every so often and the emitter silently skips a slot: a
    sensor configured for 100 Hz measured 66.7 Hz with 33% jitter, which then looked like
    a property of the *sensor* rather than of the comparison. Snapping both times to a
    grid index removes the drift entirely — and a simulation that does not honour its own
    configured rate is worse than useless, because the rate is a documented unknown that
    someone will read this output to calibrate.
    """
    if last == -np.inf:
        return True
    return int(np.floor(t / period + 1e-9)) > int(np.floor(last / period + 1e-9))


@dataclass
class MotorPhysics:
    """Second-order impedance-controlled motor, in raw codec units.

    The equation of motion is the impedance law the MIT protocol implies::

        tau = kp*(p_des - p) + kd*(v_des - v) + tau_ff + tau_external
        a   = tau/inertia - damping*v

    which gives float for free: with ``kp = 0`` the only thing moving the axis is the
    external force, which is exactly what "backdrivable" means. Modelling it any other way
    would have made ADVANCE's central mechanism a simulation artefact.
    """

    inertia: float = 0.02
    damping: float = 0.5
    v_max: float = 1e6
    """Raw-unit speed ceiling, from the commanded velocity limit."""

    position: float = 0.0
    velocity: float = 0.0

    kp: float = 0.0
    kd: float = 0.0
    p_des: float = 0.0
    v_des: float = 0.0
    tau_ff: float = 0.0
    enabled: bool = False

    def step(self, dt: float, tau_external: float = 0.0) -> None:
        if not self.enabled or dt <= 0:
            return
        tau = (
            self.kp * (self.p_des - self.position)
            + self.kd * (self.v_des - self.velocity)
            + self.tau_ff
            + tau_external
        )
        accel = tau / self.inertia - self.damping * self.velocity
        self.velocity += accel * dt
        self.velocity = float(np.clip(self.velocity, -self.v_max, self.v_max))
        self.position += self.velocity * dt

    @property
    def current(self) -> float:
        """Torque-proportional reading, as the motor would report it."""
        return self.kp * (self.p_des - self.position) + self.kd * (self.v_des - self.velocity)


class SimulatedMotor:
    """A motor on the simulated bus: decodes real command frames, emits real replies."""

    def __init__(
        self,
        name: str,
        config: MotorConfig,
        codec: Any,
        physics: MotorPhysics | None = None,
        initial_position: float = 0.0,
    ) -> None:
        self.name = name
        self.config = config
        self.codec = codec
        self.physics = physics or MotorPhysics()
        self.physics.position = initial_position
        self._last_emit = -np.inf
        self.frames_received = 0
        self.external_force: float = 0.0
        """Set by the rig each step: tissue drag, contact reaction, whatever applies."""

    def on_frame(self, can_id: int, data: bytes) -> bool:
        """Try to obey a frame. Returns whether it was ours."""
        parsed = self.codec.parse_command(can_id, data)
        if parsed is None or int(parsed.get("can_id", -1)) != self.config.can_id:
            return False
        self.frames_received += 1
        mode = parsed["mode"]
        if mode == "enable":
            self.physics.enabled = True
        elif mode == "disable":
            self.physics.enabled = False
            self.physics.kp = self.physics.kd = 0.0
        elif mode == "zero":
            self.physics.position = 0.0
        else:
            self.physics.enabled = True
            self.physics.kp = parsed["kp"]
            self.physics.kd = parsed["kd"]
            self.physics.tau_ff = parsed["torque"]
            if parsed["position"] is not None:
                self.physics.p_des = parsed["position"]
            if parsed["velocity"]:
                self.physics.v_max = abs(parsed["velocity"])
        return True

    def step(self, dt: float) -> None:
        self.physics.step(dt, tau_external=self.external_force)

    def emit(self, t: float) -> list[Frame]:
        """Status frames, at the configured rate rather than every tick."""
        if not _due(t, 1.0 / self.config.status_rate_hz, self._last_emit):
            return []
        self._last_emit = t
        can_id, data, _extended = self.codec.encode_reply(
            self.config.can_id,
            self.physics.position,
            self.physics.velocity,
            self.physics.current,
        )
        return [(t, can_id, data)]


class SimulatedSensor:
    """A periodic scalar sensor with noise, quantisation and honest latency.

    The delay line is the part that matters: ``tau_s`` is a real term in the forecast
    horizon, and a simulation that reported sensor values instantaneously would make the
    horizon look irrelevant and hide exactly the error the forecast exists to cancel.
    """

    def __init__(
        self,
        name: str,
        config: SensorConfig,
        read: Callable[[], float],
        noise_counts: float = 0.0,
        quantum_counts: float = 0.0,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.name = name
        self.config = config
        self.read = read
        self.noise_counts = float(noise_counts)
        self.quantum_counts = float(quantum_counts)
        self.rng = rng or np.random.default_rng(0)
        self._last_emit = -np.inf
        self._delay: deque[tuple[float, float]] = deque()
        self.frames_emitted = 0

    def emit(self, t: float) -> list[Frame]:
        if not _due(t, 1.0 / self.config.rate_hz, self._last_emit):
            return []
        self._last_emit = t

        counts = self.read()
        if self.noise_counts > 0:
            counts += self.rng.normal(0.0, self.noise_counts)
        if self.quantum_counts > 0:
            counts = round(counts / self.quantum_counts) * self.quantum_counts
        self._delay.append((t + self.config.latency_s, counts))

        out: list[Frame] = []
        while self._delay and self._delay[0][0] <= t:
            _release_at, value = self._delay.popleft()
            out.append((t, self.config.can_id, self._encode(value)))
            self.frames_emitted += 1
        return out

    def _encode(self, value: float) -> bytes:
        layout = self.config.layout
        raw = int(round(value / layout.scale))
        limit = 1 << (8 * layout.length)
        if layout.signed:
            raw = int(np.clip(raw, -(limit // 2), limit // 2 - 1))
        else:
            raw = int(np.clip(raw, 0, limit - 1))
        payload = bytearray(8)
        payload[layout.offset : layout.offset + layout.length] = raw.to_bytes(
            layout.length, layout.byteorder, signed=layout.signed
        )
        return bytes(payload)


@dataclass
class TissueModel:
    """What happens to the needle once it is in the phantom: stick-slip friction.

    Two behaviours have to coexist, and getting only one of them is easy:

    - **Stick.** A floating needle is carried along by the tissue as the phantom breathes.
      That is the entire premise of ADVANCE, so without it the state would be verified
      against a needle hanging in free space.
    - **Slip.** A *driven* needle must be able to advance. Tissue resists, but only up to
      a bounded force; past that the needle cuts through and the tissue re-grips further
      along.

    Modelling the grip as an unbounded spring gives the first and forbids the second — the
    first version here did exactly that, and the needle stalled 1.5 mm in with the motor
    at full torque, because the spring outgrew what the motor could produce. Coulomb
    friction with a slip limit is both physically right and the only version in which
    INSERT can succeed while ADVANCE still means something.
    """

    stiffness: float = 400.0
    """Restoring force per mm of relative displacement while stuck, in raw torque units."""

    damping: float = 5.0
    """Viscous term, per mm/s of relative velocity.

    Two constraints pin this from both sides, and both were found by getting it wrong.
    Too high relative to :attr:`max_grip` and ``damping * v`` alone reaches the slip limit
    at ordinary breathing velocities, so the tissue lets go on every tick and never
    carries the needle — which in the telemetry looks exactly like a controller that
    forgot to float. Too low and the needle-plus-tissue system is barely damped and rings
    at its natural frequency instead of riding. The values here put the coupled system
    comfortably overdamped while leaving the damping term well under the grip limit.
    """

    max_grip: float = 200.0
    """Force beyond which the tissue lets go and the needle cuts forward.

    Bounded below by what it takes to carry the needle through a breath, and above by what
    the needle motor can produce — insertion is impossible otherwise. That is a real
    constraint on the rig, not just on the simulation: it says the needle drive must be
    able to out-push tissue friction with enough margin left over that the residual
    steady-state error (``grip / kp`` under impedance control) is smaller than the
    placement tolerance.
    """

    captured: bool = False
    capture_tip_x: float = 0.0
    capture_skin_x: float = 0.0
    slips: int = 0

    def update(self, tip_x: float, skin_x: float) -> None:
        inside = tip_x > skin_x
        if inside and not self.captured:
            self.captured = True
            self.capture_tip_x = tip_x
            self.capture_skin_x = skin_x
        elif not inside and self.captured:
            self.captured = False

    def rest_x(self, skin_x: float) -> float:
        """Where the grip point currently is: it travels with the skin."""
        return self.capture_tip_x + (skin_x - self.capture_skin_x)

    def force(self, tip_x: float, tip_v: float, skin_x: float) -> float:
        """Force the tissue applies to the needle tip, in raw torque units."""
        if not self.captured:
            return 0.0
        wanted = self.stiffness * (self.rest_x(skin_x) - tip_x) - self.damping * tip_v
        if abs(wanted) <= self.max_grip:
            return wanted
        # Slip: clamp the force and re-anchor the grip so the spring sits exactly at the
        # limit. Without the re-anchor the stored displacement would keep growing and the
        # needle would snap backwards the moment it stopped being driven.
        clamped = self.max_grip if wanted > 0 else -self.max_grip
        self.capture_tip_x = tip_x + clamped / self.stiffness - (skin_x - self.capture_skin_x)
        self.slips += 1
        return clamped


@dataclass
class RigTruth:
    """Ground truth for validation and plots only.

    The simulation's ``y_clean``. Written to the run's artifacts so sensing accuracy can
    be scored afterwards; never routed to the controller.
    """

    t: float = 0.0
    skin_x: float = 0.0
    base_mm: float = 0.0
    needle_mm: float = 0.0
    needle_tip_x: float = 0.0
    tactile_deflection_mm: float = 0.0
    insertion_depth_mm: float = 0.0
    in_tissue: bool = False
    history: list[dict[str, float]] = field(default_factory=list)


class SimulatedRig:
    """The whole simulated rig, as one loopback-bus device."""

    def __init__(
        self,
        rig: RigConfig,
        source: Any,
        codecs: dict[str, Any],
        *,
        skin_rest_x: float = 150.0,
        tactile_noise_counts: float = 0.0,
        tof_noise_counts: float = 0.0,
        seed: int = 0,
        record_truth: bool = True,
    ) -> None:
        self.rig = rig
        self.geometry: RigGeometry = rig.geometry
        self.source = source
        self.skin_rest_x = float(skin_rest_x)
        """Rig-frame skin position at zero breathing signal."""

        self.rng = np.random.default_rng(seed)
        self.tissue = TissueModel()
        self.truth = RigTruth()
        self.record_truth = record_truth
        self._t = 0.0
        self._last_step = 0.0
        self._breath_held_at: float | None = None

        self.motors: dict[str, SimulatedMotor] = {}
        for name in ("base", "needle"):
            if name not in rig.motors:
                continue
            config = rig.motors[name]
            calibration = getattr(self.geometry, name)
            self.motors[name] = SimulatedMotor(
                name=name,
                config=config,
                codec=codecs[name],
                physics=MotorPhysics(
                    inertia=0.02 if name == "needle" else 0.05,
                    damping=1.0,
                ),
                initial_position=calibration.to_counts(0.0),
            )

        self.sensors: dict[str, SimulatedSensor] = {}
        if "tactile" in rig.sensors:
            self.sensors["tactile"] = SimulatedSensor(
                "tactile",
                rig.sensors["tactile"],
                read=self._read_tactile_counts,
                noise_counts=tactile_noise_counts,
                rng=self.rng,
            )
        if "tof" in rig.sensors:
            self.sensors["tof"] = SimulatedSensor(
                "tof",
                rig.sensors["tof"],
                read=self._read_tof_counts,
                noise_counts=tof_noise_counts,
                rng=self.rng,
            )

    # -- the world -------------------------------------------------------------

    def skin_x(self, t: float) -> float:
        """Rig-frame skin position at time ``t``.

        Inhale moves the skin *toward* the rig, so the signal is subtracted: a peak in the
        breathing waveform is the minimum of ``skin_x``. Getting this sign wrong would
        make the standoff constraint bind at the wrong end of the cycle.
        """
        if self._breath_held_at is not None:
            t = self._breath_held_at
        return self.skin_rest_x - float(self.source.clean(np.array([t]))[0])

    def hold_breath(self, t: float | None) -> None:
        """Freeze (or release) the phantom, for a breath-hold noise measurement."""
        self._breath_held_at = t

    def _position_mm(self, name: str) -> float:
        motor = self.motors.get(name)
        if motor is None:
            return 0.0
        return getattr(self.geometry, name).to_mm(motor.physics.position)

    def _read_tactile_counts(self) -> float:
        """Contact model, including both ways the reading can clip.

        ``max(0, ...)`` is the shallow-seating case: at end-exhale the skin recedes past
        the face, contact is lost, and the trough of the waveform is cut off. The
        saturation clamp is the too-deep case. The seating sub-step of APPROACH exists to
        find the window between them, so both have to be here for that search to be real.
        """
        face_x = self.geometry.tactile_face_x(self._position_mm("base"))
        deflection = max(0.0, face_x - self.skin_x(self._t))
        deflection = min(deflection, self.geometry.tactile_saturation_mm)
        return deflection / self.geometry.tactile_counts_to_mm

    def _read_tof_counts(self) -> float:
        distance = self.skin_x(self._t) - self.geometry.tof_face_x(self._position_mm("base"))
        return max(0.0, distance) / self.geometry.tof_counts_to_mm

    # -- LoopbackDevice protocol -----------------------------------------------

    def on_frame(self, t: float, can_id: int, data: bytes, extended: bool) -> None:
        for motor in self.motors.values():
            if motor.on_frame(can_id, data):
                return

    def emit(self, t: float) -> list[Frame]:
        self._step_to(t)
        frames: list[Frame] = []
        for motor in self.motors.values():
            frames += motor.emit(t)
        for sensor in self.sensors.values():
            frames += sensor.emit(t)
        return frames

    def _step_to(self, t: float) -> None:
        """Advance physics to ``t``, in bounded sub-steps.

        Sub-stepping keeps the motor integration stable when the control loop's tick is
        long relative to the motor's dynamics — which it is, at 200 Hz against a stiff
        position loop.
        """
        dt_total = t - self._last_step
        if dt_total <= 0:
            self._t = t
            return
        max_dt = 0.001
        n = max(1, int(np.ceil(dt_total / max_dt)))
        dt = dt_total / n
        for i in range(n):
            self._t = self._last_step + dt * (i + 1)
            skin = self.skin_x(self._t)
            needle = self.motors.get("needle")
            if needle is not None:
                tip_x = self.geometry.needle_tip_x(
                    self._position_mm("base"), self._position_mm("needle")
                )
                tip_v = self.geometry.needle.rate_to_mm_s(needle.physics.velocity)
                self.tissue.update(tip_x, skin)
                needle.external_force = self.tissue.force(tip_x, tip_v, skin)
            for motor in self.motors.values():
                motor.step(dt)
        self._last_step = t
        self._t = t
        self._record()

    def _record(self) -> None:
        base_mm = self._position_mm("base")
        needle_mm = self._position_mm("needle")
        skin = self.skin_x(self._t)
        tip_x = self.geometry.needle_tip_x(base_mm, needle_mm)
        face_x = self.geometry.tactile_face_x(base_mm)
        self.truth = RigTruth(
            t=self._t,
            skin_x=skin,
            base_mm=base_mm,
            needle_mm=needle_mm,
            needle_tip_x=tip_x,
            tactile_deflection_mm=min(
                max(0.0, face_x - skin), self.geometry.tactile_saturation_mm
            ),
            insertion_depth_mm=tip_x - skin,
            in_tissue=self.tissue.captured,
            history=self.truth.history,
        )
        if self.record_truth:
            self.truth.history.append(
                {
                    "t": self._t,
                    "skin_x": skin,
                    "base_mm": base_mm,
                    "needle_mm": needle_mm,
                    "needle_tip_x": tip_x,
                    "insertion_depth_mm": tip_x - skin,
                }
            )

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "motors": {n: m.frames_received for n, m in self.motors.items()},
            "sensors": {n: s.frames_emitted for n, s in self.sensors.items()},
            "in_tissue": self.tissue.captured,
            "insertion_depth_mm": self.truth.insertion_depth_mm,
        }
