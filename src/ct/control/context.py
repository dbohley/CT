"""Everything a procedure state is allowed to see.

One object, assembled once, handed to each state's ``update``. States hold no references
to buses or codecs and never convert units — they ask the context for millimetres and
issue commands in millimetres. That is what keeps them readable as *procedure* rather than
as plumbing, and what lets the same state code run against the simulated and the real rig.

The estimator lives here too, because it outlives the state that created it: ESTIMATE
builds the tracker, and INSERT and ADVANCE go on using it. Its samples are fed from
:meth:`ProcedureContext.update` every tick, so tracking continues regardless of which
state happens to be active.
"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from ct.control.live import SignalAccumulator
from ct.control.safety import SafetyMonitor
from ct.control.servo import LeadServo
from ct.control.state import ProcedureState
from ct.geometry import RigGeometry, RigState
from ct.hw.config import ProcedureConfig
from ct.rt.latency import LatencyBudget


class ProcedureContext:
    """The controller's world, in millimetres."""

    def __init__(
        self,
        geometry: RigGeometry,
        axes: dict[str, Any],
        sensors: dict[str, Any],
        config: ProcedureConfig,
        latency: LatencyBudget,
        safety: SafetyMonitor,
        servo: LeadServo,
        *,
        build_tracker: Callable[[], Any],
        identify: Callable[[Any], Any],
        log: Callable[[str, dict[str, Any]], None] | None = None,
        dry_run: bool = False,
    ) -> None:
        self.geometry = geometry
        self.axes = axes
        self.sensors = sensors
        self.config = config
        self.latency = latency
        self.safety = safety
        self.servo = servo
        self.dry_run = dry_run

        self._build_tracker = build_tracker
        self._identify = identify
        self._log = log

        self.tracker: Any | None = None
        """The EKF. ``None`` until ESTIMATE has identified a model."""

        self.ident: Any | None = None
        self.accumulator = SignalAccumulator()
        self.state = RigState(t=0.0)
        self.procedure_state = ProcedureState.IDLE

        # Carried between states.
        self.skin_x_at_max_inhale: float | None = None
        """Nearest the skin comes, measured during APPROACH. Sets the standoff."""

        self.breath_hold_window: tuple[float, float] | None = None
        """Wall-clock window of a breath-hold segment, if APPROACH took one.

        Handed to the identifier as ``breath_hold_window`` so ``R`` comes from a genuine
        noise measurement rather than the residual-variance upper bound that
        ``CLAUDE.md`` records collapsing NIS to ~0.027.
        """

        self.insertion_reference_skin_x: float | None = None
        self.samples_tracked = 0
        self.last_nis: float | None = None
        self._extrema: tuple[float, float] | None = None
        self._extrema_at: float | None = None

    # -- axes ------------------------------------------------------------------

    @property
    def base(self) -> Any:
        return self.axes["base"]

    @property
    def needle(self) -> Any:
        return self.axes["needle"]

    # -- per-tick --------------------------------------------------------------

    def update(self, t: float, frames: dict[str, list[tuple[float, int, bytes]]]) -> RigState:
        """Fold this tick's frames into the world view, and advance the estimator."""
        flat = [frame for batch in frames.values() for frame in batch]
        for axis in self.axes.values():
            axis.update(t, flat)
        for sensor in self.sensors.values():
            sensor.update(t, flat)

        self.state = self._read_state(t)
        self._feed_estimator()
        return self.state

    def _read_state(self, t: float) -> RigState:
        geometry = self.geometry
        base_mm = self.base.position_mm
        needle_mm = self.needle.position_mm

        stale: list[str] = []
        tactile = self.sensors.get("tactile")
        tof = self.sensors.get("tof")

        tactile_mm: float | None = None
        in_contact = False
        if tactile is not None:
            if tactile.is_stale:
                stale.append("tactile")
            if tactile.counts is not None:
                in_contact = geometry.in_contact(tactile.counts)
                tactile_mm = geometry.tactile_deflection_mm(tactile.counts)

        tof_mm: float | None = None
        if tof is not None:
            if tof.is_stale:
                stale.append("tof")
            if tof.counts is not None and geometry.tof_in_range(tof.counts):
                tof_mm = geometry.tof_distance_mm(tof.counts)

        # Tactile wins whenever we are touching: it is a contact measurement of the thing
        # we care about, where ToF is a range to whatever happens to be in its cone.
        skin_x: float | None = None
        if in_contact and tactile_mm is not None:
            skin_x = geometry.skin_x_from_deflection(base_mm, tactile_mm)
        elif tof_mm is not None:
            skin_x = geometry.tof_face_x(base_mm) + tof_mm

        return RigState(
            t=t,
            base_mm=base_mm,
            needle_mm=needle_mm,
            tactile_mm=tactile_mm,
            tof_mm=tof_mm,
            skin_x=skin_x,
            in_contact=in_contact,
            stale=tuple(stale),
        )

    def _feed_estimator(self) -> None:
        """Accumulate the tactile stream, and step the tracker once it exists.

        Raw arrival stamps, not the loop tick: the EKF handles irregular ``dt`` natively
        and resampling here would throw away real timing information for nothing.
        """
        tactile = self.sensors.get("tactile")
        if tactile is None or tactile.counts is None or tactile.stamp is None:
            return
        value = self.geometry.tactile_deflection_mm(tactile.counts)
        if not self.accumulator.offer(tactile.stamp, value):
            return
        if self.tracker is None:
            return
        step = self.tracker.step(float(tactile.stamp), float(value))
        self.samples_tracked += 1
        self.last_nis = step.nis
        trip = self.safety.check_residual(self.state.t, step.nis)
        if trip is not None:
            self.log("residual_trip", {"detail": trip.detail, "nis": step.nis})

    # -- estimator -------------------------------------------------------------

    def identify_and_start_tracker(self, fs: float | None = None) -> Any:
        """Run Stage 1 on what has accumulated, then seed Stage 2. Used by ESTIMATE.

        The two halves keep different clocks, and reconciling them is the whole subtlety
        here. :meth:`SignalAccumulator.to_batch` rebases time to zero, so the identifier's
        ``t0`` is "seconds into the calibration window" — but the tracker is then stepped
        with raw loop timestamps. Seeding it with the batch's own ``t0`` would make its
        first step span the entire gap between the two, tens of seconds of prediction in
        one go, which shows up immediately as an enormous NIS. Passing the real arrival
        time of the last calibration sample lines the two clocks up.
        """
        batch = self.accumulator.to_batch(fs=fs)
        t_end = self.accumulator.t_last
        self.ident = self._identify(batch)
        self.tracker = self._build_tracker()
        self.tracker.init(self.ident, t0=t_end)
        return self.ident

    @property
    def has_model(self) -> bool:
        return self.tracker is not None

    @property
    def omega_r(self) -> float:
        """Tracked breathing rate [rad/s], or zero before a model exists."""
        if self.tracker is None:
            return 0.0
        s, _P = self.tracker.state
        return float(s[self.tracker.layout.omega])

    @property
    def breath_period(self) -> float:
        """Tracked breath period, falling back to the configured nominal before a model."""
        omega = self.omega_r
        if omega <= 0:
            return self.config.approach.nominal_breath_s
        return 2.0 * np.pi / omega

    def horizon(self) -> float:
        """The full forecast horizon ``h`` at the current breathing rate."""
        return self.latency.horizon(self.omega_r)

    def forecast_deflection(self, h: float | None = None) -> float:
        """Tactile deflection the model expects in ``h`` seconds."""
        if self.tracker is None:
            raise RuntimeError("no tracked model yet; ESTIMATE has not run")
        return float(self.tracker.forecast(self.horizon() if h is None else h))

    def forecast_skin_x(self, h: float | None = None) -> float:
        """Rig-frame skin position the model expects in ``h`` seconds.

        The conversion the needle actually aims at. Note it uses the *current* base
        position: the base is parked from the end of APPROACH onward, so this is exact,
        and it would be wrong to forecast a base that is moving.
        """
        return self.geometry.skin_x_from_deflection(
            self.state.base_mm, self.forecast_deflection(h)
        )

    def forecast_std(self, h: float | None = None) -> float:
        if self.tracker is None:
            raise RuntimeError("no tracked model yet; ESTIMATE has not run")
        h = self.horizon() if h is None else h
        return float(np.sqrt(max(0.0, self.tracker.forecast_variance(h))))

    # -- depth -----------------------------------------------------------------

    def insertion_depth_mm(self, skin_x: float | None = None) -> float:
        """How far the tip is past the *instantaneous* skin. Negative means still clear.

        Note this oscillates over the breathing cycle by the full excursion whenever the
        needle is holding position rather than floating, because the skin moves and the
        needle does not. That is real, not an artefact — but it makes a poor termination
        criterion, so :meth:`depth_at_exhale` is what ADVANCE actually uses.
        """
        if skin_x is None:
            skin_x = self.state.skin_x
        if skin_x is None:
            raise RuntimeError("cannot measure insertion depth without a skin position")
        return self.geometry.insertion_depth_mm(
            self.state.base_mm, self.state.needle_mm, skin_x
        )

    def cycle_extrema(self) -> tuple[float, float] | None:
        """Min and max of the tracked waveform over one breath, cached for this tick.

        Sweeping a full cycle costs dozens of model evaluations, and both the firing gate
        and the depth datum want the answer on the same tick. Caching against the tick
        time keeps it to one sweep per loop iteration.
        """
        if self.tracker is None:
            return None
        if self._extrema_at == self.state.t and self._extrema is not None:
            return self._extrema
        from ct.control.gate import cycle_extrema  # noqa: PLC0415 - avoids an import cycle

        s, _P = self.tracker.state
        self._extrema = cycle_extrema(s, self.tracker.layout)
        self._extrema_at = self.state.t
        return self._extrema

    def skin_x_at_exhale(self) -> float | None:
        """Rig-frame skin position at end-exhale, from the tracked model.

        A phase-consistent datum. Every insertion is commanded at end-exhale, so measuring
        depth against the skin at that same point in the cycle is the only way the number
        means one thing from tick to tick — and it is the same reference the firing gate
        keys on, so depth and gating cannot disagree about where the skin is.
        """
        extrema = self.cycle_extrema()
        if extrema is None:
            return None
        y_min, _y_max = extrema
        return self.geometry.skin_x_from_deflection(self.state.base_mm, y_min)

    def depth_at_exhale(self) -> float | None:
        """Insertion depth measured against the end-exhale skin position."""
        skin_x = self.skin_x_at_exhale()
        return None if skin_x is None else self.insertion_depth_mm(skin_x)

    def needle_mm_for_depth(self, depth_mm: float, skin_x: float) -> float:
        """Needle command that puts the tip ``depth_mm`` past ``skin_x``."""
        return self.geometry.needle_mm_for_tip_at(skin_x + depth_mm, self.state.base_mm)

    # -- logging ---------------------------------------------------------------

    def log(self, event: str, detail: dict[str, Any] | None = None) -> None:
        """Record something worth reading back. Cheap enough to call freely."""
        if self._log is not None:
            self._log(event, detail or {})

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "samples_tracked": self.samples_tracked,
            "accumulator": self.accumulator.stats,
            "omega_r": self.omega_r,
            "last_nis": self.last_nis,
            "skin_x_at_max_inhale": self.skin_x_at_max_inhale,
            "axes": {n: a.stats for n, a in self.axes.items()},
            "sensors": {n: s.stats for n, s in self.sensors.items()},
        }
