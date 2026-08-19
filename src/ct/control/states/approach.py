"""State 1 — APPROACH: get the tactile sensor seated and the needle at standoff.

The longest state, and the only one with real sub-structure. Four steps:

**COARSE** — drive the base in under time-of-flight until the skin is within
``coarse_standoff_mm``. Open-loop speed, closed-loop distance.

**CONTACT** — creep forward in small steps until the tactile sensor reports touching.
ToF is done at this point; it was only ever there to avoid crossing a large gap slowly.

**SEAT** — the interesting one. Creep deeper, one increment at a time, watching the
peak-to-trough amplitude of the tactile signal after each. Seated too shallow, the sensor
*loses the skin at end-exhale* and the trough of the waveform is cut off, so the measured
amplitude reads low. Each increment recovers more of the excursion, until the sensor stays
in contact through the whole breath and the amplitude stops growing. That plateau is the
signal that the full breathing motion is finally visible, and it is the exit condition.

Two details that would otherwise corrupt the measurement:

- Increments are commanded **only at a detected trough** — when the skin is furthest away
  and the needle-to-skin gap is at its largest. Moving in at peak inhale would press
  hardest at exactly the wrong moment.
- Amplitude is only accumulated **while the base is stationary**. A moving base adds its
  own displacement to the tactile reading, which looks exactly like a larger breathing
  amplitude and would let the criterion satisfy itself.

**STANDOFF** — extend the *needle*, not the base, until the tip sits ``standoff_mm`` short
of the skin at maximum inhale. The base must stay seated: the tactile sensor is the
breathing signal, and backing it off would end the measurement the rest of the procedure
depends on.

Optionally followed by a **breath hold**, which is nearly free here — the base is parked
and the sensor is already seated — and gives Stage 1 an honest ``R`` instead of the
residual-variance upper bound.
"""

from __future__ import annotations

from enum import Enum

from ct.control.context import ProcedureContext
from ct.control.live import AmplitudeWatcher
from ct.control.state import ProcedureState, Transition
from ct.control.states.base import BaseState
from ct.registry import register_state


class Step(Enum):
    COARSE = "coarse"
    CONTACT = "contact"
    SEAT = "seat"
    STANDOFF = "standoff"
    BREATH_HOLD = "breath_hold"


@register_state("approach")
class ApproachState(BaseState):
    state = ProcedureState.APPROACH

    def __init__(self) -> None:
        super().__init__()
        self.step = Step.COARSE
        self.watcher: AmplitudeWatcher | None = None
        self.increments = 0
        self.stable_count = 0
        self.previous_amplitude: float | None = None
        self.max_deflection_mm = 0.0
        self.step_started_at = 0.0
        self._settling_until = 0.0
        self._last_trough_seen = 0.0

    def enter(self, ctx: ProcedureContext) -> None:
        super().enter(ctx)
        cfg = ctx.config.approach
        self.timeout_s = cfg.timeout_s
        self.step = Step.COARSE
        self.step_started_at = ctx.state.t
        window = cfg.min_breaths * cfg.nominal_breath_s
        self.watcher = AmplitudeWatcher(window_s=window)
        ctx.base.enable()
        ctx.needle.enable()
        ctx.needle.hold()
        ctx.log("approach_enter", {"step": self.step.value, "amplitude_window_s": window})

    def update(self, ctx: ProcedureContext) -> Transition | None:
        expired = self.timed_out(ctx)
        if expired is not None:
            return expired

        if self.step is Step.COARSE:
            return self._coarse(ctx)
        if self.step is Step.CONTACT:
            return self._contact(ctx)
        if self.step is Step.SEAT:
            return self._seat(ctx)
        if self.step is Step.STANDOFF:
            return self._standoff(ctx)
        return self._breath_hold(ctx)

    # -- coarse ---------------------------------------------------------------

    def _coarse(self, ctx: ProcedureContext) -> Transition | None:
        cfg = ctx.config.approach
        state = ctx.state

        if state.tof_mm is None:
            # No valid range yet. Hold rather than guess: driving a base toward a phantom
            # on a reading you do not have is the one thing this state must never do.
            ctx.base.hold()
            return None

        if state.in_contact:
            # Contact before the ToF threshold — the offsets may be off, or the phantom is
            # closer than the sensor's near limit. Either way, contact is authoritative.
            ctx.log("approach_early_contact", {"tof_mm": state.tof_mm})
            return self._begin(ctx, Step.SEAT)

        if state.tof_mm <= cfg.coarse_standoff_mm:
            return self._begin(ctx, Step.CONTACT)

        remaining = state.tof_mm - cfg.coarse_standoff_mm
        target = state.base_mm + remaining
        ctx.base.move_to(target, v_max_mm_s=cfg.coarse_speed_mm_s)
        return None

    # -- contact --------------------------------------------------------------

    def _contact(self, ctx: ProcedureContext) -> Transition | None:
        cfg = ctx.config.approach
        if ctx.state.in_contact:
            ctx.log("approach_contact", {"base_mm": ctx.state.base_mm})
            return self._begin(ctx, Step.SEAT)

        if ctx.state.t < self._settling_until:
            return None
        ctx.base.move_to(ctx.state.base_mm + cfg.creep_increment_mm,
                         v_max_mm_s=cfg.creep_speed_mm_s)
        self._settling_until = ctx.state.t + self._settle_time(cfg)
        self.increments += 1
        if self.increments > cfg.max_seat_increments:
            return Transition(
                ProcedureState.FAULT,
                f"no tactile contact after {self.increments} creep increments; check "
                "rig.geometry.tactile_contact_counts and that the sensor is publishing",
            )
        return None

    # -- seat -----------------------------------------------------------------

    def _seat(self, ctx: ProcedureContext) -> Transition | None:
        cfg = ctx.config.approach
        state = ctx.state
        assert self.watcher is not None

        if state.tactile_mm is None:
            ctx.base.hold()
            return None

        self.max_deflection_mm = max(self.max_deflection_mm, state.tactile_mm)

        # Only measure while parked. See the module docstring.
        settling = state.t < self._settling_until
        if settling:
            ctx.base.hold()
            return None
        self.watcher.add(state.t, state.tactile_mm)

        if not self.watcher.full:
            ctx.base.hold()
            return None

        amplitude = self.watcher.amplitude
        grew = (
            self.previous_amplitude is None
            or (amplitude - self.previous_amplitude) > cfg.amplitude_tol_mm
        )
        clipped = ctx.geometry.is_tactile_clipped(self.watcher.trough) or (
            self.watcher.peak >= ctx.geometry.tactile_saturation_mm - 1e-6
        )

        if not grew and not clipped:
            self.stable_count += 1
            ctx.log(
                "approach_amplitude_stable",
                {"amplitude_mm": amplitude, "count": self.stable_count,
                 "trough_mm": self.watcher.trough, "peak_mm": self.watcher.peak},
            )
            if self.stable_count >= cfg.stable_increments:
                skin_min = ctx.geometry.skin_x_from_deflection(
                    state.base_mm, self.watcher.peak
                )
                ctx.skin_x_at_max_inhale = skin_min
                ctx.log(
                    "approach_seated",
                    {"amplitude_mm": amplitude, "increments": self.increments,
                     "skin_x_at_max_inhale": skin_min, "base_mm": state.base_mm},
                )
                return self._begin(ctx, Step.STANDOFF)
        else:
            self.stable_count = 0

        self.previous_amplitude = amplitude

        if self.increments >= cfg.max_seat_increments:
            saturation = ctx.geometry.tactile_saturation_mm
            return Transition(
                ProcedureState.FAULT,
                f"tactile amplitude never settled after {self.increments} increments "
                f"(last {amplitude:.3f} mm, trough {self.watcher.trough:.3f} mm, peak "
                f"{self.watcher.peak:.3f} mm against {saturation:g} mm of sensor stroke). "
                "If the peak is at saturation while the trough is at zero, the breathing "
                "excursion is larger than the sensor's usable stroke and no seating depth "
                "can show the whole waveform -- that is a hardware requirement, not a "
                "tuning problem. Otherwise amplitude_tol_mm may be tighter than the "
                "sensor noise.",
            )

        # Advance only at a trough, when the skin is furthest away.
        if not self._at_trough(ctx, state.tactile_mm):
            ctx.base.hold()
            return None

        ctx.base.move_to(state.base_mm + cfg.creep_increment_mm,
                         v_max_mm_s=cfg.creep_speed_mm_s)
        self.increments += 1
        self._settling_until = state.t + self._settle_time(cfg)
        self.watcher.reset()
        ctx.log("approach_increment", {"n": self.increments, "amplitude_mm": amplitude})
        return None

    def _at_trough(self, ctx: ProcedureContext, tactile_mm: float) -> bool:
        """Whether the breath is near its shallowest, with the skin furthest away.

        Judged against the amplitude window's own range rather than a model, because
        during APPROACH there is no model yet — that is what ESTIMATE is for.
        """
        assert self.watcher is not None
        span = self.watcher.amplitude
        if span <= 0:
            return True
        return tactile_mm <= self.watcher.trough + 0.2 * span

    @staticmethod
    def _settle_time(cfg) -> float:
        """How long to wait after commanding a creep increment before measuring again."""
        return max(cfg.creep_increment_mm / max(cfg.creep_speed_mm_s, 1e-6), 0.05) + 0.2

    # -- standoff -------------------------------------------------------------

    def _standoff(self, ctx: ProcedureContext) -> Transition | None:
        cfg = ctx.config.approach
        if ctx.skin_x_at_max_inhale is None:
            return Transition(
                ProcedureState.FAULT, "seating finished without recording maximum inhale"
            )

        target_tip_x = ctx.skin_x_at_max_inhale - cfg.standoff_mm
        needle_mm = ctx.geometry.needle_mm_for_tip_at(target_tip_x, ctx.state.base_mm)
        lo, hi = ctx.geometry.needle.travel_mm
        if not lo <= needle_mm <= hi:
            return Transition(
                ProcedureState.FAULT,
                f"standoff needs needle at {needle_mm:.2f} mm, outside its travel "
                f"[{lo:.1f}, {hi:.1f}] mm. Check rig.geometry.needle_tip_offset_mm — the "
                "needle cannot reach the skin from where it is mounted.",
            )

        ctx.needle.move_to(needle_mm, v_max_mm_s=cfg.creep_speed_mm_s * 5.0)
        ctx.base.hold()

        if abs(ctx.state.needle_mm - needle_mm) > 0.1:
            return None

        ctx.log(
            "approach_standoff_reached",
            {"needle_mm": needle_mm, "standoff_mm": cfg.standoff_mm,
             "tip_x": ctx.geometry.needle_tip_x(ctx.state.base_mm, ctx.state.needle_mm)},
        )
        if cfg.breath_hold_s > 0:
            return self._begin(ctx, Step.BREATH_HOLD)
        return Transition(ProcedureState.ESTIMATE, "seated at standoff")

    # -- breath hold ----------------------------------------------------------

    def _breath_hold(self, ctx: ProcedureContext) -> Transition | None:
        cfg = ctx.config.approach
        ctx.base.hold()
        ctx.needle.hold()
        elapsed = ctx.state.t - self.step_started_at
        if elapsed < cfg.breath_hold_s:
            return None
        ctx.breath_hold_window = (self.step_started_at, ctx.state.t)
        ctx.log("approach_breath_hold_done", {"window": ctx.breath_hold_window})
        return Transition(
            ProcedureState.ESTIMATE, f"seated at standoff after {cfg.breath_hold_s:g} s breath hold"
        )

    # -- plumbing -------------------------------------------------------------

    def _begin(self, ctx: ProcedureContext, step: Step) -> None:
        self.step = step
        self.step_started_at = ctx.state.t
        self.stable_count = 0
        self.previous_amplitude = None
        if self.watcher is not None:
            self.watcher.reset()
        ctx.base.hold()
        ctx.log("approach_step", {"step": step.value, "t": ctx.state.t})
        return None

    def _timeout_detail(self, ctx: ProcedureContext) -> str:
        return f"still in sub-step '{self.step.value}' after {self.increments} increments"
