"""Limits, watchdogs, and the residual monitor.

The third of the three things ``CLAUDE.md`` deferred from session 001, alongside the servo
and the gate. It answers one question every tick — *is it still safe to be doing this* —
and the answer is checked before any command is issued, not after.

Four kinds of trip, deliberately different in character:

- **Soft limits** are enforced at the axis and are a last resort; what shows up here is a
  *rate* of violations, meaning the controller keeps asking for impossible things.
- **The watchdog** catches a bus that has gone quiet. A stalled sensor does not announce
  itself: the readings simply stop changing, which looks exactly like a patient holding
  still. Staleness is the only reliable way to tell those apart.
- **The tactile limit** catches the sensor being crushed into the phantom.
- **The residual monitor** catches the model having stopped describing reality. NIS is
  already computed every step by the EKF; a sustained run of large innovations means the
  breathing has changed character (a cough, a shift in position) and the forecast the gate
  is trusting is no longer trustworthy. One outlier is survivable — a run of them is not,
  hence ``nis_trip_count``.

A trip is latching. Recovering automatically from a fault would mean deciding, in
software, that whatever went wrong has stopped mattering; with a needle in tissue that is
an operator's decision.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ct.control.state import ProcedureState
from ct.geometry import RigState
from ct.hw.config import SafetyConfig


@dataclass(frozen=True)
class Trip:
    """A safety condition that fired."""

    kind: str
    detail: str
    t: float

    def __str__(self) -> str:
        return f"[{self.kind}] {self.detail}"


@dataclass
class SafetyMonitor:
    """Checked every tick, before any command is issued."""

    config: SafetyConfig
    trips: list[Trip] = field(default_factory=list)
    _nis_run: int = 0
    _worst_nis: float = 0.0
    _max_tactile_seen: float = 0.0

    @property
    def tripped(self) -> bool:
        return bool(self.trips)

    @property
    def first_trip(self) -> Trip | None:
        return self.trips[0] if self.trips else None

    def trip(self, kind: str, detail: str, t: float) -> Trip:
        """Record a trip. Latching: the first one is the one that explains the run."""
        record = Trip(kind=kind, detail=detail, t=t)
        self.trips.append(record)
        return record

    def check(
        self,
        state: RigState,
        procedure_state: ProcedureState,
        *,
        deadline_misses: int = 0,
    ) -> Trip | None:
        """Run every check. Returns the first trip, or ``None`` if all clear."""
        if self.tripped:
            return self.first_trip

        # Staleness is checked only for states that act on the reading. During APPROACH's
        # coarse drive the tactile sensor legitimately has nothing to say, and faulting on
        # that would make the procedure impossible to start.
        needs_tactile = procedure_state in (
            ProcedureState.ESTIMATE,
            ProcedureState.INSERT,
            ProcedureState.ADVANCE,
        )
        if needs_tactile and "tactile" in state.stale:
            return self.trip(
                "watchdog",
                f"tactile sensor stale during {procedure_state.value}; no fresh reading within "
                f"{self.config.watchdog_s:g} s",
                state.t,
            )

        if state.tactile_mm is not None:
            self._max_tactile_seen = max(self._max_tactile_seen, state.tactile_mm)
            if state.tactile_mm > self.config.max_tactile_mm:
                return self.trip(
                    "tactile_limit",
                    f"tactile deflection {state.tactile_mm:.2f} mm exceeds "
                    f"{self.config.max_tactile_mm:.2f} mm; the sensor is being pressed into "
                    "the phantom",
                    state.t,
                )

        if deadline_misses > self.config.max_deadline_misses:
            return self.trip(
                "timing",
                f"{deadline_misses} missed control deadlines exceeds "
                f"{self.config.max_deadline_misses}; the loop is not keeping up",
                state.t,
            )

        return None

    def check_residual(self, t: float, nis: float) -> Trip | None:
        """The residual monitor. Fed the EKF's NIS every tracked step.

        Counts *consecutive* excursions rather than a total: an isolated large innovation
        is a sensor glitch, while a run of them means the model no longer fits.
        """
        self._worst_nis = max(self._worst_nis, nis)
        if nis > self.config.max_nis:
            self._nis_run += 1
            if self._nis_run >= self.config.nis_trip_count:
                return self.trip(
                    "residual",
                    f"{self._nis_run} consecutive innovations above NIS {self.config.max_nis:g} "
                    f"(worst {self._worst_nis:.1f}); the tracked model no longer describes the "
                    "signal, so the forecast driving the gate cannot be trusted",
                    t,
                )
        else:
            self._nis_run = 0
        return None

    def check_insert_preconditions(self, state: RigState, t: float) -> Trip | None:
        """Refuse to drive the needle without contact, unless explicitly configured not to."""
        if self.config.require_contact_to_insert and not state.in_contact:
            return self.trip(
                "no_contact",
                "needle motion requested without tactile contact; the skin position is "
                "unverified",
                t,
            )
        return None

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "tripped": self.tripped,
            "trips": [{"kind": t.kind, "detail": t.detail, "t": t.t} for t in self.trips],
            "worst_nis": self._worst_nis,
            "max_tactile_mm": self._max_tactile_seen,
        }
