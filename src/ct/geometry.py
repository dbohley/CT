"""Rig frames and unit conversion — the only place raw motor units become millimetres.

This module is to the hardware layer what :mod:`ct.layout` is to the state vector: the
*single source of truth*. No module outside it may multiply by ``counts_per_mm``, apply a
frame offset, or know a gear ratio. Above the ``Axis`` boundary everything speaks
millimetres in a named frame, so recalibrating the rig is a one-file change and a config
edit rather than a hunt through the controller.

Sign convention
---------------

**``+x`` points toward the phantom**, everywhere, without exception. The origin of the rig
frame is the base carriage's home position. Consequences worth stating out loud, because
every insertion sign error comes from getting one of them backwards:

- The base advances in ``+x``.
- The needle extends in ``+x``.
- **Inhale moves the skin toward the rig, so ``skin_x`` decreases.** Max inhale is the
  *minimum* of ``skin_x`` — which is why "clear the skin at max inhale" is the binding
  constraint for standoff, and why the needle chases the skin in ``+x`` during exhale.

Frames
------

Three features ride on the base carriage, each at a fixed offset from its origin::

    x = 0                                                    increasing x -->
    |                                                                        |
    [base carriage]                                                    [phantom]
        |<-- tof_offset_mm -->|  ToF reference plane
        |<-- tactile_offset_mm --->|  tactile contact face
        |<-- needle_tip_offset_mm -->|  needle tip at needle_x = 0

    tof_face_x    = base_x + tof_offset_mm
    tactile_face_x = base_x + tactile_offset_mm
    needle_tip_x  = base_x + needle_tip_offset_mm + needle_x

``needle_tip_offset_mm - tactile_offset_mm`` is the number everyone asks for first: how
far the needle tip sits behind the tactile face when the needle is fully retracted. It is
exposed as :attr:`RigGeometry.needle_tip_behind_tactile_mm` so nobody recomputes it.

The measurement
---------------

The tactile sensor reads how far its face has pressed into the skin, so with the face at
``tactile_face_x`` the skin surface is *behind* the contact point::

    skin_x = tactile_face_x - deflection_mm

That single line is why the estimator can be handed tactile deflection directly: with the
base parked, ``deflection_mm`` is an inverted, offset copy of ``skin_x``, and the harmonic
model fits it just as happily either way up. The controller, however, needs real skin
positions to aim the needle, so it converts back through :meth:`RigGeometry.skin_x`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

# Raw motor position units are whatever the codec reports -- radians of output shaft in
# CubeMars MIT mode, 0.1-degree increments in servo mode. Geometry does not care which,
# only that the unit is linear in travel. The team says "counts", so the config says
# counts, and the codec's unit is the axis's unit.


@dataclass(frozen=True)
class AxisCalibration:
    """Raw motor units <-> millimetres of linear travel for one axis."""

    counts_per_mm: float
    """Raw motor position units per mm of travel. Folds in lead-screw pitch and gear ratio."""

    direction: int = 1
    """``+1`` if increasing motor position moves the axis in ``+x`` (toward the phantom)."""

    zero_offset_counts: float = 0.0
    """Raw reading at the axis's mechanical zero, from homing."""

    travel_mm: tuple[float, float] = (0.0, 100.0)
    """Soft limits ``(min, max)`` in mm. Commands outside this range are refused."""

    v_max_mm_s: float = 50.0
    """Speed ceiling for commanded motion."""

    a_max_mm_s2: float = 500.0
    """Acceleration ceiling, for trapezoidal profiling."""

    def __post_init__(self) -> None:
        if self.counts_per_mm == 0:
            raise ValueError("counts_per_mm must be non-zero")
        if self.direction not in (1, -1):
            raise ValueError(f"direction must be +1 or -1, got {self.direction}")
        lo, hi = self.travel_mm
        if not lo < hi:
            raise ValueError(f"travel_mm must be (min, max) with min < max, got {self.travel_mm}")
        if self.v_max_mm_s <= 0 or self.a_max_mm_s2 <= 0:
            raise ValueError("v_max_mm_s and a_max_mm_s2 must be positive")

    # -- conversion -----------------------------------------------------------

    def to_mm(self, counts: float) -> float:
        """Raw motor position -> mm of travel from the mechanical zero."""
        return self.direction * (counts - self.zero_offset_counts) / self.counts_per_mm

    def to_counts(self, mm: float) -> float:
        """mm of travel from the mechanical zero -> raw motor position."""
        return self.zero_offset_counts + self.direction * mm * self.counts_per_mm

    def rate_to_mm_s(self, counts_per_s: float) -> float:
        """Raw motor velocity -> mm/s. No offset: a rate has no datum."""
        return self.direction * counts_per_s / self.counts_per_mm

    def rate_to_counts_s(self, mm_s: float) -> float:
        return self.direction * mm_s * self.counts_per_mm

    # -- limits ---------------------------------------------------------------

    def clamp(self, mm: float) -> float:
        """Nearest position inside the soft limits."""
        lo, hi = self.travel_mm
        return min(max(mm, lo), hi)

    def in_range(self, mm: float, tol: float = 0.0) -> bool:
        lo, hi = self.travel_mm
        return lo - tol <= mm <= hi + tol

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> AxisCalibration:
        raw = dict(raw)
        if "travel_mm" in raw:
            raw["travel_mm"] = tuple(float(v) for v in raw["travel_mm"])  # YAML gives a list
        if "direction" in raw:
            raw["direction"] = int(raw["direction"])
        return cls(**raw)


@dataclass(frozen=True)
class RigGeometry:
    """Where everything is, and how raw readings become millimetres.

    Construct from the ``rig.geometry`` section of a config. Every offset here is a
    physical measurement someone has to take with calipers; each one is an entry in
    :mod:`ct.unknowns` with a placeholder so simulation runs before they exist.
    """

    base: AxisCalibration
    needle: AxisCalibration

    tof_offset_mm: float = 0.0
    """Base-carriage origin -> ToF sensor reference plane, along ``+x``."""

    tactile_offset_mm: float = 0.0
    """Base-carriage origin -> tactile contact face, along ``+x``."""

    needle_tip_offset_mm: float = 0.0
    """Base-carriage origin -> needle tip when the needle axis reads zero, along ``+x``."""

    needle_free_length_mm: float = 100.0
    """Exposed needle length: how far the tip can travel before the hub fouls the sensor."""

    tactile_counts_to_mm: float = 1.0
    """Tactile raw counts -> mm of deflection. The whole estimator is downstream of this."""

    tactile_contact_counts: float = 0.0
    """Raw tactile level above which we are touching something.

    A pure threshold, not an offset: the sensor reads about zero in free air and rises on
    contact, so deflection is ``counts * tactile_counts_to_mm`` and this only decides
    whether that number means anything yet. Set it a few standard deviations above the
    free-air noise floor.
    """

    tactile_saturation_mm: float = 20.0
    """Deflection at which the tactile sensor stops responding. Clipping above this."""

    tof_counts_to_mm: float = 1.0
    """ToF raw counts -> mm of standoff distance from the ToF reference plane."""

    tof_range_mm: tuple[float, float] = (10.0, 1000.0)
    """Valid ToF measurement window. Readings outside it are dropped, not clamped."""

    def __post_init__(self) -> None:
        if self.tactile_counts_to_mm == 0:
            raise ValueError("tactile_counts_to_mm must be non-zero")
        if self.tof_counts_to_mm == 0:
            raise ValueError("tof_counts_to_mm must be non-zero")
        if self.needle_free_length_mm <= 0:
            raise ValueError("needle_free_length_mm must be positive")
        lo, hi = self.tof_range_mm
        if not lo < hi:
            raise ValueError(f"tof_range_mm must be (min, max) with min < max, got {self.tof_range_mm}")

    # -- derived offsets -------------------------------------------------------

    @property
    def needle_tip_behind_tactile_mm(self) -> float:
        """How far the retracted needle tip sits behind the tactile face.

        Negative means "behind", which is the safe and expected sign: at ``needle_x = 0``
        the tip should be tucked back from the face so seating the sensor cannot drive the
        needle into the phantom. This is the single measurement most often asked for.
        """
        return self.needle_tip_offset_mm - self.tactile_offset_mm

    @property
    def tof_behind_tactile_mm(self) -> float:
        """How far the ToF reference plane sits behind the tactile face."""
        return self.tof_offset_mm - self.tactile_offset_mm

    # -- frames ----------------------------------------------------------------

    def tactile_face_x(self, base_mm: float) -> float:
        """Rig-frame position of the tactile contact face."""
        return base_mm + self.tactile_offset_mm

    def tof_face_x(self, base_mm: float) -> float:
        """Rig-frame position of the ToF reference plane."""
        return base_mm + self.tof_offset_mm

    def needle_tip_x(self, base_mm: float, needle_mm: float) -> float:
        """Rig-frame position of the needle tip."""
        return base_mm + self.needle_tip_offset_mm + needle_mm

    # -- sensors -> rig frame ---------------------------------------------------

    def tactile_deflection_mm(self, counts: float) -> float:
        """Raw tactile counts -> mm the face has pressed into the skin."""
        return counts * self.tactile_counts_to_mm

    def in_contact(self, counts: float) -> bool:
        """Whether the tactile face is touching anything."""
        return counts > self.tactile_contact_counts

    def is_tactile_clipped(self, deflection_mm: float, tol_mm: float = 1e-6) -> bool:
        """Whether a reading is pinned at either end of the sensor's usable range.

        Both ends matter during APPROACH, for opposite reasons. Pinned at zero means the
        sensor is seated too far out and loses the skin at end-exhale, so the trough of
        the breathing waveform is cut off. Pinned at saturation means it is seated too
        deep and the peak is cut off. Only between the two is the full excursion visible,
        which is the condition the seating sub-step is searching for.
        """
        return deflection_mm <= tol_mm or deflection_mm >= self.tactile_saturation_mm - tol_mm

    def skin_x(self, base_mm: float, tactile_counts: float) -> float:
        """Rig-frame skin position from the base position and a raw tactile reading.

        The face is pressed *into* the skin, so the surface sits behind the contact point
        by the deflection. See the module docstring.
        """
        return self.tactile_face_x(base_mm) - self.tactile_deflection_mm(tactile_counts)

    def skin_x_from_deflection(self, base_mm: float, deflection_mm: float) -> float:
        """As :meth:`skin_x`, but from a deflection already in mm.

        The estimator tracks deflection in mm, so its forecasts arrive in those units and
        this is the conversion the controller actually calls.
        """
        return self.tactile_face_x(base_mm) - deflection_mm

    def tof_distance_mm(self, counts: float) -> float:
        """Raw ToF counts -> mm from the ToF reference plane to whatever it sees."""
        return counts * self.tof_counts_to_mm

    def skin_x_from_tof(self, base_mm: float, tof_counts: float) -> float:
        """Rig-frame skin position from a ToF reading. Used before tactile contact."""
        return self.tof_face_x(base_mm) + self.tof_distance_mm(tof_counts)

    def tof_in_range(self, counts: float) -> bool:
        lo, hi = self.tof_range_mm
        return lo <= self.tof_distance_mm(counts) <= hi

    # -- rig frame -> commands --------------------------------------------------

    def needle_mm_for_tip_at(self, tip_x: float, base_mm: float) -> float:
        """Needle-axis command that puts the tip at ``tip_x``, with the base where it is."""
        return tip_x - base_mm - self.needle_tip_offset_mm

    def base_mm_for_tactile_at(self, face_x: float) -> float:
        """Base-axis command that puts the tactile face at ``face_x``."""
        return face_x - self.tactile_offset_mm

    def insertion_depth_mm(self, base_mm: float, needle_mm: float, skin_x: float) -> float:
        """How far the tip is past the skin surface. Negative means still clear of it."""
        return self.needle_tip_x(base_mm, needle_mm) - skin_x

    # -- construction -----------------------------------------------------------

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> RigGeometry:
        raw = dict(raw)
        axes = {}
        for name in ("base", "needle"):
            spec = raw.pop(name, None)
            if spec is None:
                raise ValueError(f"rig.geometry needs a '{name}' axis calibration")
            axes[name] = AxisCalibration.from_dict(spec)
        if "tof_range_mm" in raw:
            raw["tof_range_mm"] = tuple(float(v) for v in raw["tof_range_mm"])
        return cls(**axes, **raw)


@dataclass
class RigState:
    """Where the rig is right now, in millimetres — the controller's view of the world.

    Assembled once per tick from the sensor mailbox and handed to the active procedure
    state. Everything here is already converted; no procedure state ever sees a raw count.
    """

    t: float
    """Loop time [s], monotonic."""

    base_mm: float = 0.0
    needle_mm: float = 0.0

    tactile_mm: float | None = None
    """Tactile deflection [mm], or ``None`` if the reading is stale or absent."""

    tof_mm: float | None = None
    """ToF standoff [mm] from its reference plane, or ``None`` if stale or out of range."""

    skin_x: float | None = None
    """Best available rig-frame skin position: tactile if in contact, else ToF."""

    in_contact: bool = False
    stale: tuple[str, ...] = field(default_factory=tuple)
    """Names of sensors whose most recent frame is older than their staleness budget."""

    def require_skin(self) -> float:
        """Skin position, or a clear error. Procedure states that need it call this."""
        if self.skin_x is None:
            raise RuntimeError(
                f"no skin position at t={self.t:.3f}s: tactile and ToF both unavailable "
                f"(stale: {self.stale or 'none'})"
            )
        return self.skin_x
