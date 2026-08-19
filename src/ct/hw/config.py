"""Typed views over the ``rig``, ``procedure``, ``latency`` and ``servo`` config sections.

:class:`ct.config.RunConfig` keeps these as plain dicts so ``--set`` and the YAML
round-trip stay simple. This module turns them into dataclasses at the point of use, which
is where a missing key should become a clear error rather than a ``KeyError`` four states
into a run.

Everything here validates on construction. A rig config that builds is a rig config whose
numbers are at least self-consistent — whether they are *correct* is what
:mod:`ct.unknowns` is for.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from ct.geometry import RigGeometry


def _require(raw: dict[str, Any], key: str, where: str) -> Any:
    if key not in raw:
        raise ValueError(f"{where} needs a '{key}'")
    return raw[key]


def _unexpected(raw: dict[str, Any], known: set[str], where: str) -> None:
    extra = set(raw) - known
    if extra:
        raise ValueError(f"{where}: unknown key(s) {sorted(extra)}. Known: {sorted(known)}")


@dataclass(frozen=True)
class BusConfig:
    """One CAN interface.

    The rig has two: ``sensing`` (ToF, tactile, base motor, needle motor) and ``phantom``
    (the breathing phantom's motor, driven by a separate process).
    """

    backend: str = "loopback"
    """``loopback`` for simulation, ``rh02`` for the USB-CAN-FD adapter."""

    interface: str = "socketcan"
    """python-can backend name. Ignored by the loopback bus."""

    channel: str = "can0"
    bitrate: int = 1_000_000
    """Arbitration bitrate. CubeMars AK motors are 1 Mbit/s classic CAN."""

    fd: bool = False
    """Whether to open the link in CAN FD mode.

    The RH02 is FD-capable but the CubeMars motors are not: the motor bus runs classic
    CAN 2.0. Only turn this on for a bus whose devices are genuinely FD.
    """

    data_bitrate: int = 2_000_000
    """FD data-phase bitrate. Only meaningful when ``fd`` is set."""

    rx_queue: int = 4096
    """Frames the receive thread may buffer before dropping the oldest."""

    def __post_init__(self) -> None:
        if self.bitrate <= 0:
            raise ValueError("bitrate must be positive")
        if self.fd and self.data_bitrate < self.bitrate:
            raise ValueError("data_bitrate must be at least bitrate on an FD link")

    @classmethod
    def from_dict(cls, raw: dict[str, Any], name: str = "bus") -> BusConfig:
        _unexpected(raw, set(cls.__dataclass_fields__), f"rig.buses.{name}")
        return cls(**raw)


@dataclass(frozen=True)
class MotorLimits:
    tau_max: float = 2.0
    v_max: float = 20.0
    i_max: float = 20.0

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> MotorLimits:
        _unexpected(raw, set(cls.__dataclass_fields__), "motor limits")
        return cls(**{k: float(v) for k, v in raw.items()})


@dataclass(frozen=True)
class MotorConfig:
    """One CubeMars motor on a named bus."""

    bus: str = "sensing"
    codec: str = "cubemars_mit"
    """``cubemars_mit`` or ``cubemars_servo``. See the unknowns entry — MIT mode is the
    one that can float, and ADVANCE needs float."""

    can_id: int = 1
    gear_ratio: float = 9.0
    status_rate_hz: float = 200.0
    stale_after_s: float = 0.1
    """No feedback for this long and the axis reports stale, which trips the watchdog."""

    codec_params: dict[str, Any] = field(default_factory=dict)
    """Constructor arguments for the codec, chiefly its full-scale ranges.

    These matter more than they look. MIT mode packs position into 16 bits spanning
    ``[p_min, p_max]``, and the AK default of ±12.5 rad is only about two output-shaft
    turns — often less than a linear axis's full travel once a lead screw is involved. A
    command past the end of the range does not error, it *saturates*, so the axis simply
    stops short and every diagnosis points somewhere else. Set the range from the motor's
    configuration, and check that ``travel_mm * counts_per_mm`` fits inside it.
    """

    limits: MotorLimits = field(default_factory=MotorLimits)
    gains: dict[str, float] = field(default_factory=lambda: {"kp": 50.0, "kd": 2.0})
    """Position-hold gains for ordinary commanded motion."""

    float_gains: dict[str, float] = field(default_factory=lambda: {"kp": 0.0, "kd": 0.1})
    """Backdrive gains. ``kp`` must be zero or the needle will fight the tissue."""

    def __post_init__(self) -> None:
        if self.can_id <= 0:
            raise ValueError(f"can_id must be positive, got {self.can_id}")
        if self.stale_after_s <= 0:
            raise ValueError("stale_after_s must be positive")
        if self.float_gains.get("kp", 0.0) != 0.0:
            raise ValueError(
                f"float_gains.kp must be 0 for genuine backdrive, got {self.float_gains['kp']}. "
                "A non-zero stiffness means the needle drags tissue instead of riding with it."
            )

    @classmethod
    def from_dict(cls, raw: dict[str, Any], name: str = "motor") -> MotorConfig:
        raw = dict(raw)
        _unexpected(raw, set(cls.__dataclass_fields__), f"rig.motors.{name}")
        if "limits" in raw:
            raw["limits"] = MotorLimits.from_dict(raw["limits"])
        for key in ("gains", "float_gains"):
            if key in raw:
                raw[key] = {k: float(v) for k, v in raw[key].items()}
        return cls(**raw)


@dataclass(frozen=True)
class FrameLayout:
    """Where a scalar lives inside a CAN frame's payload."""

    offset: int = 0
    length: int = 2
    byteorder: str = "little"
    signed: bool = False
    scale: float = 1.0
    """Applied on decode, before geometry's counts-to-mm. Use it for a sensor that
    reports in its own fixed-point units."""

    def __post_init__(self) -> None:
        if self.byteorder not in ("little", "big"):
            raise ValueError(f"byteorder must be 'little' or 'big', got {self.byteorder!r}")
        if not 1 <= self.length <= 8:
            raise ValueError(f"length must be 1..8 bytes, got {self.length}")
        if self.offset < 0 or self.offset + self.length > 64:
            raise ValueError(f"offset {self.offset}+{self.length} outside a CAN FD payload")

    def decode(self, data: bytes) -> float | None:
        """Pull the scalar out of a payload, or ``None`` if the frame is too short."""
        end = self.offset + self.length
        if len(data) < end:
            return None
        raw = int.from_bytes(data[self.offset : end], self.byteorder, signed=self.signed)
        return raw * self.scale

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> FrameLayout:
        _unexpected(raw, set(cls.__dataclass_fields__), "frame layout")
        return cls(**raw)


@dataclass(frozen=True)
class SensorConfig:
    bus: str = "sensing"
    can_id: int = 0x100
    rate_hz: float = 100.0
    stale_after_s: float = 0.1
    latency_s: float = 0.0
    """Known transport/processing delay of this sensor. Feeds ``tau_s``."""

    layout: FrameLayout = field(default_factory=FrameLayout)

    def __post_init__(self) -> None:
        if self.rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        if self.stale_after_s <= 0:
            raise ValueError("stale_after_s must be positive")
        if self.latency_s < 0:
            raise ValueError("latency_s must be non-negative")

    @classmethod
    def from_dict(cls, raw: dict[str, Any], name: str = "sensor") -> SensorConfig:
        raw = dict(raw)
        _unexpected(raw, set(cls.__dataclass_fields__), f"rig.sensors.{name}")
        if "layout" in raw:
            raw["layout"] = FrameLayout.from_dict(raw["layout"])
        return cls(**raw)


@dataclass(frozen=True)
class RigConfig:
    """The physical rig: buses, motors, sensors, and where everything is."""

    buses: dict[str, BusConfig]
    motors: dict[str, MotorConfig]
    sensors: dict[str, SensorConfig]
    geometry: RigGeometry

    def __post_init__(self) -> None:
        for name, motor in self.motors.items():
            if motor.bus not in self.buses:
                raise ValueError(
                    f"motor '{name}' is on bus '{motor.bus}', which is not in rig.buses "
                    f"({sorted(self.buses)})"
                )
        for name, sensor in self.sensors.items():
            if sensor.bus not in self.buses:
                raise ValueError(
                    f"sensor '{name}' is on bus '{sensor.bus}', which is not in rig.buses "
                    f"({sorted(self.buses)})"
                )
        # Two devices answering to one ID on one bus is a wiring mistake that presents as
        # baffling intermittent readings. Catch it at config time.
        seen: dict[tuple[str, int], str] = {}
        for name, sensor in self.sensors.items():
            slot = (sensor.bus, sensor.can_id)
            if slot in seen:
                raise ValueError(
                    f"sensors '{seen[slot]}' and '{name}' share CAN id {sensor.can_id} on bus "
                    f"'{sensor.bus}'"
                )
            seen[slot] = name

    @property
    def is_simulated(self) -> bool:
        """True when every bus is a loopback — i.e. no hardware is involved."""
        return all(b.backend == "loopback" for b in self.buses.values())

    @classmethod
    def from_dict(cls, raw: dict[str, Any] | None) -> RigConfig:
        if not raw:
            raise ValueError("no 'rig' section in this config; a rig run needs one")
        _unexpected(raw, {"buses", "motors", "sensors", "geometry"}, "rig")
        buses = {n: BusConfig.from_dict(v, n) for n, v in _require(raw, "buses", "rig").items()}
        motors = {n: MotorConfig.from_dict(v, n) for n, v in _require(raw, "motors", "rig").items()}
        sensors = {n: SensorConfig.from_dict(v, n) for n, v in _require(raw, "sensors", "rig").items()}
        geometry = RigGeometry.from_dict(_require(raw, "geometry", "rig"))
        return cls(buses=buses, motors=motors, sensors=sensors, geometry=geometry)


# ---------------------------------------------------------------------------
# Procedure
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ApproachConfig:
    coarse_standoff_mm: float = 30.0
    """ToF distance at which the coarse drive stops and creeping begins."""

    coarse_speed_mm_s: float = 10.0
    creep_speed_mm_s: float = 1.0
    creep_increment_mm: float = 0.5
    """How far to seat per step, once in contact."""

    min_breaths: float = 2.0
    """Breaths to watch after each seating increment before judging the amplitude."""

    amplitude_tol_mm: float = 0.2
    """Amplitude growth below this between increments counts as 'stopped growing'."""

    stable_increments: int = 2
    """Consecutive non-growing increments required before the seat is accepted."""

    max_seat_increments: int = 40
    standoff_mm: float = 2.0
    """Final gap between needle tip and skin at maximum inhale."""

    breath_hold_s: float = 0.0
    """Optional pause with the phantom held still, to measure sensor noise directly.

    Worth having. `CLAUDE.md` records that the residual-variance fallback for ``R`` is
    only an upper bound and collapses NIS to ~0.027 on a drifting fundamental; a
    breath-hold segment gives the honest ``R`` instead, and APPROACH is exactly where it
    is cheapest to take — the base is parked and the sensor is already seated. Zero
    disables it.
    """

    nominal_breath_s: float = 4.0
    """Expected breath period, used only to size the observation windows before the
    estimator exists to say otherwise."""

    timeout_s: float = 300.0

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> ApproachConfig:
        _unexpected(raw, set(cls.__dataclass_fields__), "procedure.approach")
        return cls(**raw)


@dataclass(frozen=True)
class EstimateConfig:
    calib_seconds: float = 60.0
    """Signal collected before Stage 1 identification runs."""

    settle_breaths: float = 3.0
    """Breaths the EKF must run after init before convergence is even considered."""

    nis_band: tuple[float, float] = (0.2, 3.0)
    """Acceptable windowed-mean NIS. Outside it, the filter is not consistent."""

    nis_window: int = 200
    max_forecast_std_mm: float = 1.0
    """Forecast standard deviation at the working horizon required to move on."""

    timeout_s: float = 300.0

    def __post_init__(self) -> None:
        lo, hi = self.nis_band
        if not 0 < lo < hi:
            raise ValueError(f"nis_band must be (lo, hi) with 0 < lo < hi, got {self.nis_band}")

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> EstimateConfig:
        raw = dict(raw)
        _unexpected(raw, set(cls.__dataclass_fields__), "procedure.estimate")
        if "nis_band" in raw:
            raw["nis_band"] = tuple(float(v) for v in raw["nis_band"])
        return cls(**raw)


@dataclass(frozen=True)
class InsertConfig:
    initial_depth_mm: float = 10.0
    """How far past the skin the first drive goes."""

    drive_speed_mm_s: float = 50.0
    exhale_band_frac: float = 0.15
    """How near the bottom of the breathing excursion counts as end-exhale. See
    :mod:`ct.control.gate` for why this is a fraction of excursion rather than an angle."""

    max_forecast_std_mm: float = 0.5
    track_standoff: bool = True
    """Whether to hold standoff against the moving skin while waiting to fire."""

    arrival_tol_mm: float = 0.5
    """How close to the commanded position counts as arrived.

    Cannot be arbitrarily tight. Under MIT-mode impedance control the needle settles with
    a steady-state error of roughly ``tissue_force / kp`` — with a 60 N-ish grip and
    ``kp`` of 80 that is nearly 1 mm, so a 0.25 mm tolerance is simply never met and the
    state waits out its timeout instead. Either widen this, or raise ``kp`` until the
    error fits inside it; the stall detector below covers the rest.
    """

    stall_velocity_mm_s: float = 0.3
    stall_time_s: float = 0.4
    """A needle that has stopped moving has arrived, wherever it got to.

    Distinguishing "pushed as far as the tissue allows" from "still travelling" is what
    keeps a loaded insertion from looking like a hang.
    """

    timeout_s: float = 120.0

    def __post_init__(self) -> None:
        if not 0 < self.exhale_band_frac <= 1:
            raise ValueError(f"exhale_band_frac must be in (0, 1], got {self.exhale_band_frac}")

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> InsertConfig:
        _unexpected(raw, set(cls.__dataclass_fields__), "procedure.insert")
        return cls(**raw)


@dataclass(frozen=True)
class AdvanceConfig:
    total_depth_mm: float = 40.0
    """Final target depth past the skin surface."""

    increment_mm: float = 2.0
    """Advance per breath. The needle floats between increments."""

    increment_speed_mm_s: float = 20.0
    exhale_band_frac: float = 0.15
    max_forecast_std_mm: float = 0.5
    settle_s: float = 0.3
    """Time to hold position after an increment before returning to float."""

    max_increments: int = 100
    timeout_s: float = 600.0

    def __post_init__(self) -> None:
        if self.increment_mm <= 0:
            raise ValueError("increment_mm must be positive")
        if not 0 < self.exhale_band_frac <= 1:
            raise ValueError(f"exhale_band_frac must be in (0, 1], got {self.exhale_band_frac}")

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> AdvanceConfig:
        _unexpected(raw, set(cls.__dataclass_fields__), "procedure.advance")
        return cls(**raw)


@dataclass(frozen=True)
class SafetyConfig:
    """Limits that stop the run rather than shaping it."""

    max_tactile_mm: float = 15.0
    """Deflection above this means the sensor is being crushed into the phantom."""

    watchdog_s: float = 0.2
    """Longest tolerable gap in sensor feedback before faulting."""

    max_nis: float = 25.0
    """Innovation this far outside expectation means the model no longer describes the
    signal — the residual monitor's trip point."""

    nis_trip_count: int = 10
    """Consecutive out-of-range innovations required to fault, so one outlier is survivable."""

    max_deadline_misses: int = 50
    require_contact_to_insert: bool = True
    """Refuse to drive the needle without tactile contact. Rarely worth disabling."""

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> SafetyConfig:
        _unexpected(raw, set(cls.__dataclass_fields__), "procedure.safety")
        return cls(**raw)


@dataclass(frozen=True)
class ClinicalConfig:
    """Requirements from outside engineering. Recorded here so they are versioned with
    the run rather than living in someone's notes."""

    tolerance_mm: float = 2.0
    target_amplitude_mm: float = 20.0

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> ClinicalConfig:
        _unexpected(raw, set(cls.__dataclass_fields__), "procedure.clinical")
        return cls(**raw)


@dataclass(frozen=True)
class ProcedureConfig:
    loop_rate_hz: float = 200.0
    approach: ApproachConfig = field(default_factory=ApproachConfig)
    estimate: EstimateConfig = field(default_factory=EstimateConfig)
    insert: InsertConfig = field(default_factory=InsertConfig)
    advance: AdvanceConfig = field(default_factory=AdvanceConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    clinical: ClinicalConfig = field(default_factory=ClinicalConfig)
    withdraw_speed_mm_s: float = 20.0
    retract_speed_mm_s: float = 20.0

    def __post_init__(self) -> None:
        if self.loop_rate_hz <= 0:
            raise ValueError("loop_rate_hz must be positive")
        if self.advance.total_depth_mm < self.insert.initial_depth_mm:
            raise ValueError(
                f"advance.total_depth_mm ({self.advance.total_depth_mm}) is less than "
                f"insert.initial_depth_mm ({self.insert.initial_depth_mm}): the first drive "
                "would already overshoot the final target"
            )

    @property
    def dt(self) -> float:
        return 1.0 / self.loop_rate_hz

    @classmethod
    def from_dict(cls, raw: dict[str, Any] | None) -> ProcedureConfig:
        raw = dict(raw or {})
        _unexpected(raw, set(cls.__dataclass_fields__), "procedure")
        builders = {
            "approach": ApproachConfig,
            "estimate": EstimateConfig,
            "insert": InsertConfig,
            "advance": AdvanceConfig,
            "safety": SafetyConfig,
            "clinical": ClinicalConfig,
        }
        for key, builder in builders.items():
            if key in raw:
                raw[key] = builder.from_dict(raw[key])
        return cls(**raw)


# ---------------------------------------------------------------------------
# Latency and servo
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LatencyConfig:
    """The measured parts of the forecast horizon.

    ``tau_cl`` is deliberately absent: it is computed from the servo design, varies with
    breathing rate, and is not a configurable constant. See :mod:`ct.control.servo`.
    """

    tau_s: float = 0.02
    """Sensor latency. Measurable with ``ct-compare``."""

    tau_c: float = 0.005
    """Fallback compute latency, used until the loop has measured its own."""

    T_ins: float = 0.15
    """Insertion duration."""

    tau_cl_fallback: float = 0.05
    """Used only when no servo design is configured, so a run is still possible."""

    measure_tau_c: bool = True
    """Prefer the loop's own measured p95 tick time over the configured ``tau_c``."""

    def __post_init__(self) -> None:
        for name in ("tau_s", "tau_c", "T_ins", "tau_cl_fallback"):
            if getattr(self, name) < 0:
                raise ValueError(f"latency.{name} must be non-negative")

    @classmethod
    def from_dict(cls, raw: dict[str, Any] | None) -> LatencyConfig:
        raw = dict(raw or {})
        _unexpected(raw, set(cls.__dataclass_fields__), "latency")
        return cls(**raw)


@dataclass(frozen=True)
class PlantModel:
    """Second-order approximation of an axis: ``K*wn^2 / (s^2 + 2*zeta*wn*s + wn^2)``."""

    K: float = 1.0
    wn: float = 60.0
    zeta: float = 0.7

    def __post_init__(self) -> None:
        if self.wn <= 0:
            raise ValueError("plant wn must be positive")
        if self.zeta <= 0:
            raise ValueError("plant zeta must be positive")

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> PlantModel:
        _unexpected(raw, set(cls.__dataclass_fields__), "servo plant")
        return cls(**{k: float(v) for k, v in raw.items()})


@dataclass(frozen=True)
class LeadCompensator:
    """``gain * (s + zero) / (s + pole)``, with ``pole > zero`` for phase lead."""

    zero: float = 8.0
    pole: float = 80.0
    gain: float = 150.0
    """Loop gain. Against a type-1 plant the crossover lands near ``gain * zero / pole``,
    so this is the knob that sets bandwidth — and therefore ``tau_cl``. At 150 the
    placeholder design crosses over near 63 rad/s with about 40 degrees of phase margin;
    pushing it to 250 buys a shorter lag and leaves 12 degrees, which is not a servo
    anyone should put a needle on."""

    def __post_init__(self) -> None:
        if self.zero <= 0 or self.pole <= 0:
            raise ValueError("lead zero and pole must be positive")
        if self.pole <= self.zero:
            raise ValueError(
                f"a lead compensator needs pole > zero, got zero={self.zero}, pole={self.pole}. "
                "As given this is a lag network and would add delay where the design wants lead."
            )

    @property
    def alpha(self) -> float:
        """``pole / zero`` — the lead ratio, which bounds the phase it can add."""
        return self.pole / self.zero

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> LeadCompensator:
        _unexpected(raw, set(cls.__dataclass_fields__), "servo lead")
        return cls(**{k: float(v) for k, v in raw.items()})


@dataclass(frozen=True)
class AxisServoConfig:
    plant: PlantModel = field(default_factory=PlantModel)
    lead: LeadCompensator = field(default_factory=LeadCompensator)

    @classmethod
    def from_dict(cls, raw: dict[str, Any], name: str = "axis") -> AxisServoConfig:
        raw = dict(raw)
        _unexpected(raw, set(cls.__dataclass_fields__), f"servo.{name}")
        if "plant" in raw:
            raw["plant"] = PlantModel.from_dict(raw["plant"])
        if "lead" in raw:
            raw["lead"] = LeadCompensator.from_dict(raw["lead"])
        return cls(**raw)


@dataclass(frozen=True)
class ServoConfig:
    needle: AxisServoConfig = field(default_factory=AxisServoConfig)

    @classmethod
    def from_dict(cls, raw: dict[str, Any] | None) -> ServoConfig:
        raw = dict(raw or {})
        _unexpected(raw, {"needle"}, "servo")
        if "needle" in raw:
            raw["needle"] = AxisServoConfig.from_dict(raw["needle"], "needle")
        return cls(**raw)


@dataclass(frozen=True)
class RigSession:
    """Everything a rig run needs, resolved from one :class:`~ct.config.RunConfig`."""

    rig: RigConfig
    procedure: ProcedureConfig
    latency: LatencyConfig
    servo: ServoConfig

    @classmethod
    def from_run_config(cls, cfg: Any) -> RigSession:
        return cls(
            rig=RigConfig.from_dict(cfg.rig),
            procedure=ProcedureConfig.from_dict(cfg.procedure),
            latency=LatencyConfig.from_dict(cfg.latency),
            servo=ServoConfig.from_dict(cfg.servo),
        )

    def to_dict(self) -> dict[str, Any]:
        """Round-trippable description, for the run summary."""
        return {
            "rig": {
                "buses": {n: asdict(b) for n, b in self.rig.buses.items()},
                "motors": {n: asdict(m) for n, m in self.rig.motors.items()},
                "sensors": {n: asdict(s) for n, s in self.rig.sensors.items()},
                "geometry": asdict(self.rig.geometry),
            },
            "procedure": asdict(self.procedure),
            "latency": asdict(self.latency),
            "servo": asdict(self.servo),
        }
