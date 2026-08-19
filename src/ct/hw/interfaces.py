"""The hardware swap boundaries.

Structural :class:`typing.Protocol` rather than base classes, exactly as
:mod:`ct.interfaces` does for the estimator: a replacement needs matching methods and
nothing else. Combined with the registry, that means switching from the simulated rig to
the real one is a config string, not a code change.

    Bus        -- moves CAN frames. `loopback` (sim) or `rh02` (the real adapter).
    MotorCodec -- packs commands into frames and parses feedback. Per motor firmware.
    Axis       -- a joint that moves, in millimetres. `can_axis` or `sim_axis`.
    Sensor     -- produces one scalar reading, in raw counts.

Every one of these is crossed by the same control code. If a procedure state can tell
which implementation is underneath it, the abstraction has failed and the simulation is
not evidence about the real rig.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

from ct.geometry import AxisCalibration


@runtime_checkable
class Bus(Protocol):
    """A CAN bus. Send is fire-and-forget; receive is polled, never blocking."""

    name: str

    def send(self, can_id: int, data: bytes, *, extended: bool = False) -> None:
        """Queue one frame. Must not block the control tick (rule 4)."""
        ...

    def poll(self) -> list[tuple[float, int, bytes]]:
        """Drain everything that arrived since the last call, as ``(t, can_id, data)``.

        ``t`` is the host monotonic timestamp at arrival, which is what makes a controller
        log comparable against a phantom log written by a different process.
        """
        ...

    def close(self) -> None: ...

    @property
    def stats(self) -> dict[str, Any]:
        """Frame counts, drops, and errors — for the run summary and the watchdog."""
        ...


@runtime_checkable
class MotorCodec(Protocol):
    """Wire format for one motor firmware.

    Positions in and out of a codec are in the motor's own raw units. Converting those to
    millimetres is :mod:`ct.geometry`'s job and nobody else's (rule 2).
    """

    name: str

    def enable(self, can_id: int) -> tuple[int, bytes, bool]:
        """Frame that puts the motor into closed-loop control. ``(id, data, extended)``."""
        ...

    def disable(self, can_id: int) -> tuple[int, bytes, bool]: ...

    def zero(self, can_id: int) -> tuple[int, bytes, bool]:
        """Frame that declares the present position to be zero."""
        ...

    def command(
        self,
        can_id: int,
        *,
        position: float | None = None,
        velocity: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque: float = 0.0,
    ) -> tuple[int, bytes, bool]:
        """Frame for one control command, in raw motor units.

        ``kp=0, kd=small, torque=0`` is the float that ADVANCE relies on. A codec whose
        firmware cannot express that must say so via :attr:`supports_float`.
        """
        ...

    def parse(self, can_id: int, data: bytes) -> dict[str, float] | None:
        """Decode a feedback frame into ``{position, velocity, current, ...}``.

        Returns ``None`` for frames that are not this codec's feedback, so several codecs
        can share a bus without fighting over IDs.
        """
        ...

    @property
    def supports_float(self) -> bool:
        """Whether commanding genuine zero-stiffness backdrive is possible."""
        ...


@runtime_checkable
class Axis(Protocol):
    """One joint, in millimetres, in the rig frame's sign convention.

    Everything above this line is engineering units; everything below is raw counts.
    """

    name: str
    calibration: AxisCalibration

    def enable(self) -> None: ...

    def disable(self) -> None: ...

    def update(self, t: float, frames: list[tuple[float, int, bytes]]) -> None:
        """Fold this tick's frames into the axis's view of itself."""
        ...

    def hold(self) -> None:
        """Command the axis to stay where it is. The safe default every tick."""
        ...

    def move_to(self, mm: float, *, v_max_mm_s: float | None = None) -> None:
        """Command an absolute position, honouring soft limits."""
        ...

    def float_free(self) -> None:
        """Zero stiffness: let the axis be pushed around by whatever it is touching.

        ADVANCE is built on this. Raises on an axis whose firmware cannot do it.
        """
        ...

    @property
    def position_mm(self) -> float: ...

    @property
    def velocity_mm_s(self) -> float: ...

    @property
    def is_stale(self) -> bool:
        """True when no feedback has arrived recently enough to be trusted."""
        ...


@runtime_checkable
class Sensor(Protocol):
    """One scalar reading, in raw counts. Conversion to mm belongs to geometry."""

    name: str

    def update(self, t: float, frames: list[tuple[float, int, bytes]]) -> None: ...

    @property
    def counts(self) -> float | None:
        """Latest reading, or ``None`` if nothing has arrived yet."""
        ...

    @property
    def stamp(self) -> float | None:
        """Host monotonic time the latest reading arrived."""
        ...

    @property
    def is_stale(self) -> bool: ...


@runtime_checkable
class Clock(Protocol):
    """Time, so that the control loop can be run flat out in a test.

    The single most useful abstraction in the hardware layer: swapping ``RealClock`` for
    ``SimClock`` turns a twenty-minute procedure into a millisecond unit test running the
    identical control code.
    """

    def now(self) -> float:
        """Monotonic seconds. Only differences are meaningful."""
        ...

    def sleep_until(self, t: float) -> None:
        """Block (or, in simulation, simply advance) until ``now() >= t``."""
        ...

    @property
    def is_simulated(self) -> bool:
        """True when time is fictional — gates the hardware placeholder check."""
        ...
