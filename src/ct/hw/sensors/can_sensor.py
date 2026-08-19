"""CAN-framed scalar sensors: the time-of-flight rangefinder and the tactile sensor.

Both are the same thing structurally — a periodic frame carrying one number at a known
payload offset — so they share one implementation and differ only by config. What they
are *for* differs enormously:

- **ToF** guides the coarse approach and is discarded once contact is made.
- **Tactile** is the breathing signal. Everything the estimator does, and therefore every
  gate decision downstream, rests on this one scalar.

Readings come out in **raw counts**. Converting them to millimetres is
:mod:`ct.geometry`'s job (rule 2), which is why there is no scale factor here beyond the
frame layout's fixed-point ``scale``.
"""

from __future__ import annotations

from typing import Any

from ct.hw.config import SensorConfig
from ct.registry import register_sensor


@register_sensor("can_scalar")
class CANScalarSensor:
    """One scalar decoded from a periodic CAN frame."""

    def __init__(self, name: str, config: SensorConfig) -> None:
        self.name = name
        self.config = config
        self._counts: float | None = None
        self._stamp: float | None = None
        self._t: float = 0.0
        self.frames_seen = 0
        self.frames_malformed = 0

    def update(self, t: float, frames: list[tuple[float, int, bytes]]) -> None:
        """Take the newest matching frame in this tick's batch.

        Newest rather than each in turn: a control loop wants the current reading, and if
        several arrived in one tick the older ones describe a world that has moved on.
        """
        self._t = t
        for stamp, can_id, data in frames:
            if can_id != self.config.can_id:
                continue
            value = self.config.layout.decode(data)
            if value is None:
                # A short frame is a wiring or layout problem, not a transient. Counted
                # rather than raised so the run reaches its summary and says so.
                self.frames_malformed += 1
                continue
            self._counts = value
            self._stamp = stamp
            self.frames_seen += 1

    @property
    def counts(self) -> float | None:
        return self._counts

    @property
    def stamp(self) -> float | None:
        return self._stamp

    @property
    def age(self) -> float | None:
        return None if self._stamp is None else self._t - self._stamp

    @property
    def is_stale(self) -> bool:
        """True when the newest reading is older than this sensor's budget.

        A sensor that has never reported counts as stale, which is correct at startup and
        saves every caller a ``None`` check.
        """
        age = self.age
        return age is None or age > self.config.stale_after_s

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "can_id": self.config.can_id,
            "counts": self._counts,
            "age": self.age,
            "stale": self.is_stale,
            "frames_seen": self.frames_seen,
            "frames_malformed": self.frames_malformed,
        }

    def __repr__(self) -> str:
        return f"CANScalarSensor({self.name!r}, id=0x{self.config.can_id:X}, counts={self._counts})"
