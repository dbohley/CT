"""Hardware layer: CAN transport, motor codecs, axes and sensors.

Importing this package registers the built-in backends, the same way importing
``ct.sources`` registers the built-in signal sources. It does *not* import python-can —
the real bus defers that to its constructor — so the whole simulated stack, and all of
its tests, work on a machine with no CAN adapter and no driver installed.
"""

from __future__ import annotations

from ct.hw.config import (
    BusConfig,
    MotorConfig,
    ProcedureConfig,
    RigConfig,
    RigSession,
    SensorConfig,
)
from ct.hw.interfaces import Axis, Bus, Clock, MotorCodec, Sensor

__all__ = [
    "Bus",
    "MotorCodec",
    "Axis",
    "Sensor",
    "Clock",
    "BusConfig",
    "MotorConfig",
    "SensorConfig",
    "RigConfig",
    "ProcedureConfig",
    "RigSession",
]

# Registry population. Keep last: these modules import the names above.
import ct.hw.bus  # noqa: E402,F401
import ct.hw.motors  # noqa: E402,F401
import ct.hw.sensors  # noqa: E402,F401
