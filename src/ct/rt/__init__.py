"""Real-time plumbing: the clock, the control loop, the latency budget, telemetry.

Split out from ``control/`` because none of it knows anything about needles. The loop
ticks, measures itself and writes records; what happens inside a tick is the controller's
business.
"""

from __future__ import annotations

from ct.rt.clock import RealClock, SimClock, build_clock

__all__ = ["RealClock", "SimClock", "build_clock"]
