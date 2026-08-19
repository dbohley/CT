"""Time, as a swappable dependency.

The whole four-state procedure takes twenty minutes of breathing to run. Under
:class:`SimClock` the same control code — every state, every gate, every servo update —
runs in milliseconds, deterministically, with no hardware and no waiting. That is what
makes the procedure testable at all, so nothing in ``control/`` or ``rt/`` may call
:func:`time.monotonic` directly. It asks its clock.

``RealClock`` and ``SimClock`` differ in exactly one way: whether ``sleep_until`` waits or
merely assigns.
"""

from __future__ import annotations

import time


class RealClock:
    """Wall-clock time from :func:`time.monotonic`.

    Monotonic rather than wall time: NTP steps and DST have no business moving a control
    loop, and the phantom log has to align against this one across processes.
    """

    def __init__(self, spin_margin_s: float = 0.001) -> None:
        self.spin_margin_s = float(spin_margin_s)
        """How long before the deadline to stop sleeping and busy-wait.

        ``time.sleep`` typically overshoots by a millisecond or so, which at 200 Hz is a
        fifth of the period. Sleeping to just short of the deadline and spinning the rest
        costs a little CPU and buys back most of the jitter.
        """

    def now(self) -> float:
        return time.monotonic()

    def sleep_until(self, t: float) -> None:
        remaining = t - time.monotonic()
        if remaining <= 0:
            return
        if remaining > self.spin_margin_s:
            time.sleep(remaining - self.spin_margin_s)
        while time.monotonic() < t:
            pass

    @property
    def is_simulated(self) -> bool:
        return False

    def __repr__(self) -> str:
        return f"RealClock(spin_margin_s={self.spin_margin_s})"


class SimClock:
    """Fictional time that advances only when asked.

    Runs as fast as the CPU allows and gives byte-identical results every time, so a test
    can assert on exact transition times rather than on tolerances. The plant is stepped
    from the same clock, so simulated sensor dynamics stay consistent with it.
    """

    def __init__(self, t0: float = 0.0) -> None:
        self._t = float(t0)
        self.sleeps = 0
        """How many times the loop asked to wait. A loop that never sleeps is overrunning."""

    def now(self) -> float:
        return self._t

    def sleep_until(self, t: float) -> None:
        if t > self._t:
            self._t = float(t)
            self.sleeps += 1

    def advance(self, dt: float) -> float:
        """Push time forward by ``dt``. For tests that drive the clock themselves."""
        if dt < 0:
            raise ValueError(f"cannot advance time backwards by {dt}")
        self._t += float(dt)
        return self._t

    @property
    def is_simulated(self) -> bool:
        return True

    def __repr__(self) -> str:
        return f"SimClock(t={self._t:.6f})"


def build_clock(name: str, params: dict | None = None) -> RealClock | SimClock:
    """Resolve a clock by config name."""
    params = params or {}
    if name in ("real", "wall", "monotonic"):
        return RealClock(**params)
    if name in ("sim", "simulated", "fake"):
        return SimClock(**params)
    raise KeyError(f"unknown clock '{name}'. Registered: real, sim")
