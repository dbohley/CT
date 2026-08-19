"""The fixed-rate control loop, and its accounting.

One tick is: poll the buses, hand frames to the axes and sensors, run the callback,
record how long it took, sleep to the next deadline. What happens inside the callback is
the controller's business; this module only guarantees that it happens on time and that
anyone can tell when it did not.

Two design points worth stating:

**Deadlines are absolute, not relative.** The next deadline is ``t0 + n*dt``, never
``now + dt``. Accumulating from ``now`` lets every late tick push the schedule later, so a
loop that misses occasionally quietly slows down instead of catching up. On a loop feeding
a forecast horizon, drifting rate would corrupt ``tau_c`` and therefore ``h``.

**Overruns are counted, not hidden.** When a tick misses its deadline the loop reports it
and moves to the next one rather than trying to catch up by running back-to-back ticks,
which on a real rig means a burst of stale commands. ``deadline_misses`` in the run
summary is the honest measure of whether the configured rate is achievable, and
:mod:`ct.unknowns` points at it for setting ``procedure.loop_rate_hz``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from ct.hw.interfaces import Clock
from ct.rt.latency import LatencyEstimator


@dataclass
class TickInfo:
    """What the loop knows about one tick, handed to the callback."""

    n: int
    t: float
    """Loop time [s], from the clock. Monotonic; only differences are meaningful."""

    dt: float
    """Time since the previous tick. The nominal period unless the loop overran."""

    frames: dict[str, list[tuple[float, int, bytes]]]
    """This tick's received frames, per bus name."""

    late_by: float = 0.0
    """How far past its deadline this tick started. Zero when on time."""


@dataclass
class LoopStats:
    ticks: int = 0
    deadline_misses: int = 0
    worst_late_s: float = 0.0
    compute: LatencyEstimator = field(default_factory=LatencyEstimator)
    started_at: float = 0.0
    ended_at: float = 0.0

    @property
    def duration(self) -> float:
        return self.ended_at - self.started_at

    @property
    def achieved_rate_hz(self) -> float:
        return self.ticks / self.duration if self.duration > 0 else 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "ticks": self.ticks,
            "duration_s": self.duration,
            "achieved_rate_hz": self.achieved_rate_hz,
            "deadline_misses": self.deadline_misses,
            "worst_late_s": self.worst_late_s,
            "tick_compute_s": self.compute.stats,
        }


class LoopOverrun(RuntimeError):
    """Too many missed deadlines. The loop is not keeping up and said so."""


class ControlLoop:
    """Runs a callback at a fixed rate against a swappable clock.

    Under :class:`~ct.rt.clock.SimClock` this executes as fast as the CPU allows and
    produces identical results every run, which is what turns a twenty-minute insertion
    procedure into a unit test. The callback cannot tell which clock it has.
    """

    def __init__(
        self,
        clock: Clock,
        rate_hz: float,
        buses: dict[str, Any],
        on_tick: Callable[[TickInfo], bool | None],
        *,
        max_deadline_misses: int | None = None,
        compute_window: int = 512,
    ) -> None:
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self.clock = clock
        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz
        self.buses = buses
        self.on_tick = on_tick
        """Returns ``False`` to stop the loop; anything else continues."""

        self.max_deadline_misses = max_deadline_misses
        self.stats = LoopStats(compute=LatencyEstimator(window=compute_window))
        self._stop = False

    def stop(self) -> None:
        """Ask the loop to finish after the current tick."""
        self._stop = True

    def run(self, duration_s: float | None = None, max_ticks: int | None = None) -> LoopStats:
        """Tick until stopped, out of time, or out of ticks."""
        t0 = self.clock.now()
        self.stats.started_at = t0
        deadline = t0
        previous = t0
        n = 0

        while not self._stop:
            if max_ticks is not None and n >= max_ticks:
                break
            now = self.clock.now()
            if duration_s is not None and now - t0 >= duration_s:
                break

            late_by = max(0.0, now - deadline)
            if late_by > self.dt:
                self.stats.deadline_misses += 1
                self.stats.worst_late_s = max(self.stats.worst_late_s, late_by)
                # Drop the slots that were missed and resynchronise to now, rather than
                # firing them back to back to "catch up". Catching up would deliver a
                # burst of commands computed from stale sensor data, which on a rig with a
                # needle in tissue is worse than the overrun that caused it. The rate is
                # still held by absolute deadlines from here on; only the missed slots are
                # forfeited.
                deadline = now
                if (
                    self.max_deadline_misses is not None
                    and self.stats.deadline_misses > self.max_deadline_misses
                ):
                    self.stats.ended_at = now
                    raise LoopOverrun(
                        f"{self.stats.deadline_misses} missed deadlines at {self.rate_hz:g} Hz "
                        f"(worst {self.stats.worst_late_s * 1e3:.1f} ms late). The loop rate is "
                        "higher than this machine can sustain; lower procedure.loop_rate_hz."
                    )

            frames = self._poll(now)
            info = TickInfo(n=n, t=now, dt=now - previous if n else self.dt,
                            frames=frames, late_by=late_by)

            compute_start = self.clock.now()
            result = self.on_tick(info)
            self.stats.compute.record(self.clock.now() - compute_start)

            previous = now
            self.stats.ticks = n = n + 1
            if result is False:
                break

            deadline += self.dt
            self.clock.sleep_until(deadline)

        self.stats.ended_at = self.clock.now()
        return self.stats

    def _poll(self, now: float) -> dict[str, list[tuple[float, int, bytes]]]:
        """Drain every bus. Non-blocking by construction (rule 4).

        ``set_time`` is what lets a loopback bus advance its simulated devices; the real
        bus ignores it. Calling it unconditionally keeps the loop backend-blind, which is
        the property that makes the sim/real swap a config change.
        """
        out: dict[str, list[tuple[float, int, bytes]]] = {}
        for name, bus in self.buses.items():
            setter = getattr(bus, "set_time", None)
            if setter is not None:
                setter(now)
            out[name] = bus.poll()
        return out
