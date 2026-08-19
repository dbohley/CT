"""Assembling the forecast horizon ``h`` from the four things that delay a needle.

``h = tau_s + tau_c + tau_cl(omega_r) + T_ins``

The estimator never decomposes ``h`` — :mod:`ct.forecast` takes whatever number it is
handed and evaluates the model there. This module is the other side of that seam: the
place where the four terms are actually sourced, each from somewhere different.

    tau_s            configured, and measurable with ``ct-compare``
    tau_c            measured by the loop, from its own tick times
    tau_cl(omega_r)  computed from the servo design; varies with breathing rate
    T_ins            configured, from timed test insertions

Only ``tau_cl`` is a function. That is the point of the settled decision recorded in
``CLAUDE.md``: the raw actuator delay ``tau_a`` was replaced by the *residual closed-loop
tracking lag*, which is a servo-design output that changes with how fast the patient is
breathing. Treating it as a constant is the error that decision exists to prevent, so
:meth:`LatencyBudget.horizon` takes ``omega_r`` and there is no way to call it without one.
"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from ct.forecast import horizon_from_components
from ct.hw.config import LatencyConfig


class LatencyEstimator:
    """Running quantiles of how long a tick takes.

    A p95 rather than a mean: the horizon has to cover the ticks that were slow, not the
    typical one. Backed by a ring buffer so it costs nothing per tick and cannot grow
    without bound over a long run.
    """

    def __init__(self, window: int = 512, fallback: float = 0.005) -> None:
        if window <= 0:
            raise ValueError("window must be positive")
        self._buf = np.zeros(window, dtype=float)
        self._n = 0
        self._i = 0
        self.fallback = float(fallback)

    def record(self, seconds: float) -> None:
        self._buf[self._i] = seconds
        self._i = (self._i + 1) % self._buf.size
        self._n = min(self._n + 1, self._buf.size)

    def quantile(self, q: float) -> float:
        """Quantile of the recorded tick times, or the fallback before any are recorded."""
        if self._n == 0:
            return self.fallback
        return float(np.quantile(self._buf[: self._n], q))

    @property
    def p50(self) -> float:
        return self.quantile(0.50)

    @property
    def p95(self) -> float:
        return self.quantile(0.95)

    @property
    def worst(self) -> float:
        return self.fallback if self._n == 0 else float(self._buf[: self._n].max())

    @property
    def n(self) -> int:
        return self._n

    @property
    def stats(self) -> dict[str, Any]:
        return {"n": self._n, "p50": self.p50, "p95": self.p95, "max": self.worst}


class LatencyBudget:
    """The four horizon terms, and the one operation that consumes them."""

    def __init__(
        self,
        config: LatencyConfig,
        tau_cl: Callable[[float], float] | None = None,
    ) -> None:
        self.config = config
        self.tau_cl = tau_cl
        """``omega_r -> residual closed-loop lag``. Supplied by :mod:`ct.control.servo`.

        ``None`` falls back to the configured constant, which is a stopgap for running
        before the servo has been identified — and is flagged as such by
        :attr:`uses_fallback_tau_cl` so a run cannot quietly rely on it.
        """

        self.compute = LatencyEstimator(fallback=config.tau_c)

    @property
    def uses_fallback_tau_cl(self) -> bool:
        return self.tau_cl is None

    def tau_compute(self) -> float:
        """Compute latency: measured if the loop has data, configured otherwise."""
        if not self.config.measure_tau_c:
            return self.config.tau_c
        return self.compute.p95

    def tau_closed_loop(self, omega_r: float) -> float:
        if self.tau_cl is None:
            return self.config.tau_cl_fallback
        return float(self.tau_cl(omega_r))

    def horizon(self, omega_r: float) -> float:
        """The full ``h`` at this breathing rate.

        Goes through :func:`ct.forecast.horizon_from_components` rather than adding the
        four numbers here, so there is exactly one definition of what ``h`` is and the
        non-negativity checks apply to hardware-sourced values too.
        """
        return horizon_from_components(
            tau_sensor=self.config.tau_s,
            tau_compute=self.tau_compute(),
            tau_closed_loop=self.tau_closed_loop(omega_r),
            T_insertion=self.config.T_ins,
        )

    def breakdown(self, omega_r: float) -> dict[str, float]:
        """The terms individually — for telemetry and the run summary, never for the model.

        Worth logging: when a forecast is systematically early or late, which term is
        wrong is the first question, and it cannot be recovered from the sum.
        """
        tau_s = self.config.tau_s
        tau_c = self.tau_compute()
        tau_cl = self.tau_closed_loop(omega_r)
        T_ins = self.config.T_ins
        return {
            "tau_s": tau_s,
            "tau_c": tau_c,
            "tau_cl": tau_cl,
            "T_ins": T_ins,
            "h": tau_s + tau_c + tau_cl + T_ins,
            "omega_r": omega_r,
        }

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "tau_s": self.config.tau_s,
            "T_ins": self.config.T_ins,
            "compute": self.compute.stats,
            "tau_cl_fallback_in_use": self.uses_fallback_tau_cl,
        }
