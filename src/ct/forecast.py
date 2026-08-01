"""Forecasting the tracked model forward by a horizon ``h``.

The horizon is ``h = tau_s + tau_c + tau_cl(omega_r) + T_ins``:

    tau_s          sensor latency
    tau_c          computation latency
    tau_cl(omega_r) residual closed-loop tracking lag of the needle servo
    T_ins          insertion duration

All four are consumed by a *single* forecasting operation; none is "solved"
individually, and none of them is this package's to determine. In particular
``tau_cl`` is an output of the servo design (a separate, feedback loop) — it is
not a fixed actuator constant, and it varies with breathing rate. This module
only evaluates the model at whatever ``h`` the rest of the system supplies.
"""

from __future__ import annotations

import numpy as np

from ct.layout import StateLayout
from ct.tracking.measurement import advance_phase, measurement


def forecast_value(s: np.ndarray, layout: StateLayout, h: float) -> float:
    """Model value at ``t_now + h`` for state ``s``."""
    return measurement(advance_phase(s, layout, h), layout)


def forecast_series(s: np.ndarray, layout: StateLayout, horizons: np.ndarray) -> np.ndarray:
    """Model values over several horizons from one state."""
    return np.array([forecast_value(s, layout, float(h)) for h in np.atleast_1d(horizons)])


def naive_common_phase_forecast(s: np.ndarray, layout: StateLayout, h: float) -> float:
    """The WRONG way to forecast — kept only so tests can prove it is wrong.

    Rotating every harmonic's phase by the same delay-derived angle
    ``omega_r*h`` advances only the fundamental correctly. Harmonic ``k`` needs
    ``k*omega_r*h``, so this under-rotates every higher harmonic and distorts the
    waveform's shape instead of shifting it in time. Never call this outside the
    regression test in ``tests/test_forecast.py``.
    """
    s = np.array(s, dtype=float, copy=True)
    delta = s[layout.omega] * h
    s[layout.phase_idx] = s[layout.phase_idx] + delta
    return measurement(s, layout)


def horizon_from_components(
    tau_sensor: float = 0.0,
    tau_compute: float = 0.0,
    tau_closed_loop: float = 0.0,
    T_insertion: float = 0.0,
) -> float:
    """Assemble ``h`` from its four components.

    A convenience for callers, not a model: the estimator never decomposes ``h``
    and behaves identically however it was arrived at.
    """
    for name, value in (
        ("tau_sensor", tau_sensor),
        ("tau_compute", tau_compute),
        ("tau_closed_loop", tau_closed_loop),
        ("T_insertion", T_insertion),
    ):
        if value < 0:
            raise ValueError(f"{name} must be non-negative, got {value}")
    return tau_sensor + tau_compute + tau_closed_loop + T_insertion
