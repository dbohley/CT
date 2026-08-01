"""Stage 2: extended Kalman filter matched to an adaptive multi-harmonic model.

An EKF over the state

    s = [a0, A_1, phi_1, ..., A_K, phi_K, theta, omega_r]

with a linear process model and a nonlinear scalar measurement. The nearest
biomedical prior art is the Weighted-Frequency Fourier Linear Combiner (Riviere
et al., IEEE EMBS 2001), which adapts the same model by LMS. Using an EKF instead
is a deliberate choice: it produces an explicit covariance ``P`` at every step,
which the downstream gate can consume directly. LMS gives no such signal.

Numerically the important detail is that the measurement is **scalar**. ``S`` and
``R`` are plain floats, so the gain step is a division rather than a matrix
inverse — no ``solve``, no conditioning worry from inverting an innovation
covariance.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ct.layout import StateLayout, wrap_angle
from ct.registry import register_tracker
from ct.tracking.measurement import (
    advance_phase,
    measurement,
    measurement_jacobian,
    transition,
    transition_matrix,
)
from ct.types import IdentificationResult, TrackerStep


@register_tracker("harmonic_ekf")
class HarmonicEKF:
    """Recursive tracker for the multi-harmonic respiratory model."""

    def __init__(
        self,
        joseph: bool = True,
        wrap_phases: bool = True,
        clamp_amplitudes: bool = False,
        omega_bounds: tuple[float, float] | None = None,
        scale_q_with_dt: bool = True,
    ) -> None:
        # Joseph form: P = (I-KH) P (I-KH)^T + K R K^T. Algebraically identical to
        # the short form but stays symmetric positive-definite over long runs.
        self.joseph = bool(joseph)
        self.wrap_phases = bool(wrap_phases)
        # Sign/range constraints are off by default so the raw filter behaviour is
        # observable first; a filter that needs clamping to stay sane is telling
        # you something about Q, and that should not be hidden.
        self.clamp_amplitudes = bool(clamp_amplitudes)
        self.omega_bounds = omega_bounds
        self.scale_q_with_dt = bool(scale_q_with_dt)

        self.layout: StateLayout | None = None
        self._s: np.ndarray | None = None
        self._P: np.ndarray | None = None
        self._t: float = 0.0
        self._Q: np.ndarray | None = None
        self._Q_rate: np.ndarray | None = None
        self._R: float = 0.0
        self.n_steps = 0

    # -- lifecycle -------------------------------------------------------------

    def init(self, result: IdentificationResult, t0: float | None = None) -> None:
        self.layout = StateLayout(result.K)
        self._s = np.array(result.s0, dtype=float, copy=True)
        self._P = np.array(result.P0, dtype=float, copy=True)
        self._Q = np.array(result.Q, dtype=float, copy=True)
        self._R = float(result.R)
        self._t = float(result.t0 if t0 is None else t0)
        self.n_steps = 0

        # Q was measured per-sample at the calibration rate. Holding a per-second
        # rate lets an irregular dt be handled correctly instead of silently
        # under- or over-inflating the covariance.
        Ts = float(result.Ts)
        self._Q_rate = self._Q / Ts if (self.scale_q_with_dt and Ts > 0) else None
        self._nominal_Ts = Ts

    def _require_init(self) -> tuple[StateLayout, np.ndarray, np.ndarray]:
        if self.layout is None or self._s is None or self._P is None:
            raise RuntimeError("tracker used before init(); call init(result) first")
        return self.layout, self._s, self._P

    # -- recursion -------------------------------------------------------------

    def predict(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Time update. Returns ``(s_pred, P_pred)`` without committing them."""
        layout, s, P = self._require_init()
        F = transition_matrix(layout, dt)
        s_pred = transition(s, layout, dt)
        Q = self._Q_rate * dt if self._Q_rate is not None else self._Q
        P_pred = F @ P @ F.T + Q
        return s_pred, P_pred

    def step(self, t: float, y: float) -> TrackerStep:
        """One predict + update cycle against measurement ``y`` at time ``t``."""
        layout, _, _ = self._require_init()
        dt = float(t) - self._t
        if dt < 0:
            raise ValueError(f"measurement at t={t} precedes filter time {self._t}")

        s_pred, P_pred = self.predict(dt)

        H = measurement_jacobian(s_pred, layout)
        y_pred = measurement(s_pred, layout)
        innovation = float(y) - y_pred

        # Scalar measurement: S is a float, so the gain is a division.
        PHt = P_pred @ H
        S = float(H @ PHt) + self._R
        if S <= 0:
            raise FloatingPointError(f"innovation covariance is non-positive (S={S})")
        gain = PHt / S

        s_new = s_pred + gain * innovation
        if self.joseph:
            IKH = np.eye(layout.n) - np.outer(gain, H)
            P_new = IKH @ P_pred @ IKH.T + self._R * np.outer(gain, gain)
        else:
            P_new = P_pred - np.outer(gain, PHt)
        P_new = 0.5 * (P_new + P_new.T)

        self._s = self._constrain(s_new, layout)
        self._P = P_new
        self._t = float(t)
        self.n_steps += 1

        return TrackerStep(
            t=float(t),
            s=self._s.copy(),
            P=P_new.copy(),
            y_pred=y_pred,
            innovation=innovation,
            S=S,
            nis=innovation**2 / S,
        )

    def _constrain(self, s: np.ndarray, layout: StateLayout) -> np.ndarray:
        if self.wrap_phases:
            s[layout.phase_idx] = wrap_angle(s[layout.phase_idx])
            s[layout.theta] = wrap_angle(s[layout.theta])
        if self.clamp_amplitudes:
            # A_k < 0 is not an error -- it is the same waveform with phi_k + pi --
            # so fold it back rather than truncating at zero, which would bias A.
            neg = s[layout.amplitude_idx] < 0
            if neg.any():
                idx = layout.amplitude_idx[neg]
                pidx = layout.phase_idx[neg]
                s[idx] = -s[idx]
                s[pidx] = wrap_angle(s[pidx] + np.pi)
        if self.omega_bounds is not None:
            s[layout.omega] = float(np.clip(s[layout.omega], *self.omega_bounds))
        return s

    # -- forecasting -----------------------------------------------------------

    def forecast(self, h: float) -> float:
        """Predicted signal value at ``t_now + h``.

        ``h`` is the full horizon ``tau_s + tau_c + tau_cl(omega_r) + T_ins``,
        supplied by the caller. This is a time-advance of the whole model, so
        harmonic ``k`` rotates by ``k*omega_r*h``.
        """
        layout, s, _ = self._require_init()
        return measurement(advance_phase(s, layout, h), layout)

    def forecast_variance(self, h: float) -> float:
        """First-order variance of the forecast, from the same Jacobians.

        Not used by the estimator itself, but this is the quantity a gate would
        threshold on: it grows with ``h`` and with the uncertainty in ``omega_r``.
        """
        layout, s, P = self._require_init()
        F = transition_matrix(layout, h)
        Q = self._Q_rate * h if self._Q_rate is not None else self._Q
        P_h = F @ P @ F.T + Q
        s_h = advance_phase(s, layout, h)
        H = measurement_jacobian(s_h, layout)
        return float(H @ P_h @ H)

    def forecast_trajectory(self, horizons: np.ndarray) -> np.ndarray:
        """Vectorised :meth:`forecast` over several horizons."""
        return np.array([self.forecast(float(h)) for h in np.atleast_1d(horizons)])

    # -- accessors -------------------------------------------------------------

    @property
    def state(self) -> tuple[np.ndarray, np.ndarray]:
        _, s, P = self._require_init()
        return s.copy(), P.copy()

    @property
    def t(self) -> float:
        return self._t

    def std(self) -> np.ndarray:
        """Per-state standard deviations — the +-sigma ribbons in the plots."""
        _, _, P = self._require_init()
        return np.sqrt(np.maximum(np.diag(P), 0.0))

    @property
    def config(self) -> dict[str, Any]:
        return {
            "name": "harmonic_ekf",
            "joseph": self.joseph,
            "wrap_phases": self.wrap_phases,
            "clamp_amplitudes": self.clamp_amplitudes,
            "omega_bounds": self.omega_bounds,
            "scale_q_with_dt": self.scale_q_with_dt,
        }

    def __repr__(self) -> str:
        K = self.layout.K if self.layout else "?"
        return f"HarmonicEKF(K={K}, steps={self.n_steps}, t={self._t:.3f})"
