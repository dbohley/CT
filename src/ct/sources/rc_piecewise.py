"""RC-circuit piecewise chest-wall model.

Singh, Rehman, Yongchareon, Chong, "Modelling of Chest Wall Motion for
Cardiorespiratory Activity for Radar-Based NCVS Systems," Sensors 2020, 20(18),
5094.

A first-order respiratory-mechanics ODE

    tau_rs * V'(t) + V(t) = P(t) * tau_rs / R_rs        (i.e. V' + V/tau_rs = P/R_rs)

driven by a quadratic isometric pressure pulse during inhale and an exponentially
decaying pressure during exhale, solved separately per phase. This is a genuine
two-mode hybrid system, which is exactly why it is used only as a *data
generator* here and is not baked into the online EKF — tracking it directly would
need a switched/IMM filter, a scope increase rather than a state addition.

Two corrections to the coefficients as transcribed in the project reference doc,
both re-derived here (see :func:`_inhale_coeffs` and :meth:`_exhale`):

  * ``A2 = a1 - 2*a2*tau_rs``   (the doc has ``a2 - 2*a2*tau_rs``; the linear
    pressure coefficient ``a1`` must appear, and ``A3`` in the same doc already
    uses ``a1`` consistently with this form)
  * the exhale term is ``exp(-(t-t1)/tau)``, not ``exp(+(t-t1)/tau)`` — a growing
    exponential would make the exhale diverge rather than decay.

Both follow from solving ``V' + V/tau_rs = P/R_rs`` by undetermined coefficients.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ct.registry import register_source
from ct.sources.base import SyntheticSource


def _inhale_coeffs(a0: float, a1: float, a2: float, tau_rs: float) -> tuple[float, float, float]:
    """Particular-solution coefficients for a quadratic drive ``P = a0+a1 t+a2 t^2``.

    Substituting ``V_p = (tau/R)(A1 t^2 + A2 t + A3)`` into ``V' + V/tau = P/R``
    and matching powers of ``t`` gives the three coefficients below.
    """
    A1 = a2
    A2 = a1 - 2.0 * a2 * tau_rs
    A3 = a0 - a1 * tau_rs + 2.0 * a2 * tau_rs**2
    return A1, A2, A3


@register_source("rc_piecewise")
class RCPiecewiseSource(SyntheticSource):
    """Two-phase (inhale/exhale) chest-wall displacement generator.

    The initial volume ``V0`` is not a free parameter: it is solved for so the
    cycle is exactly periodic. The cycle map ``V0 -> V(T)`` is affine, so its
    fixed point is found in closed form from two evaluations rather than by
    iterating transients away. Periodicity matters — the harmonic model the
    estimator fits assumes it.
    """

    def __init__(
        self,
        fs: float = 50.0,
        noise_std: float = 0.0,
        seed: int = 0,
        t1: float = 1.6,
        t2: float = 2.4,
        R_rs: float = 2.0,
        C_rs: float = 0.5,
        tau: float = 0.8,
        a0: float = 1.0,
        a1: float = 0.6,
        a2: float = -0.35,
        amplitude: float = 10.0,
        offset: float = 0.0,
        cardiac: bool = False,
        cardiac_amplitude: float = 0.3,
        cardiac_bpm: float = 70.0,
        cardiac_mu: float = 0.4,
    ) -> None:
        super().__init__(fs=fs, noise_std=noise_std, seed=seed)
        if t1 <= 0 or t2 <= 0:
            raise ValueError("t1 and t2 must be positive")
        self.t1, self.t2 = float(t1), float(t2)
        self.T = self.t1 + self.t2
        self.R_rs, self.C_rs = float(R_rs), float(C_rs)
        self.tau_rs = self.R_rs * self.C_rs
        self.tau = float(tau)
        if abs(1.0 / self.tau_rs - 1.0 / self.tau) < 1e-12:
            raise ValueError("tau must differ from tau_rs = R_rs*C_rs (degenerate solution)")
        self.a0, self.a1, self.a2 = float(a0), float(a1), float(a2)
        self.amplitude = float(amplitude)
        self.offset = float(offset)
        self.cardiac = bool(cardiac)
        self.cardiac_amplitude = float(cardiac_amplitude)
        self.cardiac_bpm = float(cardiac_bpm)
        self.cardiac_mu = float(cardiac_mu)

        self._V0 = self._periodic_V0()
        self._scale, self._shift = self._normalisation()

    @property
    def breaths_per_min(self) -> float:
        return 60.0 / self.T

    @property
    def omega_r(self) -> float:
        return 2.0 * np.pi / self.T

    # -- the two phase solutions ----------------------------------------------

    def _pressure(self, s: np.ndarray | float) -> np.ndarray | float:
        return self.a0 + self.a1 * s + self.a2 * np.asarray(s, dtype=float) ** 2

    def _inhale(self, s: np.ndarray, V0: float) -> np.ndarray:
        """``0 <= s <= t1``, starting from volume ``V0``."""
        A1, A2, A3 = _inhale_coeffs(self.a0, self.a1, self.a2, self.tau_rs)
        decay = np.exp(-s / self.tau_rs)
        return (self.tau_rs / self.R_rs) * (
            A1 * s**2 + A2 * s + A3 * (1.0 - decay)
        ) + V0 * decay

    def _exhale(self, s: np.ndarray, V_t1: float) -> np.ndarray:
        """``0 <= s <= t2``, starting from the end-inhale volume ``V_t1``.

        Driving pressure relaxes as ``P(t1) exp(-s/tau)``; the particular
        solution's amplitude is ``P(t1) / (R_rs (1/tau_rs - 1/tau))``.
        """
        B = self._pressure(self.t1) / (self.R_rs * (1.0 / self.tau_rs - 1.0 / self.tau))
        return B * (np.exp(-s / self.tau) - np.exp(-s / self.tau_rs)) + V_t1 * np.exp(
            -s / self.tau_rs
        )

    def _cycle_map(self, V0: float) -> float:
        V_t1 = float(self._inhale(np.array([self.t1]), V0)[0])
        return float(self._exhale(np.array([self.t2]), V_t1)[0])

    def _periodic_V0(self) -> float:
        """Fixed point of the affine cycle map ``V(T) = alpha*V0 + beta``."""
        beta = self._cycle_map(0.0)
        alpha = self._cycle_map(1.0) - beta
        if abs(1.0 - alpha) < 1e-12:
            raise ValueError("cycle map has no unique periodic solution for these parameters")
        return beta / (1.0 - alpha)

    def _volume(self, t: np.ndarray) -> np.ndarray:
        s = np.mod(np.asarray(t, dtype=float), self.T)
        V_t1 = float(self._inhale(np.array([self.t1]), self._V0)[0])
        inhaling = s <= self.t1
        out = np.empty_like(s)
        out[inhaling] = self._inhale(s[inhaling], self._V0)
        out[~inhaling] = self._exhale(s[~inhaling] - self.t1, V_t1)
        return out

    def _normalisation(self) -> tuple[float, float]:
        """Rescale volume to a peak-to-peak displacement of ``amplitude``, zero-mean."""
        probe = self._volume(np.linspace(0.0, self.T, 4001))
        span = float(probe.max() - probe.min())
        scale = self.amplitude / span if span > 0 else 1.0
        return scale, -scale * float(probe.mean())

    # -- cardiac component -----------------------------------------------------

    def _cardiac(self, t: np.ndarray) -> np.ndarray:
        """Small Van der Pol oscillation at the heart rate.

        Amplitude is ~0.2-0.5 mm against 3-12 mm of respiration, so it usually
        sits near the noise floor. It is off by default; enable it to check that
        it does not leak spurious high-frequency content into the harmonic fit.
        """
        from scipy.integrate import solve_ivp

        w = 2.0 * np.pi * self.cardiac_bpm / 60.0
        mu = self.cardiac_mu

        def vdp(_t: float, z: np.ndarray) -> list[float]:
            x, v = z
            return [v, mu * w * (1.0 - x**2) * v - w**2 * x]

        t = np.asarray(t, dtype=float)
        t_span = (float(t.min()), float(t.max()) + 1e-9)
        sol = solve_ivp(vdp, t_span, [2.0, 0.0], t_eval=t, rtol=1e-8, atol=1e-10, max_step=0.05)
        x = sol.y[0]
        span = float(x.max() - x.min())
        return self.cardiac_amplitude * (x - x.mean()) / (span if span > 0 else 1.0)

    # -- source interface ------------------------------------------------------

    def clean(self, t: np.ndarray) -> np.ndarray:
        y = self._scale * self._volume(t) + self._shift + self.offset
        if self.cardiac:
            y = y + self._cardiac(t)
        return y

    def truth_dict(self) -> dict[str, Any]:
        return {
            "model": "rc_piecewise",
            "t1": self.t1,
            "t2": self.t2,
            "T": self.T,
            "omega_r": self.omega_r,
            "breaths_per_min": self.breaths_per_min,
            "R_rs": self.R_rs,
            "C_rs": self.C_rs,
            "tau_rs": self.tau_rs,
            "tau": self.tau,
            "pressure_coeffs": [self.a0, self.a1, self.a2],
            "amplitude": self.amplitude,
            "V0_periodic": self._V0,
            "cardiac": self.cardiac,
        }
