"""Stage 1: FFT + least-squares identification of the harmonic model.

Assembles the pieces into the one thing Stage 2 needs — ``(K, s0, P0, Q, R)``:

    1. coarse ``omega_r`` from the periodogram (cross-checked by autocorrelation)
    2. OLS harmonic regression at that fixed frequency, up to ``Kmax``
    3. convert to amplitudes/phases, pick ``K`` by the 95% energy rule
    4. refit at the chosen ``K`` so ``P0`` reflects the model actually tracked
    5. ``R`` from a configured measurement if given, a breath-hold segment if
       available, residual variance otherwise (an upper bound -- see ``_estimate_R``)
    6. ``Q`` from breath-to-breath refits
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ct.identification.harmonic_ls import (
    fit_harmonics,
    rect_to_polar_jacobian,
    select_K,
)
from ct.identification.noise import (
    estimate_Q,
    estimate_R_from_breath_hold,
    estimate_R_from_residuals,
)
from ct.identification.spectral import DEFAULT_BPM_RANGE, coarse_omega, omega_to_bpm
from ct.layout import StateLayout, wrap_angle
from ct.registry import register_identifier
from ct.types import IdentificationResult, SignalBatch


@register_identifier("fft_harmonic")
class FFTHarmonicIdentifier:
    """Batch identifier. Satisfies :class:`ct.interfaces.Identifier`."""

    def __init__(
        self,
        Kmax: int = 8,
        energy_threshold: float = 0.95,
        K_override: int | None = None,
        bpm_range: tuple[float, float] = DEFAULT_BPM_RANGE,
        p0_inflation: float = 2.0,
        p0_floor: float = 1e-9,
        q_scale: float = 1.0,
        q_floor: float = 1e-12,
        r_floor: float = 1e-9,
        breath_hold_window: tuple[float, float] | None = None,
        R_override: float | None = None,
    ) -> None:
        self.Kmax = int(Kmax)
        self.energy_threshold = float(energy_threshold)
        self.K_override = None if K_override is None else int(K_override)
        self.bpm_range = (float(bpm_range[0]), float(bpm_range[1]))
        # P0 is inflated as a model-mismatch margin: the OLS covariance describes
        # only sampling error under a model we know is an approximation.
        self.p0_inflation = float(p0_inflation)
        self.p0_floor = float(p0_floor)
        self.q_scale = float(q_scale)
        self.q_floor = float(q_floor)
        self.r_floor = float(r_floor)
        self.breath_hold_window = breath_hold_window
        # A directly measured R, e.g. from scripts/measure_sensor_noise.py. Beats both
        # other paths because it is the only one that is a measurement of the *sensor*
        # rather than of a fit: breath_hold_window still needs a still segment inside the
        # calibration window to exist, and the residual variance is an upper bound
        # containing every part of the waveform the truncated model failed to represent.
        self.R_override = None if R_override is None else float(R_override)

    def identify(self, batch: SignalBatch) -> IdentificationResult:
        t, y = batch.t, batch.y

        omega_hat, spec_info = coarse_omega(t, y, bpm_range=self.bpm_range)

        Kmax = min(self.Kmax, max(1, (batch.N - 4) // 2))
        wide = fit_harmonics(t, y, omega_hat, Kmax)
        K_energy, k_info = select_K(wide.amplitudes, self.energy_threshold)
        K = self.K_override if self.K_override is not None else K_energy
        K = int(np.clip(K, 1, Kmax))

        fit = wide if K == Kmax else fit_harmonics(t, y, omega_hat, K, t_ref=wide.t_ref)
        layout = StateLayout(K)

        # theta at the END of the calibration window, so tracking resumes exactly
        # where identification stopped.
        t0 = float(t[-1])
        theta0 = wrap_angle(fit.phase_at(t0))
        s0 = layout.pack(
            a0=fit.a0,
            A=fit.amplitudes,
            phi=fit.phases,
            theta=theta0,
            omega_r=omega_hat,
        )

        omega_resolution = float(spec_info["resolution_omega"])
        P0 = self._build_P0(fit, layout, omega_resolution)
        R = self._estimate_R(batch, fit)
        Q, q_info = estimate_Q(
            t,
            y,
            fit,
            K,
            Ts=batch.Ts,
            omega_resolution=omega_resolution,
            q_scale=self.q_scale,
            q_floor=self.q_floor,
            bpm_range=self.bpm_range,
        )

        diagnostics: dict[str, Any] = {
            "omega_hat": omega_hat,
            "bpm_hat": omega_to_bpm(omega_hat),
            "K_energy_rule": K_energy,
            "K_used": K,
            "Kmax": Kmax,
            "energy_threshold": self.energy_threshold,
            "amplitudes_wide": wide.amplitudes,
            "phases_wide": wide.phases,
            "cumulative_energy": k_info["cumulative_energy"],
            "energy_fraction": k_info["energy_fraction"],
            "residual_var": fit.residual_var,
            "residual_var_wide": wide.residual_var,
            "R_source": self._R_source,
            "spectrum_freqs": spec_info["spectrum_freqs"],
            "spectrum_mag": spec_info["spectrum_mag"],
            "omega_resolution": omega_resolution,
            "relative_disagreement": spec_info["relative_disagreement"],
            "state_names": layout.names,
            "q": q_info,
            "calibration_window": (float(t[0]), t0),
            "identifier": "fft_harmonic",
        }

        return IdentificationResult(
            K=K, s0=s0, P0=P0, Q=Q, R=R, t0=t0, Ts=batch.Ts, diagnostics=diagnostics
        )

    # -- pieces ----------------------------------------------------------------

    def _build_P0(self, fit, layout: StateLayout, omega_resolution: float) -> np.ndarray:
        """Push the OLS coefficient covariance into amplitude/phase coordinates.

        ``G`` is the delta-method Jacobian of ``[a0, A_1, phi_1, ...]`` w.r.t. the
        regression coefficients ``[a0, alpha_1, beta_1, ...]``; the cross terms
        are kept rather than zeroed, since a strong ``alpha``/``beta``
        correlation is exactly what makes an amplitude and its phase correlated.
        """
        K = layout.K
        m = 1 + 2 * K
        G = np.zeros((layout.n, m))
        G[layout.a0, 0] = 1.0
        for k in range(1, K + 1):
            J = rect_to_polar_jacobian(fit.coeffs[2 * k - 1], fit.coeffs[2 * k])
            G[layout.A(k), 2 * k - 1 : 2 * k + 1] = J[0]
            G[layout.phi(k), 2 * k - 1 : 2 * k + 1] = J[1]

        P0 = G @ fit.cov @ G.T

        # theta: no OLS counterpart (it is a state, not a coefficient). Give it
        # the fundamental phase's scale.
        P0[layout.theta, layout.theta] = max(P0[layout.phi(1), layout.phi(1)], self.p0_floor)
        # omega: the frequency-search resolution, 2*pi/(N*Ts).
        P0[layout.omega, layout.omega] = omega_resolution**2

        P0 = self.p0_inflation * P0
        P0 = 0.5 * (P0 + P0.T)
        idx = np.diag_indices_from(P0)
        P0[idx] = np.maximum(P0[idx], self.p0_floor)
        return P0

    def _estimate_R(self, batch: SignalBatch, fit) -> float:
        if self.R_override is not None:
            self._R_source = "configured (R_override)"
            return max(self.R_override, self.r_floor)
        if self.breath_hold_window is not None:
            lo, hi = self.breath_hold_window
            seg = batch.y[(batch.t >= lo) & (batch.t < hi)]
            if seg.size >= 2:
                self._R_source = "breath_hold"
                return max(estimate_R_from_breath_hold(seg), self.r_floor)
        self._R_source = "stage1_residual_variance (upper bound)"
        return max(estimate_R_from_residuals(fit), self.r_floor)

    _R_source = "unset"

    @property
    def config(self) -> dict[str, Any]:
        return {
            "name": "fft_harmonic",
            "Kmax": self.Kmax,
            "energy_threshold": self.energy_threshold,
            "K_override": self.K_override,
            "R_override": self.R_override,
            "p0_inflation": self.p0_inflation,
            "q_scale": self.q_scale,
        }
