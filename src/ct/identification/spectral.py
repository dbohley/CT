"""Coarse fundamental-frequency estimation.

Two independent estimators — FFT peak and autocorrelation peak — because they
fail differently. The FFT peak is biased by leakage and can lock onto a strong
second harmonic; autocorrelation is robust to that but sensitive to baseline
drift. The identifier runs both and warns when they disagree, which is a cheap
early signal that the recording is not a clean periodic breathing trace.
"""

from __future__ import annotations

import warnings

import numpy as np

# Plausible human respiratory rates. Anything outside this band is not a breath.
DEFAULT_BPM_RANGE = (5.0, 45.0)


def bpm_to_omega(bpm: float) -> float:
    return 2.0 * np.pi * bpm / 60.0


def omega_to_bpm(omega: float) -> float:
    return 60.0 * omega / (2.0 * np.pi)


def fft_peak_omega(
    t: np.ndarray,
    y: np.ndarray,
    bpm_range: tuple[float, float] = DEFAULT_BPM_RANGE,
) -> tuple[float, dict]:
    """Fundamental angular frequency from the periodogram peak.

    The peak bin is refined by fitting a parabola through the log-magnitudes of
    the peak and its two neighbours — the standard sub-bin interpolation, worth
    doing because the raw bin spacing ``2*pi/(N*Ts)`` is also what sets the
    initial ``omega`` variance in ``P0``.
    """
    y = np.asarray(y, dtype=float)
    n = y.size
    fs = 1.0 / float(np.median(np.diff(t)))

    # Remove mean and linear trend: baseline drift otherwise dominates the DC bin
    # and leaks into the lowest few bins where slow breathing lives.
    detrended = y - np.polyval(np.polyfit(t, y, 1), t)
    windowed = detrended * np.hanning(n)

    spec = np.abs(np.fft.rfft(windowed))
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)

    lo, hi = bpm_range[0] / 60.0, bpm_range[1] / 60.0
    band = (freqs >= lo) & (freqs <= hi)
    if not band.any():
        raise ValueError(
            f"no FFT bins inside {bpm_range} bpm — record is too short "
            f"(need >= {1.0 / lo:.0f}s at fs={fs:.3g} Hz for the low end)"
        )

    idx = int(np.flatnonzero(band)[np.argmax(spec[band])])
    delta = _parabolic_offset(spec, idx)
    df = freqs[1] - freqs[0]
    f_hat = freqs[idx] + delta * df

    return 2.0 * np.pi * f_hat, {
        "method": "fft",
        "f_hat": f_hat,
        "bpm_hat": f_hat * 60.0,
        "bin_index": idx,
        "bin_hz": df,
        "resolution_omega": 2.0 * np.pi * df,
        "spectrum_freqs": freqs,
        "spectrum_mag": spec,
    }


def _parabolic_offset(spec: np.ndarray, idx: int) -> float:
    """Sub-bin peak offset in bins, from a parabola through log-magnitudes."""
    if idx <= 0 or idx >= spec.size - 1:
        return 0.0
    a, b, c = (np.log(max(spec[i], 1e-300)) for i in (idx - 1, idx, idx + 1))
    denom = a - 2.0 * b + c
    if abs(denom) < 1e-30:
        return 0.0
    return float(np.clip(0.5 * (a - c) / denom, -0.5, 0.5))


def autocorr_peak_omega(
    t: np.ndarray,
    y: np.ndarray,
    bpm_range: tuple[float, float] = DEFAULT_BPM_RANGE,
) -> tuple[float, dict]:
    """Fundamental angular frequency from the first autocorrelation peak."""
    y = np.asarray(y, dtype=float)
    n = y.size
    fs = 1.0 / float(np.median(np.diff(t)))
    detrended = y - np.polyval(np.polyfit(t, y, 1), t)

    # FFT-based autocorrelation, zero-padded to avoid circular wrap-around.
    nfft = 1 << int(np.ceil(np.log2(2 * n)))
    spec = np.fft.rfft(detrended, nfft)
    ac = np.fft.irfft(spec * np.conj(spec), nfft)[:n]
    if ac[0] <= 0:
        raise ValueError("signal has zero variance; cannot estimate frequency")
    ac = ac / ac[0]

    lag_min = int(np.floor(fs * 60.0 / bpm_range[1]))
    lag_max = int(np.ceil(fs * 60.0 / bpm_range[0]))
    lag_max = min(lag_max, n - 2)
    if lag_max <= lag_min + 1:
        raise ValueError(f"record too short to autocorrelate over {bpm_range} bpm")

    window = ac[lag_min : lag_max + 1]
    peak = lag_min + int(np.argmax(window))
    delta = _parabolic_offset_linear(ac, peak)
    period = (peak + delta) / fs

    return 2.0 * np.pi / period, {
        "method": "autocorr",
        "period_s": period,
        "bpm_hat": 60.0 / period,
        "peak_lag": peak,
        "peak_value": float(ac[peak]),
    }


def _parabolic_offset_linear(ac: np.ndarray, idx: int) -> float:
    if idx <= 0 or idx >= ac.size - 1:
        return 0.0
    a, b, c = ac[idx - 1], ac[idx], ac[idx + 1]
    denom = a - 2.0 * b + c
    if abs(denom) < 1e-30:
        return 0.0
    return float(np.clip(0.5 * (a - c) / denom, -0.5, 0.5))


def coarse_omega(
    t: np.ndarray,
    y: np.ndarray,
    bpm_range: tuple[float, float] = DEFAULT_BPM_RANGE,
    disagreement_tol: float = 0.10,
) -> tuple[float, dict]:
    """Best coarse ``omega_r``, cross-checked between the two estimators.

    The FFT estimate is returned (it is the one whose bin width defines the
    ``omega`` entry of ``P0``); a relative disagreement above ``disagreement_tol``
    raises a warning rather than an error, because on real data a mild mismatch
    is informative but not fatal.
    """
    omega_fft, info_fft = fft_peak_omega(t, y, bpm_range)
    try:
        omega_ac, info_ac = autocorr_peak_omega(t, y, bpm_range)
        rel = abs(omega_fft - omega_ac) / omega_fft
        if rel > disagreement_tol:
            warnings.warn(
                f"FFT and autocorrelation frequency estimates disagree by {rel:.1%} "
                f"({omega_to_bpm(omega_fft):.2f} vs {omega_to_bpm(omega_ac):.2f} bpm). "
                "Check the recording for drift, motion artefact, or non-stationarity.",
                stacklevel=2,
            )
    except ValueError:
        omega_ac, info_ac, rel = float("nan"), {}, float("nan")

    return omega_fft, {
        **info_fft,
        "omega_fft": omega_fft,
        "omega_autocorr": omega_ac,
        "relative_disagreement": rel,
        "autocorr": info_ac,
    }
