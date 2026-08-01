"""Every source must present the same face to the estimator."""

from __future__ import annotations

import numpy as np
import pytest

from ct.registry import SOURCES, build_source
from ct.sources.csv_source import CSVSource, write_csv
from ct.sources.lujan import LujanSource
from ct.sources.rc_piecewise import RCPiecewiseSource, _inhale_coeffs

SYNTHETIC = ["sinusoid", "lujan", "rc_piecewise"]


@pytest.mark.parametrize("name", SYNTHETIC)
def test_batch_contract(name):
    src = build_source(name, {"fs": 50.0, "noise_std": 0.1, "seed": 1})
    b = src.batch(20.0)
    assert b.N == 1000
    assert b.t.shape == b.y.shape == (1000,)
    assert b.fs == 50.0
    assert b.Ts == pytest.approx(0.02)
    assert b.y_clean is not None and b.truth is not None
    assert np.isfinite(b.y).all()
    np.testing.assert_allclose(np.diff(b.t), 0.02, atol=1e-12)


@pytest.mark.parametrize("name", SYNTHETIC)
def test_seeded_batches_are_deterministic(name):
    a = build_source(name, {"fs": 50.0, "noise_std": 0.2, "seed": 7}).batch(10.0)
    b = build_source(name, {"fs": 50.0, "noise_std": 0.2, "seed": 7}).batch(10.0)
    np.testing.assert_array_equal(a.y, b.y)


@pytest.mark.parametrize("name", SYNTHETIC)
def test_stream_matches_batch(name):
    """stream() is the online path; it must deliver exactly what batch() would."""
    src = build_source(name, {"fs": 50.0, "noise_std": 0.1, "seed": 2})
    b = src.batch(5.0)
    streamed = np.array(list(src.stream(5.0)))
    np.testing.assert_allclose(streamed[:, 0], b.t)
    np.testing.assert_allclose(streamed[:, 1], b.y)


@pytest.mark.parametrize("name", SYNTHETIC)
def test_noise_is_added_at_the_requested_scale(name):
    src = build_source(name, {"fs": 50.0, "noise_std": 0.3, "seed": 5})
    b = src.batch(200.0)
    assert np.std(b.y - b.y_clean) == pytest.approx(0.3, rel=0.05)


@pytest.mark.parametrize("name", SYNTHETIC)
def test_signals_are_periodic(name):
    """Every generator must repeat at its stated period — the harmonic model
    the estimator fits assumes it."""
    src = build_source(name, {"fs": 100.0, "noise_std": 0.0})
    T = 2 * np.pi / src.truth_dict()["omega_r"]
    t = np.linspace(20.0, 20.0 + T, 500)
    np.testing.assert_allclose(src.clean(t), src.clean(t + T), atol=1e-8)


def test_sources_are_all_registered():
    assert {"sinusoid", "lujan", "rc_piecewise", "csv"} <= set(SOURCES)


# -- Lujan ---------------------------------------------------------------------


def test_lujan_n1_is_a_pure_sinusoid():
    """cos^2(u) = (1 + cos 2u)/2, so n=1 must be exactly one harmonic."""
    src = LujanSource(fs=100.0, n=1, a=10.0, b=0.0, phi=0.0, breaths_per_min=15.0)
    t = np.linspace(0, 20, 2000)
    expected = -5.0 - 5.0 * np.cos(2 * np.pi * t / src.T)
    np.testing.assert_allclose(src.clean(t), expected, atol=1e-10)


@pytest.mark.parametrize("n", [1, 2, 3, 4])
def test_lujan_exact_fourier_coefficients(n):
    """The closed-form binomial coefficients must match a numerical DFT."""
    src = LujanSource(fs=200.0, n=n, a=10.0, breaths_per_min=15.0)
    t = np.arange(0, 10 * src.T, 1 / 200.0)
    y = src.clean(t) - src.clean(t).mean()
    spec = np.fft.rfft(y) / (y.size / 2)
    freqs = np.fft.rfftfreq(y.size, 1 / 200.0)
    measured = [np.abs(spec[np.argmin(np.abs(freqs - k / src.T))]) for k in range(1, n + 1)]
    np.testing.assert_allclose(measured, src.fourier_coefficients(), rtol=1e-3, atol=1e-6)


@pytest.mark.parametrize("n", [1, 2, 3])
def test_lujan_has_exactly_n_harmonics(n):
    """Harmonic n+1 must be numerically absent, not merely small."""
    src = LujanSource(fs=200.0, n=n, a=10.0, breaths_per_min=15.0)
    t = np.arange(0, 20 * src.T, 1 / 200.0)
    y = src.clean(t) - src.clean(t).mean()
    spec = np.abs(np.fft.rfft(y))
    freqs = np.fft.rfftfreq(y.size, 1 / 200.0)
    at = lambda k: spec[np.argmin(np.abs(freqs - k / src.T))]  # noqa: E731
    assert at(n + 1) < 1e-6 * at(1)


# -- RC piecewise --------------------------------------------------------------


def test_rc_inhale_coefficients_solve_the_ode():
    """Check A1/A2/A3 against the ODE they claim to solve.

    The project reference doc transcribes A2 as ``a2 - 2*a2*tau``; substituting
    the particular solution into ``V' + V/tau = P/R`` gives ``a1 - 2*a2*tau``.
    This test is what pins that correction down.
    """
    a0, a1, a2, tau, R = 1.0, 0.6, -0.35, 1.0, 2.0
    A1, A2, A3 = _inhale_coeffs(a0, a1, a2, tau)

    s = np.linspace(0.1, 2.0, 50)
    Vp = (tau / R) * (A1 * s**2 + A2 * s + A3)
    dVp = (tau / R) * (2 * A1 * s + A2)
    P = a0 + a1 * s + a2 * s**2
    np.testing.assert_allclose(dVp + Vp / tau, P / R, atol=1e-12)


def test_rc_is_continuous_at_the_phase_transition():
    """Volume must not jump at t1; only its derivative may kink."""
    src = RCPiecewiseSource(fs=1000.0, noise_std=0.0)
    eps = 1e-6
    before = src.clean(np.array([src.t1 - eps]))[0]
    after = src.clean(np.array([src.t1 + eps]))[0]
    assert before == pytest.approx(after, abs=1e-4)


def test_rc_cycle_is_exactly_periodic():
    """The solved-for V0 must be a genuine fixed point of the cycle map."""
    src = RCPiecewiseSource(fs=100.0, noise_std=0.0)
    assert src._cycle_map(src._V0) == pytest.approx(src._V0, abs=1e-12)


def test_rc_normalises_to_the_requested_amplitude():
    src = RCPiecewiseSource(fs=200.0, noise_std=0.0, amplitude=12.0)
    y = src.clean(np.arange(0, 4 * src.T, 0.005))
    assert float(y.max() - y.min()) == pytest.approx(12.0, rel=1e-3)


def test_rc_rejects_degenerate_time_constants():
    with pytest.raises(ValueError, match="must differ"):
        RCPiecewiseSource(R_rs=2.0, C_rs=0.5, tau=1.0)


def test_rc_cardiac_component_is_small_and_optional():
    base = RCPiecewiseSource(fs=100.0, noise_std=0.0, cardiac=False)
    with_heart = RCPiecewiseSource(fs=100.0, noise_std=0.0, cardiac=True, cardiac_amplitude=0.3)
    t = np.arange(0, 20.0, 0.01)
    delta = with_heart.clean(t) - base.clean(t)
    assert np.ptp(delta) == pytest.approx(0.3, rel=0.2)
    assert np.ptp(delta) < 0.1 * np.ptp(base.clean(t))


# -- CSV -----------------------------------------------------------------------


def test_csv_roundtrip(tmp_path):
    """The seam for real data: what goes out must come back identical."""
    src = build_source("lujan", {"fs": 50.0, "noise_std": 0.1, "seed": 4})
    original = src.batch(30.0)
    path = write_csv(original, tmp_path / "trace.csv")

    loaded = CSVSource(path).batch()
    np.testing.assert_allclose(loaded.t, original.t)
    np.testing.assert_allclose(loaded.y, original.y)
    np.testing.assert_allclose(loaded.y_clean, original.y_clean)
    assert loaded.fs == pytest.approx(original.fs)


def test_csv_has_no_truth():
    """Nothing in the estimator may depend on y_clean/truth, because real data
    has neither."""
    src = build_source("sinusoid", {"fs": 50.0})
    import tempfile

    with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as fh:
        import pandas as pd

        b = src.batch(10.0)
        pd.DataFrame({"t": b.t, "y": b.y}).to_csv(fh.name, index=False)
        loaded = CSVSource(fh.name).batch()
    assert loaded.truth is None
    assert loaded.y_clean is None


def test_csv_rejects_nonuniform_timestamps(tmp_path):
    import pandas as pd

    t = np.concatenate([np.arange(0, 1, 0.02), np.arange(1.5, 2.5, 0.02)])
    path = tmp_path / "gappy.csv"
    pd.DataFrame({"t": t, "y": np.sin(t)}).to_csv(path, index=False)
    with pytest.raises(ValueError, match="non-uniform"):
        CSVSource(path)


def test_csv_missing_column_names_what_it_found(tmp_path):
    import pandas as pd

    path = tmp_path / "wrong.csv"
    pd.DataFrame({"time": [0, 1], "value": [0, 1]}).to_csv(path, index=False)
    with pytest.raises(KeyError, match="time"):
        CSVSource(path)
