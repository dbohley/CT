"""End-to-end runs, and proof that the three boundaries are actually swappable."""

from __future__ import annotations

import numpy as np
import pytest

from ct.config import RunConfig
from ct.registry import TRACKERS, build_source, register_tracker
from ct.run import run_pipeline, sweep_horizons

CONFIGS = ["sinusoid", "lujan_n2", "lujan_n3", "rc_piecewise", "rc_piecewise_k4"]


@pytest.fixture(scope="module")
def short_cfg():
    """A short run: enough breaths to identify and track, fast enough for CI."""
    return RunConfig(
        name="test_run",
        duration=150.0,
        fs=50.0,
        calib_seconds=60.0,
        horizon=0.25,
        seed=0,
        source={"name": "lujan", "params": {"n": 2, "a": 10.0, "noise_std": 0.15}},
    )


def test_pipeline_runs_end_to_end(short_cfg, tmp_path):
    result = run_pipeline(short_cfg.apply_overrides({"output_dir": str(tmp_path)}))

    assert result.ident.K == 2
    assert result.history.n_steps > 4000
    assert np.isfinite(result.history.s).all()
    assert result.summary["nis_mean"] == pytest.approx(1.0, abs=0.3)
    assert result.summary["tracking_rmse_vs_clean"] < result.summary["noise_std"]


@pytest.mark.parametrize("source_name", ["sinusoid", "lujan", "rc_piecewise"])
def test_pipeline_runs_on_every_synthetic_source(source_name, short_cfg, tmp_path):
    """Nothing in the estimator may care which generator produced the samples."""
    cfg = short_cfg.apply_overrides(
        {
            "source.name": source_name,
            "source.params": {"noise_std": 0.15},
            "output_dir": str(tmp_path),
            # rc_piecewise needs a tighter threshold; see configs/rc_piecewise.yaml.
            "identifier.params.energy_threshold": 0.999,
        }
    )
    result = run_pipeline(cfg)
    assert result.summary["forecast"]["rmse"] < 0.05 * 10.0  # < 5% of amplitude
    assert result.summary["forecast"]["bias"] == pytest.approx(0.0, abs=0.05)


def test_pipeline_runs_from_a_csv_trace(short_cfg, tmp_path):
    """The path real sensor data will take."""
    from ct.sources.csv_source import write_csv

    src = build_source("lujan", {"fs": 50.0, "noise_std": 0.15, "seed": 0, "n": 2})
    path = write_csv(src.batch(150.0), tmp_path / "trace.csv")

    cfg = short_cfg.apply_overrides(
        {"source.name": "csv", "source.params": {"path": str(path)}, "output_dir": str(tmp_path)}
    )
    result = run_pipeline(cfg)
    assert result.ident.K == 2
    assert result.batch.truth is None  # real-data path carries no ground truth
    assert result.summary["nis_mean"] == pytest.approx(1.0, abs=0.3)


@pytest.mark.parametrize("name", CONFIGS)
def test_shipped_configs_are_valid_and_runnable(name, tmp_path):
    """Every config in configs/ must load and produce a finite run."""
    cfg = RunConfig.from_yaml(name).apply_overrides(
        {"duration": 150.0, "calib_seconds": 60.0, "output_dir": str(tmp_path / name)}
    )
    result = run_pipeline(cfg)
    assert result.ident.K >= 1
    assert np.isfinite(result.history.y_pred).all()


def test_forecast_error_grows_smoothly_with_the_horizon(short_cfg):
    """A jump or sawtooth here is the signature of the common-phase-rotation bug
    (see tests/test_forecast.py); correct time-advance degrades gradually."""
    horizons = [0.05, 0.1, 0.2, 0.4, 0.8, 1.2]
    rows = sweep_horizons(short_cfg, horizons)
    rmse = np.array([r["rmse"] for r in rows])

    assert rmse[0] < 0.05
    assert rmse[-1] >= rmse[0], "error should not be lower at a 24x longer horizon"
    # Non-decreasing to within sampling noise — a longer horizon cannot genuinely
    # be easier to predict, but neighbouring horizons differ by very little here.
    assert np.all(np.diff(rmse) > -0.05 * rmse[:-1])
    # No single step may blow up: smooth growth, not a discontinuity. The
    # common-phase-rotation bug produces exactly such a jump.
    assert np.max(rmse[1:] / np.maximum(rmse[:-1], 1e-12)) < 6.0


def test_swapping_the_tracker_needs_no_other_change(short_cfg, tmp_path):
    """The point of the registry: a new tracker reaches the pipeline by name."""

    @register_tracker("test_persistence_tracker")
    class PersistenceTracker:
        """Deliberately trivial: predicts the last measurement, forever."""

        def __init__(self, unused: float = 0.0):
            self.unused = unused

        def init(self, result, t0=None):
            from ct.layout import StateLayout

            self.layout = StateLayout(result.K)
            self._s = np.array(result.s0, float)
            self._P = np.array(result.P0, float)
            self._last = 0.0
            self._t = result.t0

        def step(self, t, y):
            from ct.types import TrackerStep

            innovation = y - self._last
            self._last = y
            self._t = t
            return TrackerStep(
                t=t, s=self._s.copy(), P=self._P.copy(),
                y_pred=self._last, innovation=innovation, S=1.0, nis=innovation**2,
            )

        def forecast(self, h):
            return self._last

        @property
        def state(self):
            return self._s.copy(), self._P.copy()

        @property
        def config(self):
            return {"name": "test_persistence_tracker"}

    assert "test_persistence_tracker" in TRACKERS

    cfg = short_cfg.apply_overrides(
        {"tracker.name": "test_persistence_tracker", "tracker.params": {}, "output_dir": str(tmp_path)}
    )
    result = run_pipeline(cfg)
    assert result.history.n_steps > 4000

    # And it should be clearly worse than the EKF — proving the swap took effect.
    ekf = run_pipeline(short_cfg.apply_overrides({"output_dir": str(tmp_path)}))
    assert result.summary["forecast"]["rmse"] > 5 * ekf.summary["forecast"]["rmse"]


def test_swapping_the_identifier_needs_no_other_change(short_cfg, tmp_path):
    from ct.registry import IDENTIFIERS, register_identifier

    @register_identifier("test_fixed_K_identifier")
    class FixedKIdentifier:
        """Wraps the real identifier but pins K, standing in for any alternative
        Stage-1 method."""

        def __init__(self, K: int = 3):
            self.K = K

        def identify(self, batch):
            from ct.identification.fft_identifier import FFTHarmonicIdentifier

            return FFTHarmonicIdentifier(Kmax=8, K_override=self.K).identify(batch)

    assert "test_fixed_K_identifier" in IDENTIFIERS
    cfg = short_cfg.apply_overrides(
        {
            "identifier.name": "test_fixed_K_identifier",
            "identifier.params": {"K": 3},
            "output_dir": str(tmp_path),
        }
    )
    assert run_pipeline(cfg).ident.K == 3


def test_unknown_names_list_what_is_registered():
    from ct.registry import build_source as bs

    with pytest.raises(KeyError, match="Registered:"):
        bs("no_such_source")


# -- config layer --------------------------------------------------------------


def test_config_overrides_reach_nested_fields():
    cfg = RunConfig().apply_overrides({"fs": "100", "source.params.n": "3", "source.name": "lujan"})
    assert cfg.fs == 100.0 and isinstance(cfg.fs, float)
    assert cfg.source["params"]["n"] == 3
    assert cfg.source["name"] == "lujan"


def test_config_rejects_unknown_keys():
    with pytest.raises(ValueError, match="unknown config key"):
        RunConfig.from_dict({"nonsense": 1})


def test_config_rejects_an_impossible_calibration_window():
    with pytest.raises(ValueError, match="calib_seconds"):
        RunConfig(duration=60.0, calib_seconds=90.0).validate()


def test_config_resolves_bare_names():
    assert RunConfig.from_yaml("lujan_n2").source["name"] == "lujan"


def test_missing_config_lists_the_available_ones():
    with pytest.raises(FileNotFoundError, match="Available in configs/"):
        RunConfig.from_yaml("does_not_exist")


def test_config_roundtrips_through_yaml(tmp_path):
    cfg = RunConfig.from_yaml("rc_piecewise")
    path = cfg.dump(tmp_path / "resolved.yaml")
    assert RunConfig.from_yaml(path).to_dict() == cfg.to_dict()
