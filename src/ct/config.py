"""YAML experiment configs with CLI overrides.

A run is fully described by one YAML file, so results are reproducible by
committing the config next to them. Every field is reachable from the command
line via ``--set dotted.key=value``, and the common ones have dedicated flags.
"""

from __future__ import annotations

import copy
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_ROOT / "configs"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "outputs"


@dataclass
class RunConfig:
    """One experiment: where the signal comes from, how it is fit and tracked."""

    name: str = "run"
    duration: float = 180.0
    fs: float = 50.0
    calib_seconds: float = 60.0
    horizon: float = 0.25
    seed: int = 0

    source: dict[str, Any] = field(default_factory=lambda: {"name": "sinusoid", "params": {}})
    identifier: dict[str, Any] = field(
        default_factory=lambda: {"name": "fft_harmonic", "params": {}}
    )
    tracker: dict[str, Any] = field(default_factory=lambda: {"name": "harmonic_ekf", "params": {}})

    # -- rig controller (session 002) -----------------------------------------
    # Optional, and `None` by default, so every estimator-only config predating the
    # hardware layer stays valid. Typed views are built at use time in `ct.hw.config`;
    # keeping them as plain dicts here is what lets `--set rig.buses.sensing.channel=can1`
    # keep working through `apply_overrides` without a schema round-trip.
    rig: dict[str, Any] | None = None
    procedure: dict[str, Any] | None = None
    latency: dict[str, Any] | None = None
    servo: dict[str, Any] | None = None

    output_dir: str | None = None

    def __post_init__(self) -> None:
        # YAML and the CLI both hand us whatever scalar type the text looked
        # like, so `fs: 100` arrives as an int. Coerce to the declared types once,
        # here, rather than defending against it at every use site.
        self.name = str(self.name)
        for field_name in ("duration", "fs", "calib_seconds", "horizon"):
            setattr(self, field_name, float(getattr(self, field_name)))
        self.seed = int(self.seed)
        if self.output_dir is not None:
            self.output_dir = str(self.output_dir)

    # -- construction ---------------------------------------------------------

    @classmethod
    def from_yaml(cls, path: str | Path) -> RunConfig:
        path = _resolve_config_path(path)
        with open(path) as fh:
            raw = yaml.safe_load(fh) or {}
        cfg = cls.from_dict(raw)
        if cfg.name == "run":
            cfg.name = path.stem
        return cfg

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> RunConfig:
        known = set(cls.__dataclass_fields__)
        unknown = set(raw) - known
        if unknown:
            raise ValueError(
                f"unknown config key(s): {sorted(unknown)}. Known keys: {sorted(known)}"
            )
        cfg = cls(**raw)
        cfg.validate()
        return cfg

    def validate(self) -> None:
        if self.fs <= 0:
            raise ValueError("fs must be positive")
        if self.duration <= 0:
            raise ValueError("duration must be positive")
        if not 0 < self.calib_seconds <= self.duration:
            raise ValueError("calib_seconds must be in (0, duration]")
        for key in ("source", "identifier", "tracker"):
            spec = getattr(self, key)
            if "name" not in spec:
                raise ValueError(f"config section '{key}' needs a 'name'")
            spec.setdefault("params", {})
        # The rig sections are validated properly when their typed views are built, in
        # `ct.hw.config`. All that is checked here is shape, so that a mistyped
        # `--set rig=nonsense` fails at parse time rather than three states into a run.
        for key in ("rig", "procedure", "latency", "servo"):
            spec = getattr(self, key)
            if spec is not None and not isinstance(spec, dict):
                raise ValueError(f"config section '{key}' must be a mapping, got {type(spec).__name__}")

    @property
    def has_rig(self) -> bool:
        """Whether this config describes a physical rig as well as an estimator run."""
        return self.rig is not None

    # -- overriding -----------------------------------------------------------

    def apply_overrides(self, overrides: dict[str, Any]) -> RunConfig:
        """Return a copy with dotted-key overrides applied.

        ``{"fs": 100, "source.params.n": 3}`` sets a top-level field and a nested
        source parameter respectively. Values arriving as strings from the CLI
        are coerced via YAML scalar rules, so ``n=3`` becomes an int.
        """
        data = copy.deepcopy(asdict(self))
        for dotted, value in overrides.items():
            if value is None:
                continue
            parts = dotted.split(".")
            node = data
            for p in parts[:-1]:
                if p not in node or not isinstance(node[p], dict):
                    node[p] = {}
                node = node[p]
            node[parts[-1]] = _coerce(value)
        return RunConfig.from_dict(data)

    # -- paths ----------------------------------------------------------------

    def run_dir(self) -> Path:
        d = Path(self.output_dir) if self.output_dir else DEFAULT_OUTPUT_DIR / self.name
        d.mkdir(parents=True, exist_ok=True)
        return d

    def dump(self, path: str | Path | None = None) -> Path:
        """Write the resolved config next to the run's artifacts."""
        path = Path(path) if path else self.run_dir() / "config.resolved.yaml"
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as fh:
            yaml.safe_dump(asdict(self), fh, sort_keys=False)
        return path

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def __str__(self) -> str:
        return json.dumps(asdict(self), indent=2)


def _resolve_config_path(path: str | Path) -> Path:
    p = Path(path)
    if p.exists():
        return p
    # allow bare names: `--config lujan_n2`
    for cand in (CONFIG_DIR / p.name, CONFIG_DIR / f"{p.name}.yaml"):
        if cand.exists():
            return cand
    known = sorted(q.stem for q in CONFIG_DIR.glob("*.yaml")) if CONFIG_DIR.exists() else []
    raise FileNotFoundError(f"config '{path}' not found. Available in configs/: {known}")


def _coerce(value: Any) -> Any:
    """Turn CLI strings into YAML scalars; pass through anything already typed."""
    if not isinstance(value, str):
        return value
    try:
        return yaml.safe_load(value)
    except yaml.YAMLError:
        return value


def parse_set_overrides(pairs: list[str] | None) -> dict[str, Any]:
    """Parse ``--set a.b=c`` pairs into an override dict."""
    out: dict[str, Any] = {}
    for pair in pairs or []:
        if "=" not in pair:
            raise ValueError(f"--set expects key=value, got '{pair}'")
        key, value = pair.split("=", 1)
        out[key.strip()] = value.strip()
    return out
