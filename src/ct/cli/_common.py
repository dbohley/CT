"""Shared argument parsing and printing for the ``ct-*`` scripts.

Every script accepts ``--config`` plus dedicated flags for the fields you change
most often, and ``--set dotted.key=value`` for everything else. The resolved
config is written next to each run's artifacts so a result can always be traced
back to the exact inputs that produced it.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from ct.config import RunConfig, parse_set_overrides


def add_common_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    g = parser.add_argument_group("configuration")
    g.add_argument("--config", "-c", default=None, help="YAML config (name or path)")
    g.add_argument("--name", default=None, help="run name; names the outputs/ subdirectory")
    g.add_argument("--output-dir", default=None, help="override the output directory")
    g.add_argument("--source", default=None, help="source name, e.g. sinusoid|lujan|rc_piecewise|csv")
    g.add_argument("--duration", type=float, default=None, help="record length [s]")
    g.add_argument("--fs", type=float, default=None, help="sample rate [Hz]")
    g.add_argument("--calib-seconds", type=float, default=None, help="calibration window [s]")
    g.add_argument("--horizon", type=float, default=None, help="forecast horizon h [s]")
    g.add_argument("--seed", type=int, default=None, help="RNG seed")
    g.add_argument("--noise-std", type=float, default=None, help="sensor noise std dev")
    g.add_argument("--kmax", type=int, default=None, help="max harmonics considered")
    g.add_argument("--energy-threshold", type=float, default=None, help="K-selection energy fraction")
    g.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="override any config field, e.g. --set source.params.n=3 (repeatable)",
    )
    g.add_argument("--plot", action="store_true", help="write diagnostic figures")
    g.add_argument("--quiet", action="store_true", help="suppress the summary tables")
    return parser


_FLAG_TO_KEY = {
    "name": "name",
    "output_dir": "output_dir",
    "source": "source.name",
    "duration": "duration",
    "fs": "fs",
    "calib_seconds": "calib_seconds",
    "horizon": "horizon",
    "seed": "seed",
    "noise_std": "source.params.noise_std",
    "kmax": "identifier.params.Kmax",
    "energy_threshold": "identifier.params.energy_threshold",
}


def resolve_config(args: argparse.Namespace) -> RunConfig:
    """Config file (or defaults) with every CLI override applied."""
    cfg = RunConfig.from_yaml(args.config) if args.config else RunConfig()
    overrides: dict[str, Any] = {
        key: getattr(args, flag)
        for flag, key in _FLAG_TO_KEY.items()
        if getattr(args, flag, None) is not None
    }
    overrides.update(parse_set_overrides(getattr(args, "overrides", None)))
    return cfg.apply_overrides(overrides)


# -- printing -----------------------------------------------------------------


def header(text: str) -> None:
    print(f"\n{text}\n{'=' * len(text)}")


def print_table(rows: list[dict[str, Any]], columns: list[str] | None = None) -> None:
    """Fixed-width table, so terminal output is diffable between runs."""
    if not rows:
        print("(no rows)")
        return
    columns = columns or list(rows[0])
    cells = [[_fmt(r.get(c, "")) for c in columns] for r in rows]
    widths = [max(len(c), *(len(row[i]) for row in cells)) for i, c in enumerate(columns)]
    print("  ".join(c.ljust(w) for c, w in zip(columns, widths)))
    print("  ".join("-" * w for w in widths))
    for row in cells:
        print("  ".join(c.ljust(w) for c, w in zip(row, widths)))


def print_kv(d: dict[str, Any], indent: int = 0) -> None:
    pad = " " * indent
    width = max((len(str(k)) for k in d), default=0)
    for k, v in d.items():
        if isinstance(v, dict):
            print(f"{pad}{k}:")
            print_kv(v, indent + 2)
        else:
            print(f"{pad}{str(k).ljust(width)}  {_fmt(v)}")


def _fmt(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:.6g}"
    if isinstance(v, (np.floating, np.integer)):
        return _fmt(v.item())
    if isinstance(v, np.ndarray):
        return np.array2string(v, precision=4, max_line_width=200)
    return str(v)


def save_json(obj: Any, path: str | Path) -> Path:
    """Persist a summary dict; numpy scalars/arrays are converted, not crashed on."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as fh:
        json.dump(obj, fh, indent=2, default=_json_default)
    return path


def _json_default(o: Any) -> Any:
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer, np.bool_)):
        return o.item()
    if isinstance(o, Path):
        return str(o)
    return str(o)
