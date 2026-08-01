"""CSV-backed source — the seam for real sensor data.

Once teammates supply a recording, it plugs in here with no change anywhere else
in the pipeline. ``fs`` is inferred from the timestamps unless given explicitly.
There is no ``y_clean`` and no ``truth``, which is precisely why nothing in the
estimator is allowed to depend on those fields.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterator

import numpy as np
import pandas as pd

from ct.registry import register_source
from ct.types import SignalBatch


@register_source("csv")
class CSVSource:
    """Reads ``(t, y)`` columns from a CSV file."""

    def __init__(
        self,
        path: str | Path,
        t_column: str = "t",
        y_column: str = "y",
        fs: float | None = None,
        scale: float = 1.0,
        offset: float = 0.0,
    ) -> None:
        self.path = Path(path)
        if not self.path.exists():
            raise FileNotFoundError(f"CSV source file not found: {self.path}")
        df = pd.read_csv(self.path)
        for col in (t_column, y_column):
            if col not in df.columns:
                raise KeyError(f"column '{col}' not in {self.path} (has {list(df.columns)})")
        self._t = df[t_column].to_numpy(dtype=float)
        self._y = df[y_column].to_numpy(dtype=float) * scale + offset
        self._y_clean = (
            df["y_clean"].to_numpy(dtype=float) * scale + offset
            if "y_clean" in df.columns
            else None
        )
        self.fs = float(fs) if fs is not None else self._infer_fs()

    def _infer_fs(self) -> float:
        if self._t.size < 2:
            raise ValueError("need at least 2 samples to infer fs; pass fs explicitly")
        dt = np.diff(self._t)
        median = float(np.median(dt))
        jitter = float(np.max(np.abs(dt - median)) / median) if median > 0 else np.inf
        if jitter > 0.05:
            raise ValueError(
                f"timestamps in {self.path} are non-uniform (max deviation {jitter:.1%} of "
                "the median step). Resample before use or pass fs explicitly."
            )
        return 1.0 / median

    def batch(self, duration_s: float | None = None, t0: float = 0.0) -> SignalBatch:
        m = self._t >= t0
        if duration_s is not None:
            m &= self._t < t0 + duration_s
        return SignalBatch(
            t=self._t[m],
            y=self._y[m],
            fs=self.fs,
            y_clean=None if self._y_clean is None else self._y_clean[m],
            truth=None,
        )

    def stream(
        self, duration_s: float | None = None, t0: float = 0.0
    ) -> Iterator[tuple[float, float]]:
        b = self.batch(duration_s, t0)
        yield from zip(b.t.tolist(), b.y.tolist())

    @property
    def duration(self) -> float:
        return float(self._t[-1] - self._t[0]) if self._t.size > 1 else 0.0

    def __repr__(self) -> str:
        return f"CSVSource({self.path.name}, N={self._t.size}, fs={self.fs:.4g})"


def write_csv(batch: SignalBatch, path: str | Path) -> Path:
    """Write a batch to a CSV that :class:`CSVSource` can read back."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {"t": batch.t, "y": batch.y}
    if batch.y_clean is not None:
        data["y_clean"] = batch.y_clean
    pd.DataFrame(data).to_csv(path, index=False)
    return path
