"""Per-tick records, written as JSONL.

One line per tick, one file per run. JSONL rather than a binary format because the first
thing anyone does with a run that went wrong is grep it, and because ``ct-compare`` has to
read a controller log and a phantom log written by two different processes.

Every record carries the host monotonic timestamp, which is what makes cross-process
alignment possible at all — see :mod:`ct.phantom.driver` for the other end of that.

Records are buffered and flushed periodically rather than written per tick: a 200 Hz loop
would otherwise spend a meaningful share of its budget in ``write``, and a missed deadline
caused by logging would be an unusually silly failure.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterator

import numpy as np


def _default(o: Any) -> Any:
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer, np.bool_)):
        return o.item()
    if isinstance(o, Path):
        return str(o)
    if hasattr(o, "value"):  # enums, including ProcedureState
        return o.value
    return str(o)


class TelemetryWriter:
    """Buffered JSONL sink."""

    def __init__(self, path: str | Path, flush_every: int = 200, enabled: bool = True) -> None:
        self.path = Path(path)
        self.flush_every = int(flush_every)
        self.enabled = enabled
        self.written = 0
        self._buf: list[str] = []
        self._fh = None
        if self.enabled:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._fh = open(self.path, "w")

    def write(self, record: dict[str, Any]) -> None:
        if not self.enabled or self._fh is None:
            return
        self._buf.append(json.dumps(record, default=_default))
        self.written += 1
        if len(self._buf) >= self.flush_every:
            self.flush()

    def flush(self) -> None:
        if self._fh is None or not self._buf:
            return
        self._fh.write("\n".join(self._buf) + "\n")
        self._fh.flush()
        self._buf.clear()

    def close(self) -> None:
        self.flush()
        if self._fh is not None:
            self._fh.close()
            self._fh = None

    def __enter__(self) -> TelemetryWriter:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def read_jsonl(path: str | Path) -> Iterator[dict[str, Any]]:
    """Stream records back. Blank and malformed lines are skipped.

    Tolerant on purpose: a run killed mid-flush leaves a truncated final line, and that
    log is usually the most interesting one to read.
    """
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def load_jsonl(path: str | Path) -> list[dict[str, Any]]:
    return list(read_jsonl(path))


def to_arrays(records: list[dict[str, Any]], keys: list[str]) -> dict[str, np.ndarray]:
    """Columnar view of selected keys, with ``nan`` where a record lacked one.

    Records are ragged by design — a tick before the tracker starts has no forecast — and
    padding with ``nan`` keeps everything on a common time axis for plotting.
    """
    out: dict[str, np.ndarray] = {}
    for key in keys:
        values = []
        for record in records:
            value = record.get(key)
            values.append(np.nan if value is None or isinstance(value, str) else float(value))
        out[key] = np.array(values, dtype=float)
    return out
