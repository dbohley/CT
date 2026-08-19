"""Figures for a rig run.

Deliberately few. Three questions are worth a picture after an insertion, and the rest is
better read out of the telemetry log:

    procedure.png  what the axes did, and when each state handed over
    gate.png       what the firing gate saw, and why it opened when it did
    sensing.png    what the estimator made of the tactile signal
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import numpy as np  # noqa: E402

STATE_COLORS = {
    "approach": "#cfe8ff",
    "estimate": "#d9f2d9",
    "insert": "#ffe4c4",
    "advance": "#f3d9f3",
    "withdraw": "#eeeeee",
    "retract": "#eeeeee",
    "done": "#e8e8e8",
    "fault": "#ffd0d0",
}


def _shade_states(ax: Any, records: list[dict[str, Any]]) -> None:
    """Colour the background by procedure state, so transitions are readable at a glance."""
    if not records:
        return
    start = records[0]["t"]
    current = records[0]["state"]
    for record in records:
        if record["state"] != current:
            ax.axvspan(start, record["t"], color=STATE_COLORS.get(current, "#ffffff"), zorder=0)
            start, current = record["t"], record["state"]
    ax.axvspan(start, records[-1]["t"], color=STATE_COLORS.get(current, "#ffffff"), zorder=0)


def _column(records: list[dict[str, Any]], key: str) -> np.ndarray:
    return np.array([r.get(key, np.nan) if r.get(key) is not None else np.nan
                     for r in records], dtype=float)


def plot_run(result: Any, out_dir: str | Path) -> list[Path]:
    """Write the run's figures. Returns the paths written."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    records = result.records
    if not records:
        return []

    t = _column(records, "t")
    written: list[Path] = []

    # -- procedure ------------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    _shade_states(axes[0], records)
    axes[0].plot(t, _column(records, "base_mm"), label="base", lw=1.2)
    axes[0].plot(t, _column(records, "needle_mm"), label="needle", lw=1.2)
    axes[0].set_ylabel("axis [mm]")
    axes[0].legend(loc="upper left", fontsize=8)
    axes[0].set_title(f"{result.config.name} — procedure "
                      f"(final: {result.final_state.value})")

    _shade_states(axes[1], records)
    axes[1].plot(t, _column(records, "tactile_mm"), lw=1.0, label="tactile deflection")
    axes[1].plot(t, _column(records, "forecast_mm"), lw=0.8, alpha=0.8,
                 label="forecast at t+h")
    axes[1].set_ylabel("deflection [mm]")
    axes[1].legend(loc="upper left", fontsize=8)

    _shade_states(axes[2], records)
    axes[2].plot(t, _column(records, "depth_mm"), lw=1.2, color="#8b0000")
    axes[2].axhline(0.0, color="k", lw=0.6, ls=":")
    axes[2].set_ylabel("insertion depth [mm]")
    axes[2].set_xlabel("t [s]")

    for entry in result.assembly.procedure.history:
        for ax in axes:
            ax.axvline(entry["t"], color="k", lw=0.7, ls="--", alpha=0.6)

    fig.tight_layout()
    path = out_dir / "procedure.png"
    fig.savefig(path, dpi=130)
    plt.close(fig)
    written.append(path)

    # -- gate -----------------------------------------------------------------
    std = _column(records, "forecast_std_mm")
    if np.isfinite(std).any():
        fig, axes = plt.subplots(2, 1, figsize=(11, 5.5), sharex=True)
        _shade_states(axes[0], records)
        axes[0].plot(t, std, lw=1.0, color="#444")
        axes[0].set_ylabel("forecast std [mm]")
        axes[0].set_title("what the gate saw — the covariance an LMS predictor cannot give")

        _shade_states(axes[1], records)
        axes[1].plot(t, _column(records, "h"), lw=1.0, label="h")
        axes[1].plot(t, _column(records, "nis"), lw=0.6, alpha=0.6, label="NIS")
        axes[1].set_ylabel("h [s] / NIS")
        axes[1].set_xlabel("t [s]")
        axes[1].legend(loc="upper left", fontsize=8)
        fig.tight_layout()
        path = out_dir / "gate.png"
        fig.savefig(path, dpi=130)
        plt.close(fig)
        written.append(path)

    # -- sensing vs truth -----------------------------------------------------
    plant = result.assembly.plant
    if plant is not None and plant.truth.history:
        history = plant.truth.history
        tt = np.array([h["t"] for h in history])
        fig, ax = plt.subplots(figsize=(11, 4))
        ax.plot(tt, [h["skin_x"] for h in history], lw=1.0, label="skin (truth)")
        ax.plot(tt, [h["needle_tip_x"] for h in history], lw=1.0, label="needle tip")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("rig frame x [mm]")
        ax.set_title("simulation ground truth — validation only, never seen by the controller")
        ax.legend(loc="upper left", fontsize=8)
        fig.tight_layout()
        path = out_dir / "sensing.png"
        fig.savefig(path, dpi=130)
        plt.close(fig)
        written.append(path)

    return written
