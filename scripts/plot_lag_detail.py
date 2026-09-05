#!/usr/bin/env python3
"""Zoom in on a few breaths of a bench run to make the sensing lag visible by eye.

``ct-compare`` reports the tactile-vs-phantom lag as one number (0.349s on
``20260903-171153``), which is easy to doubt when the same two traces, plotted over a whole
180s hold, look almost superimposed at that scale. This script answers "does the mechanism
really cause a delay this size" by cropping to a handful of breath periods and drawing two
panels:

- **unshifted** -- the sensor and the phantom's true position, plotted as recorded. The
  sensor visibly trails.
- **lag-removed** -- the same window, with the sensor's time axis shifted backward by the
  measured lag. If ``compare_logs`` measured the right number, the two traces should now
  fall on top of each other.

Uses the same alignment convention as ``ct.phantom.driver.compare_logs`` (a positive lag
means the sensor trails the phantom) so the shift direction here can never disagree with what
``ct-compare`` reports.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ct.phantom.driver import SENSOR_SIGN, compare_logs
from ct.rt.telemetry import load_jsonl


def _phantom_series(records: list[dict], field: str) -> tuple[np.ndarray, np.ndarray]:
    """Phantom ``(t, y)`` for a named field, mirroring ``compare_logs``'s own extraction."""
    rows = [r for r in records if r.get(field) is not None]
    t = np.array([r["t"] for r in rows], dtype=float)
    y = np.array([r[field] for r in rows], dtype=float)
    return t, y


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path, help="run directory, e.g. outputs/approach_and_seat/<ts>")
    p.add_argument("--phase", default="standoff_hold")
    p.add_argument("--sensor", default="tactile_mm")
    p.add_argument("--truth", default="measured_mm")
    p.add_argument("--breaths", type=float, default=3.0, help="how many breath periods to show")
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args()

    controller_path = args.run / "samples.jsonl"
    phantom_path = args.run / "phantom" / "samples.jsonl"
    out = args.out or args.run / "lag_detail.png"

    metrics = compare_logs(
        phantom_path, controller_path,
        phase=args.phase, phantom_field=args.truth, sensor_field=args.sensor,
    )
    lag_s = metrics["lag_s"]
    breath_s = metrics["breath_period_s"]
    sign = SENSOR_SIGN.get(args.sensor, 1.0)

    phantom = load_jsonl(phantom_path)
    controller = [r for r in load_jsonl(controller_path)
                  if r.get(args.sensor) is not None and r.get("phase") == args.phase]
    tp, yp = _phantom_series(phantom, args.truth)
    tc = np.array([r["t"] for r in controller], dtype=float)
    yc = np.array([r[args.sensor] for r in controller], dtype=float) * sign

    # A window a few breaths wide, centered in the hold rather than at its edge, so it isn't
    # contaminated by whatever transition preceded standoff_hold.
    t0, t1 = tc[0], tc[-1]
    span = args.breaths * breath_s
    center = 0.5 * (t0 + t1)
    w0, w1 = max(t0, center - span / 2), min(t1, center + span / 2)

    mp = (tp >= w0 - lag_s) & (tp <= w1 + lag_s)  # pad so the shifted curve still has data
    mc = (tc >= w0) & (tc <= w1)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=False)

    ax1.plot(tp[mp] - w0, yp[mp] - yp[mp].mean(), label=f"phantom truth ({args.truth})",
              color="tab:blue", lw=1.6)
    ax1b = ax1.twinx()
    ax1b.plot(tc[mc] - w0, yc[mc] - yc[mc].mean(), label=f"sensor ({args.sensor})",
               color="tab:orange", lw=1.6)
    ax1.set_title(f"unshifted -- sensor visibly trails truth (measured lag {lag_s * 1000:.0f} ms)")
    ax1.set_ylabel("truth, demeaned (mm)", color="tab:blue")
    ax1b.set_ylabel("sensor, demeaned (mm)", color="tab:orange")
    ax1.set_xlabel("time (s)")
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines1b, labels1b = ax1b.get_legend_handles_labels()
    ax1.legend(lines1 + lines1b, labels1 + labels1b, loc="upper right")

    # Positive lag_s means the sensor lags: shifting the sensor's own time axis backward by
    # lag_s (t_shown = t_recorded - lag_s) is what compare_logs's `_apply_lag` does with an
    # integer sample shift; doing it here in continuous time is the same operation.
    ax2.plot(tp[mp] - w0, yp[mp] - yp[mp].mean(), label=f"phantom truth ({args.truth})",
              color="tab:blue", lw=1.6)
    ax2b = ax2.twinx()
    ax2b.plot(tc[mc] - lag_s - w0, yc[mc] - yc[mc].mean(),
               label=f"sensor ({args.sensor}), shifted back {lag_s * 1000:.0f} ms",
               color="tab:orange", lw=1.6, ls="--")
    ax2.set_title("lag removed -- shifting the sensor back by the measured lag should snap it onto truth")
    ax2.set_ylabel("truth, demeaned (mm)", color="tab:blue")
    ax2b.set_ylabel("sensor, demeaned (mm)", color="tab:orange")
    ax2.set_xlabel("time (s)")
    lines2, labels2 = ax2.get_legend_handles_labels()
    lines2b, labels2b = ax2b.get_legend_handles_labels()
    ax2.legend(lines2 + lines2b, labels2 + labels2b, loc="upper right")

    fig.suptitle(
        f"{args.run.name} -- {args.phase}: {args.sensor} vs {args.truth}, "
        f"{args.breaths:g} breaths (T={breath_s:.2f}s)"
    )
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")
    print(f"lag_s={lag_s:.4f}  breath_period_s={breath_s:.3f}  "
          f"correlation(unshifted)={metrics['correlation_unshifted']:.3f}  "
          f"correlation(aligned)={metrics['correlation']:.3f}")


if __name__ == "__main__":
    main()
