"""Shared helpers for the EKF parameter-sweep tooling.

Split out so ``collect_sweep_trial.py`` (physical collection, one trial per invocation) and
``sweep_ekf_params.py`` (offline sweep) agree on what "profile name" and "valid trial" mean,
rather than each carrying its own copy that can silently drift apart -- the same reasoning as
``ct.diagnostics.metrics.is_frequency_locked`` (session 015/016).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent

DEFAULT_PROFILES = ["emma", "derek", "jake", "junrong", "patient1"]
DEFAULT_TRIALS_PER_PROFILE = 2
DEFAULT_OUT_ROOT = REPO_ROOT / "outputs" / "param_sweep_runs"

# Calibrated against the six real summary.json files on disk as of session 015/016: four good
# runs measured contact_t of 2.36-6.43s; two bad ones measured 0.0063s and 0.0002s (the exact
# near-instant-contact signature session 012 diagnosed -- a drifted tactile zero already past
# the contact threshold before any real approach happened). 1.0s is a wide margin between them,
# not a guessed number.
DEFAULT_MIN_CONTACT_S = 1.0


def resolve_profile_name(name: str) -> str:
    """``run_approach_and_seat.py --profile`` wants the bare name run_breathing_profile.py
    resolves against breathe_profiles/ -- accept either that or the short form (e.g. "emma")
    the user actually thinks in."""
    if name.endswith("_normal_breathing"):
        return name
    if (REPO_ROOT / "breathe_profiles" / f"{name}_normal_breathing.csv").exists():
        return f"{name}_normal_breathing"
    return name


def validate_trial(summary: dict[str, Any], min_contact_s: float = DEFAULT_MIN_CONTACT_S) -> tuple[bool, str]:
    """Did this attempt actually collect a real 180s hold, or should it be discarded?

    Three checks, all against ``summary.json`` -- ``fault_reason`` alone is not enough: run
    20260903-152958 measured ``fault_reason: None`` with ``contact_t=0.0063s`` and never
    reached ``standoff_hold`` at all, so a validator trusting only the fault flag would have
    accepted a run that produced no usable hold.
    """
    if summary.get("fault_reason"):
        return False, f"faulted: {summary['fault_reason']}"
    if summary.get("phase_reached") != "standoff_hold":
        return False, f"did not reach standoff_hold (stopped at '{summary.get('phase_reached')}')"
    contact_t = summary.get("contact_t")
    if contact_t is None or contact_t <= min_contact_s:
        return False, (
            f"contact_t={contact_t} is at or below {min_contact_s:.2f}s -- the near-instant-"
            f"contact signature of a drifted tactile zero (session 012), not a real approach"
        )
    return True, "ok"
