"""The simulated rig.

Off-limits to ``control/``, ``hw/`` and ``rt/`` (rule 3 in ``CLAUDE.md``). This is the
hardware layer's ``truth``: if the controller could import it, running the controller in
simulation would stop being evidence about the real rig. ``test_boundaries.py`` walks the
import graph to make sure it stays that way.
"""

from __future__ import annotations

from ct.plant.rig import (
    MotorPhysics,
    RigTruth,
    SimulatedMotor,
    SimulatedRig,
    SimulatedSensor,
    TissueModel,
)

__all__ = [
    "SimulatedRig",
    "SimulatedMotor",
    "SimulatedSensor",
    "MotorPhysics",
    "TissueModel",
    "RigTruth",
]
