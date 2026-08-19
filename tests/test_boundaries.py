"""The architectural rules, enforced rather than asserted in prose.

``CLAUDE.md`` states four hard rules. Three of them are checkable by machine, and a rule
nobody checks is a rule that decays into a comment. These tests are what let the README
claim that swapping simulation for hardware is a config change: if the controller could
reach into the simulated plant, or convert units behind geometry's back, the claim would
quietly stop being true and nothing would notice.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src" / "ct"

#: The controller may not see the simulated rig. `ct/rig.py` is the orchestrator that
#: wires one to the other -- the same role `run.py` plays for the estimator -- so it is
#: the single documented exception.
PLANT_FORBIDDEN_IN = ("control", "hw", "rt", "phantom")

#: The estimator must keep working, and keep being testable, without the hardware layer.
ESTIMATOR_MODULES = ("identification", "tracking", "sources")
ESTIMATOR_FILES = ("forecast.py", "layout.py", "types.py", "interfaces.py", "run.py")
HARDWARE_PACKAGES = ("ct.hw", "ct.control", "ct.rt", "ct.plant")


def _python_files(*parts: str) -> list[Path]:
    root = SRC.joinpath(*parts)
    if root.is_file():
        return [root]
    return sorted(root.rglob("*.py"))


def _string_literal_lines(text: str, path: Path) -> set[int]:
    """Line numbers occupied by string literals, so prose is not mistaken for code."""
    lines: set[int] = set()
    for node in ast.walk(ast.parse(text, filename=str(path))):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            lines.update(range(node.lineno, (node.end_lineno or node.lineno) + 1))
    return lines


def _imported_modules(path: Path) -> set[str]:
    """Every module name this file imports, however it spells the import."""
    tree = ast.parse(path.read_text(), filename=str(path))
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            found.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            found.add(node.module)
    return found


@pytest.mark.parametrize("package", PLANT_FORBIDDEN_IN)
def test_controller_cannot_import_the_simulated_plant(package):
    """Rule 3. The hardware-layer equivalent of 'no estimator code may read truth'.

    A controller that can see simulation ground truth proves nothing when you run it in
    simulation, which would make the entire sim-first strategy worthless.
    """
    offenders = [
        (path.relative_to(SRC), module)
        for path in _python_files(package)
        for module in _imported_modules(path)
        if module.startswith("ct.plant")
    ]
    assert not offenders, (
        f"{package}/ imports the simulated plant: {offenders}. Only ct/rig.py may wire the "
        "two together."
    )


def test_the_orchestrator_is_the_only_bridge():
    """`ct/rig.py` is allowed to see both — and is expected to, or nothing is wired up."""
    modules = _imported_modules(SRC / "rig.py")
    assert any(m.startswith("ct.control") for m in modules)
    # The plant import is deliberately function-local, so check the source text instead.
    assert "ct.plant.rig" in (SRC / "rig.py").read_text()


@pytest.mark.parametrize("target", ESTIMATOR_MODULES + ESTIMATOR_FILES)
def test_estimator_does_not_depend_on_the_hardware_layer(target):
    """The dependency runs one way. `import ct` must not pull in the rig."""
    offenders = [
        (path.relative_to(SRC), module)
        for path in _python_files(target)
        for module in _imported_modules(path)
        if any(module.startswith(pkg) for pkg in HARDWARE_PACKAGES)
    ]
    assert not offenders, f"{target} imports the hardware layer: {offenders}"


def test_importing_ct_does_not_import_the_hardware_layer():
    """`import ct` stays cheap and dependency-light.

    Concretely: the estimator must import on a machine with no CAN adapter and no
    python-can, because that is most machines, including CI.
    """
    import subprocess
    import sys

    result = subprocess.run(
        [sys.executable, "-c",
         "import ct, sys; "
         "bad=[m for m in sys.modules if m.startswith(('ct.hw','ct.control','ct.plant','ct.rt'))]; "
         "print(bad)"],
        capture_output=True, text=True, check=True,
    )
    assert result.stdout.strip() == "[]", (
        f"`import ct` dragged in the hardware layer: {result.stdout.strip()}"
    )


def test_only_geometry_converts_raw_units_to_millimetres():
    """Rule 2. `counts_per_mm` may be *named* elsewhere, but never computed with.

    This is rule 1 (`layout.py` owns state ordering) applied one layer down. It is what
    makes recalibrating the rig a one-file change instead of a hunt through the
    controller, and what keeps a procedure state from quietly growing its own idea of how
    many counts make a millimetre.
    """
    offenders: list[str] = []
    for path in SRC.rglob("*.py"):
        if path.name == "geometry.py":
            continue
        text = path.read_text()
        prose = _string_literal_lines(text, path)
        for lineno, line in enumerate(text.splitlines(), start=1):
            if "counts_per_mm" not in line:
                continue
            stripped = line.strip()
            # Naming it is fine — in a comment, a docstring, a config key or a keyword
            # argument. Computing with it is what the rule forbids.
            if stripped.startswith("#") or lineno in prose:
                continue
            before, _, after = line.partition("counts_per_mm")
            if before.rstrip().endswith(("*", "/")) or after.lstrip().startswith(("*", "/")):
                offenders.append(f"{path.relative_to(SRC)}:{lineno}: {stripped}")
    assert not offenders, (
        "raw-unit arithmetic outside geometry.py:\n" + "\n".join(offenders)
    )


def test_no_module_calls_the_wall_clock_directly():
    """Nothing in the control path may read time except through its ``Clock``.

    A single ``time.monotonic()`` in a procedure state would make the whole four-state
    procedure untestable under ``SimClock`` — it would block on real seconds while the
    rest of the system ran in simulated ones.
    """
    allowed = {"rt/clock.py", "hw/bus/canfd_rh02.py"}
    offenders = []
    for package in ("control", "rt"):
        for path in _python_files(package):
            rel = str(path.relative_to(SRC))
            if rel in allowed:
                continue
            text = path.read_text()
            for call in ("time.monotonic(", "time.time(", "time.sleep("):
                if call in text:
                    offenders.append(f"{rel}: {call}")
    assert not offenders, (
        "direct wall-clock use in the control path:\n" + "\n".join(offenders)
        + "\nAsk the injected Clock instead, or SimClock-based tests stop working."
    )
