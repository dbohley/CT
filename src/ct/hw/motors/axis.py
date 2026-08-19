"""An axis: a motor, a codec, a bus and a calibration, presenting as millimetres.

This is the line rule 2 draws. Below it, raw motor units and CAN frames. Above it, only
millimetres in the rig frame — no procedure state ever sees a count.

There is deliberately **no separate simulated axis class**. Simulation swaps the *bus*,
not the axis: :class:`~ct.plant.rig.SimulatedMotor` speaks the same codec on a loopback
bus, so the identical ``CANAxis`` — the same packing, the same parsing, the same limit
checks — runs in a unit test and on hardware. A byte-packing bug therefore fails in
simulation instead of waiting for a needle to be attached. A parallel ``SimAxis`` would
have been easier and would have tested nothing.

Commands are sent on every call rather than latched, and the control loop calls exactly
one command method per axis per tick. That gives the periodic command stream MIT mode's
watchdog expects, and makes "the tick writes motor commands" true rather than aspirational.
"""

from __future__ import annotations

from typing import Any

from ct.geometry import AxisCalibration
from ct.hw.config import MotorConfig
from ct.hw.interfaces import Bus, MotorCodec


class AxisLimitError(RuntimeError):
    """A command was refused for being outside the axis's soft limits."""


class CANAxis:
    """One joint on a CAN bus."""

    def __init__(
        self,
        name: str,
        bus: Bus,
        codec: MotorCodec,
        config: MotorConfig,
        calibration: AxisCalibration,
        *,
        dry_run: bool = False,
    ) -> None:
        self.name = name
        self.bus = bus
        self.codec = codec
        self.config = config
        self.calibration = calibration
        self.dry_run = dry_run
        """When set, commands are counted and logged but never put on the bus.

        The first thing to run against real hardware: it exercises the bus, the codec and
        the geometry with the motors unable to move.
        """

        self._position_counts: float | None = None
        self._velocity_counts_s: float = 0.0
        self._current: float = 0.0
        self._stamp: float | None = None
        self._t: float = 0.0
        self._enabled = False
        self._hold_target_mm: float | None = None

        self.commands_sent = 0
        self.commands_suppressed = 0
        self.limit_violations = 0
        self.last_command: dict[str, Any] | None = None

        self._check_travel_fits_codec()

    def _check_travel_fits_codec(self) -> None:
        """Refuse an axis whose travel does not fit its codec's position range.

        Found the hard way. MIT mode packs position into 16 bits spanning ``[p_min,
        p_max]``, defaulting to ±12.5 rad — roughly two output-shaft turns. Ask for more
        than that and the codec *saturates silently*: the axis creeps to the end of the
        range and stops, feedback agrees with the (clipped) command, and nothing anywhere
        reports an error. Checking at construction turns a baffling afternoon into a
        message naming the two numbers that disagree.
        """
        span = getattr(self.codec, "position_range", None)
        if span is None:
            return
        lo_counts, hi_counts = span
        lo_mm, hi_mm = self.calibration.travel_mm
        needed = [self.calibration.to_counts(lo_mm), self.calibration.to_counts(hi_mm)]
        if min(needed) < lo_counts or max(needed) > hi_counts:
            raise AxisLimitError(
                f"axis '{self.name}': travel [{lo_mm:g}, {hi_mm:g}] mm needs codec positions "
                f"[{min(needed):.2f}, {max(needed):.2f}], outside '{self.codec.name}' range "
                f"[{lo_counts:g}, {hi_counts:g}]. Commands past the range saturate silently. "
                f"Widen the codec range via rig.motors.{self.name}.codec_params, reduce "
                "counts_per_mm, or use a codec with a wider position field."
            )

    # -- lifecycle -------------------------------------------------------------

    def enable(self) -> None:
        self._send(*self.codec.enable(self.config.can_id))
        self._enabled = True

    def disable(self) -> None:
        self._send(*self.codec.disable(self.config.can_id))
        self._enabled = False

    def zero_here(self) -> None:
        """Declare the present position to be the axis's zero.

        Part of homing. The calibration's ``zero_offset_counts`` exists for the case where
        the mechanical zero cannot be commanded away; when it can, this is cleaner.
        """
        self._send(*self.codec.zero(self.config.can_id))
        self._position_counts = 0.0

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    # -- feedback --------------------------------------------------------------

    def update(self, t: float, frames: list[tuple[float, int, bytes]]) -> None:
        """Fold this tick's frames into the axis's view of itself.

        Every frame is offered to the codec, which returns ``None`` for anything that is
        not its own feedback. That is what lets several motors and both codecs share one
        bus without an ID-routing table.
        """
        self._t = t
        for stamp, can_id, data in frames:
            parsed = self.codec.parse(can_id, data)
            if parsed is None or int(parsed.get("node_id", -1)) != self.config.can_id:
                continue
            self._position_counts = parsed["position"]
            self._velocity_counts_s = parsed.get("velocity", 0.0)
            self._current = parsed.get("current", 0.0)
            self._stamp = stamp

    @property
    def position_mm(self) -> float:
        """Position in mm. Zero until the first feedback frame arrives."""
        if self._position_counts is None:
            return 0.0
        return self.calibration.to_mm(self._position_counts)

    @property
    def velocity_mm_s(self) -> float:
        return self.calibration.rate_to_mm_s(self._velocity_counts_s)

    @property
    def current(self) -> float:
        """Motor current, in the codec's units. A proxy for insertion force."""
        return self._current

    @property
    def has_feedback(self) -> bool:
        return self._position_counts is not None

    @property
    def is_stale(self) -> bool:
        if self._stamp is None:
            return True
        return (self._t - self._stamp) > self.config.stale_after_s

    @property
    def age(self) -> float | None:
        return None if self._stamp is None else self._t - self._stamp

    # -- commands --------------------------------------------------------------

    def hold(self) -> None:
        """Re-issue the standing position command.

        *Not* "servo to wherever you are now". Re-reading the encoder every tick makes
        ``hold`` actively harmful in two ways: a commanded move gets cancelled partway,
        because the next tick's hold re-targets the position the axis has only reached so
        far; and a backdrivable axis can be pushed anywhere at all, since each tick
        ratchets the target to wherever it was just shoved. Both were observed. Latching
        the last commanded target fixes both, and makes "command a move, then hold it"
        mean what it reads as.
        """
        if self._hold_target_mm is None:
            self._hold_target_mm = self.position_mm
        self.move_to(self._hold_target_mm, check_limits=False, latch=False)

    def move_to(
        self,
        mm: float,
        *,
        v_max_mm_s: float | None = None,
        check_limits: bool = True,
        latch: bool = True,
    ) -> None:
        """Command an absolute position in mm.

        Soft limits are enforced here rather than deeper down, because this is the last
        place the request is still in units a human reasoned about.

        ``latch`` records this as the target :meth:`hold` will keep re-issuing; it is only
        false when ``hold`` itself is the caller.
        """
        if latch:
            self._hold_target_mm = mm
        if check_limits and not self.calibration.in_range(mm):
            self.limit_violations += 1
            lo, hi = self.calibration.travel_mm
            raise AxisLimitError(
                f"axis '{self.name}': commanded {mm:.3f} mm, outside soft limits "
                f"[{lo:.3f}, {hi:.3f}] mm"
            )
        mm = self.calibration.clamp(mm)
        v = min(v_max_mm_s or self.calibration.v_max_mm_s, self.calibration.v_max_mm_s)
        gains = self.config.gains
        self._send(
            *self.codec.command(
                self.config.can_id,
                position=self.calibration.to_counts(mm),
                velocity=abs(self.calibration.rate_to_counts_s(v)),
                kp=gains.get("kp", 0.0),
                kd=gains.get("kd", 0.0),
                torque=0.0,
            ),
            described={"kind": "move_to", "mm": mm, "v_max_mm_s": v},
        )

    def float_free(self) -> None:
        """Zero stiffness — let the axis be moved by whatever it is touching.

        What ADVANCE relies on: between increments the needle rides with the tissue
        rather than holding a position through the breathing cycle.
        """
        if not self.codec.supports_float:
            raise AxisLimitError(
                f"axis '{self.name}': codec '{self.codec.name}' cannot express zero-stiffness "
                "backdrive, which the ADVANCE state requires. Flash this motor for MIT mode, "
                "or do not run ADVANCE on it."
            )
        # Clear the latch: after floating, the axis is wherever the tissue left it, and
        # the next `hold` should keep it *there* rather than snapping back to a target
        # from before the float. ADVANCE depends on exactly this.
        self._hold_target_mm = None
        gains = self.config.float_gains
        self._send(
            *self.codec.command(
                self.config.can_id,
                position=None,
                velocity=0.0,
                kp=0.0,
                kd=gains.get("kd", 0.0),
                torque=0.0,
            ),
            described={"kind": "float", "kd": gains.get("kd", 0.0)},
        )

    def stop(self) -> None:
        """Command zero motion without disabling. Used on fault."""
        self._send(
            *self.codec.command(self.config.can_id, position=None, velocity=0.0, kp=0.0, kd=0.0),
            described={"kind": "stop"},
        )

    # -- plumbing --------------------------------------------------------------

    def _send(
        self,
        can_id: int,
        data: bytes,
        extended: bool,
        described: dict[str, Any] | None = None,
    ) -> None:
        self.last_command = described
        if self.dry_run:
            self.commands_suppressed += 1
            return
        self.bus.send(can_id, data, extended=extended)
        self.commands_sent += 1

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "codec": self.codec.name,
            "can_id": self.config.can_id,
            "position_mm": self.position_mm,
            "velocity_mm_s": self.velocity_mm_s,
            "enabled": self._enabled,
            "stale": self.is_stale,
            "commands_sent": self.commands_sent,
            "commands_suppressed": self.commands_suppressed,
            "limit_violations": self.limit_violations,
        }

    def __repr__(self) -> str:
        return f"CANAxis({self.name!r}, id={self.config.can_id}, at={self.position_mm:.2f}mm)"
