"""CubeMars AK-series MIT-mode protocol.

One 8-byte command frame carries position, velocity, stiffness, damping and feed-forward
torque together, which makes it an impedance interface rather than a position interface.
That matters here for one specific reason: **commanding ``kp=0, kd=small, tau=0`` gives
genuine zero-stiffness backdrive**, and ADVANCE is built entirely on letting the needle
float in tissue between increments. Servo mode has no clean equivalent, which is why the
recommendation is to run the needle motor in MIT mode. See the ``rig.motors.needle.codec``
entry in :mod:`ct.unknowns`.

Wire format
-----------

Command — standard 11-bit ID equal to the motor's node ID, 8 bytes, big-endian
bit-packed::

    bits    field    range (model-specific)
    16      p_des    [p_min, p_max]   rad
    12      v_des    [v_min, v_max]   rad/s
    12      kp       [0, kp_max]
    12      kd       [0, kd_max]
    12      t_ff     [t_min, t_max]   N·m

Reply — ID 0, 6 bytes: ``[node_id, p(16), v(12), i(12)]``.

Special frames, all ``FF FF FF FF FF FF FF xx``: ``FC`` enter motor mode, ``FD`` exit,
``FE`` set the present position as zero.

Units and ranges
----------------

**Position is in radians of the output shaft**, so for this codec the "counts" in
``geometry.counts_per_mm`` are radians per millimetre. The servo-mode codec reports
degrees. Calibration numbers are therefore *not* interchangeable between the two codecs —
switching one motor's codec means recalibrating that axis.

The ``*_max`` ranges are model-specific (AK80-9 differs from AK70-10 differs from
AK80-64) and the defaults here are AK80-9's. A mismatch does not error; it silently
scales every command, which is the worst failure mode available. Set them from the
datasheet for the motor actually fitted.
"""

from __future__ import annotations

from typing import Any

from ct.registry import register_codec

#: Enter closed-loop control.
ENTER_MOTOR_MODE = bytes([0xFF] * 7 + [0xFC])
#: Leave closed-loop control; the motor goes limp.
EXIT_MOTOR_MODE = bytes([0xFF] * 7 + [0xFD])
#: Declare the present position to be zero.
SET_ZERO_POSITION = bytes([0xFF] * 7 + [0xFE])


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """Quantise ``x`` into ``bits`` unsigned bits spanning ``[x_min, x_max]``."""
    span = x_max - x_min
    x = min(max(x, x_min), x_max)
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(value: int, x_min: float, x_max: float, bits: int) -> float:
    """Inverse of :func:`float_to_uint`."""
    span = x_max - x_min
    return value * span / ((1 << bits) - 1) + x_min


@register_codec("cubemars_mit")
class CubeMarsMIT:
    """MIT-mode codec. Defaults are AK80-9; override per the fitted motor's datasheet."""

    name = "cubemars_mit"

    def __init__(
        self,
        p_min: float = -12.5,
        p_max: float = 12.5,
        v_min: float = -50.0,
        v_max: float = 50.0,
        kp_min: float = 0.0,
        kp_max: float = 500.0,
        kd_min: float = 0.0,
        kd_max: float = 5.0,
        t_min: float = -18.0,
        t_max: float = 18.0,
    ) -> None:
        self.p_min, self.p_max = float(p_min), float(p_max)
        self.v_min, self.v_max = float(v_min), float(v_max)
        self.kp_min, self.kp_max = float(kp_min), float(kp_max)
        self.kd_min, self.kd_max = float(kd_min), float(kd_max)
        self.t_min, self.t_max = float(t_min), float(t_max)
        for lo, hi, what in (
            (self.p_min, self.p_max, "position"),
            (self.v_min, self.v_max, "velocity"),
            (self.kp_min, self.kp_max, "kp"),
            (self.kd_min, self.kd_max, "kd"),
            (self.t_min, self.t_max, "torque"),
        ):
            if lo >= hi:
                raise ValueError(f"{what} range must have min < max, got ({lo}, {hi})")

    # -- mode frames -----------------------------------------------------------

    def enable(self, can_id: int) -> tuple[int, bytes, bool]:
        return (can_id, ENTER_MOTOR_MODE, False)

    def disable(self, can_id: int) -> tuple[int, bytes, bool]:
        return (can_id, EXIT_MOTOR_MODE, False)

    def zero(self, can_id: int) -> tuple[int, bytes, bool]:
        return (can_id, SET_ZERO_POSITION, False)

    # -- command ---------------------------------------------------------------

    def command(
        self,
        can_id: int,
        *,
        position: float | None = None,
        velocity: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque: float = 0.0,
    ) -> tuple[int, bytes, bool]:
        """Pack one impedance command. ``position`` is radians of output shaft.

        ``position=None`` means "no position target", which is only meaningful with
        ``kp=0``; the packed value is then ignored by the motor. Passing ``None`` with a
        non-zero ``kp`` is a bug worth catching here rather than discovering as motion.
        """
        if position is None:
            if kp != 0.0:
                raise ValueError(
                    f"position=None with kp={kp}: the motor would servo to an arbitrary "
                    "target. Pass a position, or set kp=0 for a float command."
                )
            position = 0.0

        p_int = float_to_uint(position, self.p_min, self.p_max, 16)
        v_int = float_to_uint(velocity, self.v_min, self.v_max, 12)
        kp_int = float_to_uint(kp, self.kp_min, self.kp_max, 12)
        kd_int = float_to_uint(kd, self.kd_min, self.kd_max, 12)
        t_int = float_to_uint(torque, self.t_min, self.t_max, 12)

        data = bytes(
            (
                (p_int >> 8) & 0xFF,
                p_int & 0xFF,
                (v_int >> 4) & 0xFF,
                ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F),
                kp_int & 0xFF,
                (kd_int >> 4) & 0xFF,
                ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F),
                t_int & 0xFF,
            )
        )
        return (can_id, data, False)

    # -- feedback --------------------------------------------------------------

    def parse(self, can_id: int, data: bytes) -> dict[str, float] | None:
        """Decode a reply frame.

        MIT-mode replies all arrive on CAN ID 0 with the node ID in the first payload
        byte, so the ID alone does not identify the sender — the caller has to match on
        ``node_id`` from the returned dict. Returns ``None`` for anything that is not a
        well-formed reply, so several codecs can share a bus.
        """
        if can_id != 0 or len(data) < 6:
            return None
        node_id = data[0]
        p_int = (data[1] << 8) | data[2]
        v_int = (data[3] << 4) | (data[4] >> 4)
        i_int = ((data[4] & 0x0F) << 8) | data[5]
        return {
            "node_id": float(node_id),
            "position": uint_to_float(p_int, self.p_min, self.p_max, 16),
            "velocity": uint_to_float(v_int, self.v_min, self.v_max, 12),
            "current": uint_to_float(i_int, self.t_min, self.t_max, 12),
        }

    def parse_command(self, can_id: int, data: bytes) -> dict[str, Any] | None:
        """Decode a command frame — the inverse of :meth:`command`.

        The simulated motors use this to obey real frames, which is what makes simulation
        exercise the packing rather than bypass it. A round-trip test over this pair is
        the cheapest guard there is against a bit-shift error that would otherwise show up
        as unexplained motion on the bench.
        """
        if len(data) != 8:
            return None
        if data[:7] == b"\xff" * 7:
            mode = {0xFC: "enable", 0xFD: "disable", 0xFE: "zero"}.get(data[7])
            return None if mode is None else {"mode": mode, "can_id": can_id}

        p_int = (data[0] << 8) | data[1]
        v_int = (data[2] << 4) | (data[3] >> 4)
        kp_int = ((data[3] & 0x0F) << 8) | data[4]
        kd_int = (data[5] << 4) | (data[6] >> 4)
        t_int = ((data[6] & 0x0F) << 8) | data[7]
        return {
            "mode": "command",
            "can_id": can_id,
            "position": uint_to_float(p_int, self.p_min, self.p_max, 16),
            "velocity": uint_to_float(v_int, self.v_min, self.v_max, 12),
            "kp": uint_to_float(kp_int, self.kp_min, self.kp_max, 12),
            "kd": uint_to_float(kd_int, self.kd_min, self.kd_max, 12),
            "torque": uint_to_float(t_int, self.t_min, self.t_max, 12),
        }

    def encode_reply(
        self, node_id: int, position: float, velocity: float, current: float
    ) -> tuple[int, bytes, bool]:
        """Build a reply frame. Used by the simulated motors to answer realistically.

        Living next to :meth:`parse` rather than in the plant is deliberate: encode and
        decode of one wire format belong together, and a round-trip test over this pair
        is what proves the packing is right.
        """
        p_int = float_to_uint(position, self.p_min, self.p_max, 16)
        v_int = float_to_uint(velocity, self.v_min, self.v_max, 12)
        i_int = float_to_uint(current, self.t_min, self.t_max, 12)
        data = bytes(
            (
                node_id & 0xFF,
                (p_int >> 8) & 0xFF,
                p_int & 0xFF,
                (v_int >> 4) & 0xFF,
                ((v_int & 0x0F) << 4) | ((i_int >> 8) & 0x0F),
                i_int & 0xFF,
            )
        )
        return (0, data, False)

    @property
    def supports_float(self) -> bool:
        return True

    @property
    def position_range(self) -> tuple[float, float]:
        """Commandable position span, in radians of output shaft.

        Worth checking against the axis's travel: the AK default of ±12.5 rad is only
        about two turns, and a command past the end saturates silently rather than
        erroring. See :attr:`~ct.hw.config.MotorConfig.codec_params`.
        """
        return (self.p_min, self.p_max)

    def __repr__(self) -> str:
        return f"CubeMarsMIT(p=±{self.p_max}, v=±{self.v_max}, t=±{self.t_max})"
