"""CubeMars AK-series servo-mode protocol (the VESC-derived one).

Provided so a motor that is already flashed for servo mode can be driven without
reflashing, and so the base and phantom axes — which only need point-to-point moves —
have a working option either way.

**It cannot float.** Servo mode offers duty, current, speed and position commands, none
of which expresses "zero stiffness, ride with whatever you are touching". ADVANCE depends
on that, so :attr:`supports_float` is ``False`` here and an axis using this codec raises
if asked. The closest approximation — commanding zero current — leaves the motor's own
loops running and is not the same thing. If the needle motor turns out to be flashed for
servo mode, that is a finding worth acting on rather than working around.

Wire format
-----------

Extended 29-bit IDs, with the command in the upper byte::

    arbitration_id = node_id | (command << 8)

    cmd 0  SET_DUTY          int32   duty * 100000
    cmd 1  SET_CURRENT       int32   amps * 1000
    cmd 2  SET_CURRENT_BRAKE int32   amps * 1000
    cmd 3  SET_RPM           int32   electrical RPM
    cmd 4  SET_POS           int32   degrees * 10000
    cmd 5  SET_ORIGIN        int8    0 temporary, 1 permanent, 2 restore default
    cmd 6  SET_POS_SPD       int32 degrees*10000, int16 speed/10, int16 accel/10

Status frame 1 arrives on ``node_id | (9 << 8)``: position ``int16`` in 0.1°, speed
``int16`` in 10 ERPM, current ``int16`` in 0.01 A, then temperature and error bytes.

Units
-----

**Position is in degrees**, where the MIT codec uses radians. ``geometry.counts_per_mm``
is therefore codec-specific: changing an axis's codec means recalibrating that axis. The
scalings above are from the CubeMars servo-mode manual and should be confirmed on the
bench before being trusted — a wrong scale factor here produces smooth, plausible, wrong
motion.
"""

from __future__ import annotations

import struct
from typing import Any

from ct.registry import register_codec

CMD_SET_DUTY = 0
CMD_SET_CURRENT = 1
CMD_SET_CURRENT_BRAKE = 2
CMD_SET_RPM = 3
CMD_SET_POS = 4
CMD_SET_ORIGIN = 5
CMD_SET_POS_SPD = 6
CMD_STATUS_1 = 9


def _extended_id(node_id: int, command: int) -> int:
    return (node_id & 0xFF) | (command << 8)


@register_codec("cubemars_servo")
class CubeMarsServo:
    """Servo-mode codec. Position in degrees; no float capability."""

    name = "cubemars_servo"

    def __init__(
        self,
        pos_scale: float = 10000.0,
        current_scale: float = 1000.0,
        status_pos_scale: float = 10.0,
        status_speed_scale: float = 10.0,
        status_current_scale: float = 100.0,
        max_current_a: float = 20.0,
    ) -> None:
        self.pos_scale = float(pos_scale)
        self.current_scale = float(current_scale)
        self.status_pos_scale = float(status_pos_scale)
        self.status_speed_scale = float(status_speed_scale)
        self.status_current_scale = float(status_current_scale)
        self.max_current_a = float(max_current_a)

    # -- mode frames -----------------------------------------------------------

    def enable(self, can_id: int) -> tuple[int, bytes, bool]:
        """Servo mode has no explicit enable; it acts on the first command.

        A zero-current frame is sent so that "enable" is still a real, observable action
        on the bus, which keeps bring-up debugging symmetrical with MIT mode.
        """
        return self._current(can_id, 0.0)

    def disable(self, can_id: int) -> tuple[int, bytes, bool]:
        """Zero current — the motor coasts."""
        return self._current(can_id, 0.0)

    def zero(self, can_id: int) -> tuple[int, bytes, bool]:
        """Set the present position as a temporary origin."""
        return (_extended_id(can_id, CMD_SET_ORIGIN), bytes([0]), True)

    def _current(self, can_id: int, amps: float) -> tuple[int, bytes, bool]:
        amps = min(max(amps, -self.max_current_a), self.max_current_a)
        payload = struct.pack(">i", int(round(amps * self.current_scale)))
        return (_extended_id(can_id, CMD_SET_CURRENT), payload, True)

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
        """Pack a command. ``position`` is degrees.

        ``kp`` and ``kd`` are accepted for interface compatibility and ignored: servo mode
        runs its own gains internally. That asymmetry with MIT mode is exactly why
        :attr:`supports_float` is ``False``.
        """
        if position is None:
            return self._current(can_id, torque)
        if velocity > 0:
            # Position-with-speed-limit: the profiled move the base wants.
            payload = struct.pack(
                ">ihh",
                int(round(position * self.pos_scale)),
                int(round(velocity / 10.0)),
                0,  # acceleration 0 means "use the configured default"
            )
            return (_extended_id(can_id, CMD_SET_POS_SPD), payload, True)
        payload = struct.pack(">i", int(round(position * self.pos_scale)))
        return (_extended_id(can_id, CMD_SET_POS), payload, True)

    # -- feedback --------------------------------------------------------------

    def parse(self, can_id: int, data: bytes) -> dict[str, float] | None:
        """Decode a status-1 frame; ``None`` for anything else."""
        if (can_id >> 8) != CMD_STATUS_1 or len(data) < 6:
            return None
        pos_raw, spd_raw, cur_raw = struct.unpack(">hhh", data[:6])
        out = {
            "node_id": float(can_id & 0xFF),
            "position": pos_raw / self.status_pos_scale,
            "velocity": spd_raw * self.status_speed_scale,
            "current": cur_raw / self.status_current_scale,
        }
        if len(data) >= 8:
            out["temperature"] = float(struct.unpack(">b", data[6:7])[0])
            out["error"] = float(data[7])
        return out

    def parse_command(self, can_id: int, data: bytes) -> dict[str, Any] | None:
        """Decode a command frame — the inverse of :meth:`command`."""
        command = can_id >> 8
        node_id = can_id & 0xFF
        if command == CMD_SET_POS and len(data) >= 4:
            (pos_raw,) = struct.unpack(">i", data[:4])
            return {
                "mode": "command",
                "can_id": node_id,
                "position": pos_raw / self.pos_scale,
                "velocity": 0.0,
                "kp": 1.0,  # servo mode's gains are internal; 1.0 marks "position-controlled"
                "kd": 0.0,
                "torque": 0.0,
            }
        if command == CMD_SET_POS_SPD and len(data) >= 8:
            pos_raw, spd_raw, _accel = struct.unpack(">ihh", data[:8])
            return {
                "mode": "command",
                "can_id": node_id,
                "position": pos_raw / self.pos_scale,
                "velocity": spd_raw * 10.0,
                "kp": 1.0,
                "kd": 0.0,
                "torque": 0.0,
            }
        if command in (CMD_SET_CURRENT, CMD_SET_CURRENT_BRAKE) and len(data) >= 4:
            (cur_raw,) = struct.unpack(">i", data[:4])
            return {
                "mode": "command",
                "can_id": node_id,
                "position": None,
                "velocity": 0.0,
                "kp": 0.0,
                "kd": 0.0,
                "torque": cur_raw / self.current_scale,
            }
        if command == CMD_SET_ORIGIN:
            return {"mode": "zero", "can_id": node_id}
        return None

    def encode_reply(
        self, node_id: int, position: float, velocity: float, current: float
    ) -> tuple[int, bytes, bool]:
        """Build a status-1 frame, for the simulated motors."""
        payload = struct.pack(
            ">hhhbB",
            int(round(position * self.status_pos_scale)),
            int(round(velocity / self.status_speed_scale)),
            int(round(current * self.status_current_scale)),
            25,  # temperature, °C
            0,  # error code
        )
        return (_extended_id(node_id, CMD_STATUS_1), payload, True)

    @property
    def supports_float(self) -> bool:
        return False

    @property
    def position_range(self) -> tuple[float, float]:
        """Commandable span in degrees, from the int32 payload.

        Effectively unbounded for any real axis — which is servo mode's one clear
        advantage over MIT mode here, and the reason a long-travel base is easier to drive
        this way even though the needle cannot be.
        """
        limit = (2**31 - 1) / self.pos_scale
        return (-limit, limit)

    def __repr__(self) -> str:
        return f"CubeMarsServo(pos_scale={self.pos_scale})"
