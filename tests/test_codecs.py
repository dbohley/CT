"""CubeMars wire formats.

A bit-packing error here does not raise — it produces smooth, plausible, wrong motion,
which is the most expensive kind of bug to find on a bench with a needle attached. So the
codecs are pinned two ways: golden byte vectors for the frames that have documented
values, and encode/decode round trips for everything else.

The round trips are not busywork. The simulated motors obey real frames through
:meth:`parse_command`, so these pairs are exactly what makes simulation exercise the
packing rather than bypass it.
"""

from __future__ import annotations

import pytest

from ct.hw.motors.cubemars_mit import (
    ENTER_MOTOR_MODE,
    EXIT_MOTOR_MODE,
    MIT_FAULT_CODES,
    SET_ZERO_POSITION,
    CubeMarsMIT,
    float_to_uint,
    uint_to_float,
)
from ct.hw.motors.cubemars_servo import CMD_SET_POS, CMD_STATUS_1, CubeMarsServo


@pytest.fixture
def mit():
    return CubeMarsMIT()


@pytest.fixture
def servo():
    return CubeMarsServo()


# -- quantisation -------------------------------------------------------------


@pytest.mark.parametrize("bits", [12, 16])
@pytest.mark.parametrize("value", [-12.5, -3.0, 0.0, 4.25, 12.5])
def test_quantisation_round_trips_within_one_step(bits, value):
    lo, hi = -12.5, 12.5
    step = (hi - lo) / ((1 << bits) - 1)
    recovered = uint_to_float(float_to_uint(value, lo, hi, bits), lo, hi, bits)
    assert recovered == pytest.approx(value, abs=step)


def test_out_of_range_values_saturate_rather_than_wrap():
    """Saturating is bad; wrapping would be catastrophic — a full-scale reversal."""
    assert float_to_uint(1e6, -12.5, 12.5, 16) == (1 << 16) - 1
    assert float_to_uint(-1e6, -12.5, 12.5, 16) == 0


# -- MIT mode -----------------------------------------------------------------


@pytest.mark.parametrize(
    "method,expected",
    [("enable", ENTER_MOTOR_MODE), ("disable", EXIT_MOTOR_MODE), ("zero", SET_ZERO_POSITION)],
)
def test_mit_mode_frames_are_the_documented_bytes(mit, method, expected):
    can_id, data, extended = getattr(mit, method)(3)
    assert (can_id, data, extended) == (3, expected, False)
    assert data[:7] == b"\xff" * 7


def test_mit_command_round_trips(mit):
    can_id, data, extended = mit.command(
        2, position=1.5, velocity=-2.0, kp=50.0, kd=1.25, torque=0.75
    )
    assert extended is False and len(data) == 8
    parsed = mit.parse_command(can_id, data)
    assert parsed["mode"] == "command"
    assert parsed["position"] == pytest.approx(1.5, abs=1e-3)
    assert parsed["velocity"] == pytest.approx(-2.0, abs=0.05)
    assert parsed["kp"] == pytest.approx(50.0, abs=0.5)
    assert parsed["kd"] == pytest.approx(1.25, abs=0.01)
    assert parsed["torque"] == pytest.approx(0.75, abs=0.02)


def test_mit_reply_round_trips(mit):
    can_id, data, _ = mit.encode_reply(7, position=-3.25, velocity=1.5, current=2.0)
    assert can_id == 0, "MIT replies all arrive on ID 0 with the node id in the payload"
    parsed = mit.parse(can_id, data)
    assert int(parsed["node_id"]) == 7
    assert parsed["error"] == 0
    assert parsed["position"] == pytest.approx(-3.25, abs=1e-3)
    assert parsed["velocity"] == pytest.approx(1.5, abs=0.05)


def test_mit_reply_splits_fault_code_from_node_id(mit):
    """Found on the bench: a node-2 reply reporting undervoltage decoded as node "146"
    until this split existed, because the first byte packs id and fault into one byte
    (id | err << 4) rather than being a bare node id."""
    can_id, data, _ = mit.encode_reply(2, position=0.0, velocity=0.0, current=0.0, error=0x9)
    parsed = mit.parse(can_id, data)
    assert int(parsed["node_id"]) == 2
    assert int(parsed["error"]) == 0x9
    assert MIT_FAULT_CODES[int(parsed["error"])] == "undervoltage"


def test_mit_float_is_expressible(mit):
    """``kp=0, kd=small, tau=0``. ADVANCE is built entirely on this frame."""
    assert mit.supports_float
    can_id, data, _ = mit.command(1, position=None, kp=0.0, kd=0.1, torque=0.0)
    parsed = mit.parse_command(can_id, data)
    assert parsed["kp"] == pytest.approx(0.0)
    assert parsed["kd"] == pytest.approx(0.1, abs=0.01)
    assert parsed["torque"] == pytest.approx(0.0, abs=0.01)


def test_mit_refuses_a_position_free_command_with_stiffness(mit):
    """Without a position target, non-zero kp would servo to whatever happened to be packed."""
    with pytest.raises(ValueError, match="position=None"):
        mit.command(1, position=None, kp=10.0)


def test_mit_ignores_frames_that_are_not_its_replies(mit):
    assert mit.parse(0x123, b"\x00" * 6) is None
    assert mit.parse(0, b"\x00" * 3) is None


def test_mit_position_range_is_only_about_two_turns(mit):
    """The constraint that pushed the base onto servo mode.

    +/-12.5 rad is roughly two output-shaft turns; a long linear axis will not fit.
    """
    lo, hi = mit.position_range
    assert (lo, hi) == (-12.5, 12.5)


# -- servo mode ---------------------------------------------------------------


def test_servo_uses_extended_ids_with_the_command_in_the_high_byte(servo):
    can_id, data, extended = servo.command(5, position=90.0)
    assert extended is True
    assert can_id & 0xFF == 5
    assert can_id >> 8 == CMD_SET_POS
    assert len(data) == 4


def test_servo_command_round_trips(servo):
    can_id, data, _ = servo.command(5, position=123.456)
    parsed = servo.parse_command(can_id, data)
    assert parsed["can_id"] == 5
    assert parsed["position"] == pytest.approx(123.456, abs=1e-3)


def test_servo_position_with_speed_limit_round_trips(servo):
    can_id, data, _ = servo.command(5, position=45.0, velocity=200.0)
    assert len(data) == 8
    parsed = servo.parse_command(can_id, data)
    assert parsed["position"] == pytest.approx(45.0, abs=1e-3)
    assert parsed["velocity"] == pytest.approx(200.0, abs=10.0)


def test_servo_status_frame_round_trips(servo):
    can_id, data, _ = servo.encode_reply(4, position=36.5, velocity=200.0, current=1.5)
    assert can_id >> 8 == CMD_STATUS_1
    parsed = servo.parse(can_id, data)
    assert int(parsed["node_id"]) == 4
    assert parsed["position"] == pytest.approx(36.5, abs=0.1)


def test_servo_cannot_float(servo):
    """The finding that decides which firmware the needle motor must run.

    Servo mode has duty, current, speed and position commands, and none of them expresses
    zero stiffness. ADVANCE needs one that does.
    """
    assert not servo.supports_float


def test_servo_has_room_for_a_long_axis(servo):
    lo, hi = servo.position_range
    assert hi > 1e5, "an int32 degrees field should cover any realistic linear travel"


# -- coexistence --------------------------------------------------------------


def test_the_two_codecs_do_not_claim_each_others_frames(mit, servo):
    """Both share the sensing bus, so each must reject the other's traffic.

    Without this, a servo-mode status frame could be decoded as an MIT reply and quietly
    move an axis's believed position.
    """
    mit_reply_id, mit_reply, _ = mit.encode_reply(1, 1.0, 0.0, 0.0)
    servo_status_id, servo_status, _ = servo.encode_reply(1, 10.0, 0.0, 0.0)
    servo_cmd_id, servo_cmd, _ = servo.command(1, position=10.0, velocity=100.0)

    assert servo.parse(mit_reply_id, mit_reply) is None
    assert mit.parse(servo_status_id, servo_status) is None
    # An 8-byte servo command must not be mistaken for an MIT command on node 1.
    parsed = mit.parse_command(servo_cmd_id, servo_cmd)
    assert parsed is None or parsed["can_id"] != 1
