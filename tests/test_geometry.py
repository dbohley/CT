"""Frames and unit conversion — the one place raw counts become millimetres.

Everything the controller believes about where things are passes through this module, so
a sign error here is a needle in the wrong place with no other symptom.
"""

from __future__ import annotations

import pytest

from ct.geometry import AxisCalibration, RigGeometry, RigState


@pytest.fixture
def calibration():
    return AxisCalibration(counts_per_mm=2.5, direction=1, zero_offset_counts=10.0,
                           travel_mm=(0.0, 100.0))


@pytest.fixture
def geometry():
    return RigGeometry(
        base=AxisCalibration(counts_per_mm=2.0, travel_mm=(0.0, 200.0)),
        needle=AxisCalibration(counts_per_mm=0.5, travel_mm=(0.0, 80.0)),
        tof_offset_mm=-10.0,
        tactile_offset_mm=0.0,
        needle_tip_offset_mm=-20.0,
        tactile_counts_to_mm=0.01,
        tactile_contact_counts=50.0,
        tactile_saturation_mm=20.0,
        tof_counts_to_mm=1.0,
    )


@pytest.mark.parametrize("mm", [0.0, 1.0, 37.5, -12.25, 100.0])
def test_counts_and_millimetres_round_trip(calibration, mm):
    assert calibration.to_mm(calibration.to_counts(mm)) == pytest.approx(mm)


@pytest.mark.parametrize("direction", [1, -1])
def test_direction_flips_the_sense_of_travel_but_not_the_round_trip(direction):
    cal = AxisCalibration(counts_per_mm=2.0, direction=direction, zero_offset_counts=5.0)
    assert cal.to_mm(cal.to_counts(20.0)) == pytest.approx(20.0)
    assert (cal.to_counts(20.0) > cal.zero_offset_counts) is (direction == 1)


def test_a_rate_has_no_datum(calibration):
    """Velocities convert without the zero offset. Applying it would be a real bug."""
    assert calibration.rate_to_mm_s(calibration.rate_to_counts_s(4.0)) == pytest.approx(4.0)
    assert calibration.rate_to_counts_s(0.0) == 0.0


def test_travel_limits_are_rejected_not_silently_clamped(calibration):
    assert calibration.in_range(50.0)
    assert not calibration.in_range(150.0)
    assert calibration.clamp(150.0) == pytest.approx(100.0)


def test_axis_calibration_rejects_impossible_configurations():
    with pytest.raises(ValueError, match="counts_per_mm"):
        AxisCalibration(counts_per_mm=0.0)
    with pytest.raises(ValueError, match="direction"):
        AxisCalibration(counts_per_mm=1.0, direction=0)
    with pytest.raises(ValueError, match="travel_mm"):
        AxisCalibration(counts_per_mm=1.0, travel_mm=(10.0, 5.0))


# -- frames ------------------------------------------------------------------


def test_features_ride_on_the_base(geometry):
    assert geometry.tactile_face_x(100.0) == pytest.approx(100.0)
    assert geometry.tof_face_x(100.0) == pytest.approx(90.0)
    assert geometry.needle_tip_x(100.0, 0.0) == pytest.approx(80.0)
    assert geometry.needle_tip_x(100.0, 25.0) == pytest.approx(105.0)


def test_the_number_everyone_asks_for_first(geometry):
    """How far the retracted needle tip sits behind the tactile face.

    Negative is the safe sign: the tip is tucked back, so seating the sensor against the
    phantom cannot drive the needle into it.
    """
    assert geometry.needle_tip_behind_tactile_mm == pytest.approx(-20.0)
    assert geometry.tof_behind_tactile_mm == pytest.approx(-10.0)


def test_skin_sits_behind_the_contact_face_by_the_deflection(geometry):
    """The sign that decides whether the needle aims at the skin or through it."""
    counts = 300.0  # 3 mm of deflection at 0.01 mm/count
    assert geometry.tactile_deflection_mm(counts) == pytest.approx(3.0)
    assert geometry.skin_x(base_mm=100.0, tactile_counts=counts) == pytest.approx(97.0)


def test_contact_threshold_is_a_threshold_not_an_offset(geometry):
    """Deflection is a pure scale; the contact level only decides *whether* we are touching."""
    assert not geometry.in_contact(10.0)
    assert geometry.in_contact(60.0)
    assert geometry.tactile_deflection_mm(100.0) == pytest.approx(1.0)


def test_needle_command_and_tip_position_are_inverses(geometry):
    needle_mm = geometry.needle_mm_for_tip_at(tip_x=105.0, base_mm=100.0)
    assert geometry.needle_tip_x(100.0, needle_mm) == pytest.approx(105.0)


def test_insertion_depth_is_signed_from_the_skin(geometry):
    # Tip at 105, skin at 100 -> 5 mm in.
    assert geometry.insertion_depth_mm(100.0, 25.0, skin_x=100.0) == pytest.approx(5.0)
    # Tip short of the skin reads negative, which is what "still clear" should look like.
    assert geometry.insertion_depth_mm(100.0, 10.0, skin_x=100.0) == pytest.approx(-10.0)


# -- clipping ----------------------------------------------------------------


def test_clipping_is_detected_at_both_ends(geometry):
    """The condition APPROACH's seating step searches for.

    Pinned at zero means the sensor loses the skin at end-exhale and the trough is cut
    off; pinned at saturation means it is seated too deep and the peak is cut off. Only
    between the two is the whole breath visible.
    """
    assert geometry.is_tactile_clipped(0.0)
    assert geometry.is_tactile_clipped(geometry.tactile_saturation_mm)
    assert not geometry.is_tactile_clipped(10.0)


def test_tof_out_of_range_readings_are_rejected_not_clamped(geometry):
    assert geometry.tof_in_range(500.0)
    assert not geometry.tof_in_range(5000.0)


# -- rig state ---------------------------------------------------------------


def test_rig_state_refuses_to_invent_a_skin_position():
    """States that need the skin get a clear error, not a silent zero."""
    state = RigState(t=1.0, stale=("tactile", "tof"))
    with pytest.raises(RuntimeError, match="no skin position"):
        state.require_skin()
    assert RigState(t=1.0, skin_x=42.0).require_skin() == pytest.approx(42.0)


def test_geometry_from_dict_builds_both_axes():
    geometry = RigGeometry.from_dict(
        {
            "base": {"counts_per_mm": 2.0, "travel_mm": [0.0, 200.0]},
            "needle": {"counts_per_mm": 0.5, "travel_mm": [0.0, 80.0]},
            "tactile_offset_mm": 1.5,
        }
    )
    assert geometry.base.travel_mm == (0.0, 200.0)
    assert geometry.tactile_offset_mm == pytest.approx(1.5)

    with pytest.raises(ValueError, match="needle"):
        RigGeometry.from_dict({"base": {"counts_per_mm": 2.0}})
