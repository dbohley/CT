"""The physical constants we do not know yet — as data, not as TODO comments.

Most of the numbers that make the rig work have not been measured. Rather than scatter
``# TODO: measure this`` through the controller, every one of them is an entry here,
carrying the dotted config key it plugs into, its units, how to measure it, who owns it,
and a placeholder that lets simulation run today.

Three things consume this list:

1. ``ct-unknowns`` prints it, grouped by owner. ``--format md`` writes ``docs/unknowns.md``
   — that file is what you hand the team.
2. ``ct-unknowns --check <config>`` resolves every key against a real config and reports
   which are still sitting at their placeholder.
3. :func:`gate_or_raise` refuses to start a *hardware* run when a placeholder still blocks
   a state that run intends to enter. Simulation is unaffected: placeholders are the point
   in simulation.

The rule that keeps this honest: **add an entry here rather than a bare TODO.** A number
that is missing from this list is a number nobody is going to measure.

``test_unknowns.py`` asserts every :attr:`Unknown.key` resolves against the shipped rig
config, so the list cannot quietly drift away from the schema it describes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Sequence

from ct.control.state import ProcedureState

#: Who is expected to produce the number. Not a job title — a place to look.
OWNERS: tuple[str, ...] = ("mech", "electrical", "sensing", "controls", "procedure", "clinical")


@dataclass(frozen=True)
class Unknown:
    """One number we need, and everything required to go get it."""

    key: str
    """Dotted config path where the value plugs in, e.g. ``rig.geometry.base.counts_per_mm``."""

    units: str
    what: str
    """One sentence, written for a teammate rather than a programmer."""

    why: str
    """What it feeds, and what goes wrong while it is a guess."""

    how_to_measure: str
    """The procedure that produces the number. Concrete enough to act on."""

    owner: str
    placeholder: Any
    """Value used until the real one arrives. Simulation runs on these."""

    blocks: tuple[ProcedureState, ...] = ()
    """States that must not run on hardware while this is still a placeholder."""

    def __post_init__(self) -> None:
        if self.owner not in OWNERS:
            raise ValueError(f"unknown owner '{self.owner}' for {self.key}; expected one of {OWNERS}")
        if not self.key or " " in self.key:
            raise ValueError(f"key must be a dotted config path, got {self.key!r}")

    def resolve(self, cfg: dict[str, Any]) -> Any:
        """Current value of this key in ``cfg``, or :data:`MISSING` if absent."""
        node: Any = cfg
        for part in self.key.split("."):
            if not isinstance(node, dict) or part not in node:
                return MISSING
            node = node[part]
        return node

    def is_placeholder(self, cfg: dict[str, Any]) -> bool:
        """True if this config has not yet been given a real measurement."""
        value = self.resolve(cfg)
        if value is MISSING:
            return True
        return _equalish(value, self.placeholder)


class _Missing:
    def __repr__(self) -> str:
        return "(absent)"

    def __bool__(self) -> bool:
        return False


#: Sentinel for "this key is not in the config at all", distinct from a placeholder value.
MISSING = _Missing()


def _equalish(a: Any, b: Any) -> bool:
    """Value equality that survives YAML round-trips (lists vs tuples, ints vs floats)."""
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_equalish(x, y) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return float(a) == float(b)
    return bool(a == b)


# ---------------------------------------------------------------------------
# The list.
# ---------------------------------------------------------------------------
#
# Ordered by owner so `ct-unknowns` reads as a set of work packages rather than an
# undifferentiated pile. Within an owner, roughly in the order they are needed.

UNKNOWNS: tuple[Unknown, ...] = (
    # -- mech ---------------------------------------------------------------
    Unknown(
        key="rig.geometry.base.counts_per_mm",
        units="motor units / mm",
        what="How much the base motor's reported position changes per millimetre the base carriage travels.",
        why="Every base command and every base reading passes through this. Wrong by a factor "
            "and the approach either crawls or slams into the phantom.",
        how_to_measure="Home the base, command a large open-loop move, measure the carriage travel "
                       "with calipers or a dial indicator, and divide the motor's reported delta by "
                       "the measured millimetres. Repeat in both directions to catch backlash.",
        owner="mech",
        placeholder=100.0,
        blocks=(ProcedureState.APPROACH, ProcedureState.RETRACT),
    ),
    Unknown(
        key="rig.geometry.needle.counts_per_mm",
        units="motor units / mm",
        what="Same, for the needle drive.",
        why="Sets insertion depth. This is the number that decides whether a commanded 10 mm "
            "insertion is 10 mm of needle in tissue.",
        how_to_measure="Same procedure as the base, with the needle out of tissue and a scale "
                       "behind the tip. Do it at the speeds insertion will actually use — a "
                       "lead screw under load is not the same as one turning free.",
        owner="mech",
        placeholder=200.0,
        blocks=(ProcedureState.INSERT, ProcedureState.ADVANCE, ProcedureState.WITHDRAW),
    ),
    Unknown(
        key="rig.geometry.needle_tip_offset_mm",
        units="mm",
        what="Distance from the base carriage origin to the needle tip when the needle axis reads zero, "
             "measured along +x (toward the phantom).",
        why="With rig.geometry.tactile_offset_mm this gives how far the retracted tip sits behind the "
            "tactile face — the number that decides whether seating the sensor is safe. Get it wrong in "
            "the optimistic direction and the needle touches the phantom during APPROACH.",
        how_to_measure="Retract the needle fully, home the base, and measure from the carriage datum "
                       "to the tip. Then measure to the tactile contact face the same way; the "
                       "difference is what matters and it is worth recording both.",
        owner="mech",
        placeholder=-20.0,
        blocks=(ProcedureState.APPROACH, ProcedureState.INSERT, ProcedureState.ADVANCE),
    ),
    Unknown(
        key="rig.geometry.tactile_offset_mm",
        units="mm",
        what="Distance from the base carriage origin to the tactile sensor's contact face, along +x.",
        why="Converts tactile deflection into an absolute skin position, which is what the needle aims at.",
        how_to_measure="Calipers from the carriage datum to the face, with the sensor mounted as it "
                       "will actually run.",
        owner="mech",
        placeholder=0.0,
        blocks=(ProcedureState.APPROACH, ProcedureState.INSERT, ProcedureState.ADVANCE),
    ),
    Unknown(
        key="rig.geometry.tof_offset_mm",
        units="mm",
        what="Distance from the base carriage origin to the ToF sensor's reference plane, along +x.",
        why="Coarse approach is closed on ToF. This offset is what turns a range reading into a "
            "distance-to-contact for the tactile face.",
        how_to_measure="Calipers to the sensor's stated reference plane — check the datasheet for "
                       "where that plane actually is; it is often the die, not the housing face.",
        owner="mech",
        placeholder=-10.0,
        blocks=(ProcedureState.APPROACH,),
    ),
    Unknown(
        key="rig.geometry.base.direction",
        units="+1 or -1",
        what="Whether increasing base motor position moves the carriage toward the phantom.",
        why="A sign error here drives the base away during approach, or into the phantom during retract.",
        how_to_measure="Command a small positive move with the phantom well clear and watch which way "
                       "it goes. Do this before anything else on the bus.",
        owner="mech",
        placeholder=1,
        blocks=(ProcedureState.APPROACH, ProcedureState.RETRACT),
    ),
    Unknown(
        key="rig.geometry.needle.direction",
        units="+1 or -1",
        what="Whether increasing needle motor position extends the needle toward the phantom.",
        why="As above, with a sharp object.",
        how_to_measure="Same, with the needle clear of everything.",
        owner="mech",
        placeholder=1,
        blocks=(ProcedureState.INSERT, ProcedureState.ADVANCE, ProcedureState.WITHDRAW),
    ),
    Unknown(
        key="rig.geometry.base.travel_mm",
        units="[mm, mm]",
        what="Soft limits on base travel: how far it can go before it hits something.",
        why="The last line of defence before mechanical damage. Commands outside it are refused.",
        how_to_measure="Jog to each hard stop by hand with power off, read the positions, and back "
                       "off a few millimetres at each end.",
        owner="mech",
        placeholder=[0.0, 200.0],
        blocks=(ProcedureState.APPROACH, ProcedureState.RETRACT),
    ),
    Unknown(
        key="rig.geometry.needle.travel_mm",
        units="[mm, mm]",
        what="Soft limits on needle extension.",
        why="Caps the deepest insertion the hardware will physically accept, independent of what "
            "the procedure config asks for.",
        how_to_measure="Jog the needle to each hard stop by hand with power off and read the "
                       "positions, then back off a few millimetres at each end. Do it with the "
                       "needle fitted -- the hub fouling the sensor mount is usually the real "
                       "limit, not the screw.",
        owner="mech",
        placeholder=[0.0, 80.0],
        blocks=(ProcedureState.INSERT, ProcedureState.ADVANCE),
    ),
    Unknown(
        key="rig.geometry.needle_free_length_mm",
        units="mm",
        what="Exposed needle length: how far the tip can advance before the hub fouls the sensor mount.",
        why="Bounds total insertion depth for real, whatever the procedure config says.",
        how_to_measure="Measure the needle from tip to hub shoulder with it mounted.",
        owner="mech",
        placeholder=100.0,
        blocks=(ProcedureState.ADVANCE,),
    ),

    # -- electrical ---------------------------------------------------------
    Unknown(
        key="rig.buses.sensing.channel",
        units="string",
        what="python-can channel for the sensing bus — the one carrying the ToF sensor, the tactile "
             "sensor, the base motor and the needle motor.",
        why="Nothing on the rig talks without it.",
        how_to_measure="Plug in the USB-CAN-FD RH02, then `python -m can.viewer -i <interface> -c <channel>` "
                       "until frames appear. On Linux the adapter usually enumerates as SocketCAN "
                       "(gs_usb/candleLight) and the channel is 'can0'; on Windows it is typically a "
                       "vendor DLL with a numeric channel. Settle this before building anything on it.",
        owner="electrical",
        placeholder="can0",
        blocks=(),  # a hardware run fails loudly on its own if this is wrong
    ),
    Unknown(
        key="rig.buses.sensing.interface",
        units="string",
        what="python-can interface backend name for the RH02 adapter.",
        why="Selects the driver. 'socketcan' on Linux, a vendor backend on Windows.",
        how_to_measure="`python -m can.detect_available_configs` with the adapter plugged in.",
        owner="electrical",
        placeholder="socketcan",
        blocks=(),
    ),
    Unknown(
        key="rig.buses.sensing.bitrate",
        units="bit/s",
        what="Arbitration bitrate on the sensing bus.",
        why="Must match what the motors and sensors are configured for or the bus stays silent. "
            "CubeMars AK motors ship at 1 Mbit/s classic CAN.",
        how_to_measure="Read it off the motor configuration in the CubeMars tool; confirm by getting "
                       "clean frames in can.viewer.",
        owner="electrical",
        placeholder=1000000,
        blocks=(),
    ),
    Unknown(
        key="rig.motors.needle.codec",
        units="cubemars_mit | cubemars_servo",
        what="Which firmware protocol the needle motor is running.",
        why="This one changes the procedure, not just the wire format. ADVANCE works by letting the "
            "needle float in tissue, and float (kp=0, kd=small, tau=0) is native to MIT mode with no "
            "clean equivalent in servo mode. Strong recommendation: run the needle motor in MIT mode.",
        how_to_measure="Check what is flashed, with whoever set the motors up. The two protocols are "
                       "trivially distinguishable on the bus: MIT mode uses standard 11-bit IDs equal "
                       "to the node ID, servo mode uses 29-bit extended IDs with the command in the "
                       "upper byte.",
        owner="electrical",
        placeholder="cubemars_mit",
        blocks=(ProcedureState.ADVANCE,),
    ),
    Unknown(
        key="rig.motors.base.can_id",
        units="int",
        what="CAN node ID of the base motor.",
        why="Addresses every command to it, and identifies its feedback frames.",
        how_to_measure="From the CubeMars configuration tool, or watch the bus while powering one "
                       "motor at a time.",
        owner="electrical",
        placeholder=1,
        blocks=(ProcedureState.APPROACH, ProcedureState.RETRACT),
    ),
    Unknown(
        key="rig.motors.needle.can_id",
        units="int",
        what="CAN node ID of the needle motor.",
        why="Addresses every needle command, and identifies its feedback frames. A collision "
            "with the base motor's ID would make each axis act on the other's commands.",
        how_to_measure="From the CubeMars configuration tool, or power one motor at a time and "
                       "watch which IDs appear on the bus. Confirm the two motors differ.",
        owner="electrical",
        placeholder=2,
        blocks=(ProcedureState.INSERT, ProcedureState.ADVANCE, ProcedureState.WITHDRAW),
    ),
    Unknown(
        key="rig.motors.needle.gear_ratio",
        units="dimensionless",
        what="Needle motor gearbox ratio (AK80-9 is 9, AK70-10 is 10, AK80-64 is 64).",
        why="Decides whether reported position is before or after the gearbox. Folded into "
            "counts_per_mm, but recorded separately so the calibration can be sanity-checked "
            "against the mechanics rather than just trusted.",
        how_to_measure="Motor part number and datasheet.",
        owner="electrical",
        placeholder=9.0,
        blocks=(),
    ),
    Unknown(
        key="rig.motors.needle.limits.tau_max",
        units="N·m",
        what="Torque ceiling the needle motor may be commanded to.",
        why="Bounds how hard the needle can push. On a needle this is a safety limit, not a "
            "performance setting.",
        how_to_measure="Motor datasheet for the ceiling; then decide a working value well under it "
                       "by pushing into a test block and finding what force is acceptable.",
        owner="electrical",
        placeholder=2.0,
        blocks=(ProcedureState.INSERT, ProcedureState.ADVANCE),
    ),
    Unknown(
        key="rig.motors.needle.float_gains",
        units="{kp, kd}",
        what="The MIT-mode gains that make the needle genuinely backdrivable — free to ride with "
             "tissue, with just enough damping not to oscillate.",
        why="ADVANCE is built entirely on this. kp must be 0; kd is the part that has to be found "
            "experimentally.",
        how_to_measure="With the needle in a phantom, command kp=0, tau=0 and sweep kd upward from 0. "
                       "Too low and it rattles; too high and it drags the tissue. Take the smallest "
                       "value that is visibly settled.",
        owner="controls",
        placeholder={"kp": 0.0, "kd": 0.1},
        blocks=(ProcedureState.ADVANCE,),
    ),
    Unknown(
        key="rig.sensors.tactile.can_id",
        units="int",
        what="CAN ID of the tactile sensor's data frame.",
        why="This is the frame the entire estimator consumes.",
        how_to_measure="Watch the bus with only the tactile sensor powered.",
        owner="electrical",
        placeholder=256,
        blocks=(ProcedureState.APPROACH, ProcedureState.ESTIMATE, ProcedureState.INSERT, ProcedureState.ADVANCE),
    ),
    Unknown(
        key="rig.sensors.tactile.layout",
        units="{offset, length, byteorder, signed, scale}",
        what="Where in the tactile frame's payload the reading sits, and how it is encoded.",
        why="Decodes the frame. A wrong byteorder produces plausible-looking garbage rather than "
            "an obvious error, which is the dangerous kind of wrong.",
        how_to_measure="Sensor datasheet, then confirm by pressing the sensor by hand and watching "
                       "the decoded value move monotonically.",
        owner="electrical",
        placeholder={"offset": 0, "length": 2, "byteorder": "little", "signed": False, "scale": 1.0},
        blocks=(ProcedureState.APPROACH, ProcedureState.ESTIMATE, ProcedureState.INSERT, ProcedureState.ADVANCE),
    ),
    Unknown(
        key="rig.sensors.tof.can_id",
        units="int",
        what="CAN ID of the time-of-flight sensor's data frame.",
        why="The coarse approach closes on it. Getting it wrong means the base never starts "
            "moving, because no valid range ever arrives.",
        how_to_measure="Watch the bus with only the ToF sensor powered, then confirm the decoded "
                       "value falls as you move a hand toward it.",
        owner="electrical",
        placeholder=257,
        blocks=(ProcedureState.APPROACH,),
    ),
    Unknown(
        key="rig.sensors.tof.layout",
        units="{offset, length, byteorder, signed, scale}",
        what="Payload layout of the ToF frame.",
        why="Decodes the range reading. A wrong byteorder yields plausible but wrong distances, "
            "so the coarse approach would drive confidently to the wrong place.",
        how_to_measure="Datasheet, confirmed against a tape measure at two known distances -- "
                       "check both, so a scale error cannot hide as an offset.",
        owner="electrical",
        placeholder={"offset": 0, "length": 2, "byteorder": "little", "signed": False, "scale": 1.0},
        blocks=(ProcedureState.APPROACH,),
    ),

    # -- sensing ------------------------------------------------------------
    Unknown(
        key="rig.geometry.tactile_counts_to_mm",
        units="mm / count",
        what="Millimetres of skin displacement per raw tactile count.",
        why="The most load-bearing number on this list. The estimator's amplitudes, the standoff, "
            "the firing gate's variance threshold and every insertion depth are all downstream of "
            "it. If it is wrong, everything downstream is wrong by the same factor and nothing "
            "looks obviously broken.",
        how_to_measure="Mount the sensor against a micrometer stage or a surface on a dial indicator. "
                       "Step known displacements across the working range, record raw counts, and fit "
                       "a line. Keep the residuals — if it is not linear, this needs to become a "
                       "curve rather than a scalar, and the config will have to grow to match.",
        owner="sensing",
        placeholder=0.01,
        blocks=(ProcedureState.APPROACH, ProcedureState.ESTIMATE, ProcedureState.INSERT, ProcedureState.ADVANCE),
    ),
    Unknown(
        key="rig.geometry.tactile_contact_counts",
        units="counts",
        what="Raw tactile level that means 'touching', as distinct from noise with nothing there.",
        why="Ends the contact sub-step of APPROACH. Too low and it declares contact on noise; too "
            "high and it presses hard into the phantom before noticing.",
        how_to_measure="Record the sensor's raw output in free air for a minute; take the mean plus "
                       "several standard deviations. Then confirm it triggers on a light touch.",
        owner="sensing",
        placeholder=50.0,
        blocks=(ProcedureState.APPROACH,),
    ),
    Unknown(
        key="rig.geometry.tactile_saturation_mm",
        units="mm",
        what="Deflection beyond which the tactile sensor stops responding.",
        why="Bounds how hard the sensor may be seated, and is why the seat sub-step watches for the "
            "measured amplitude to stop growing.",
        how_to_measure="Falls out of the counts_to_mm calibration above: the displacement where the "
                       "line goes flat.",
        owner="sensing",
        placeholder=20.0,
        blocks=(ProcedureState.APPROACH,),
    ),
    Unknown(
        key="rig.geometry.tof_counts_to_mm",
        units="mm / count",
        what="Millimetres of standoff per raw ToF count.",
        why="Scales the coarse approach. Less critical than the tactile scale — the approach is a "
            "closed loop that ends on tactile contact regardless — but a wrong scale makes it "
            "approach at the wrong speed.",
        how_to_measure="Two known distances on a tape measure, then a line through them. Check "
                       "linearity across the full range; ToF sensors are often nonlinear at the "
                       "near end.",
        owner="sensing",
        placeholder=1.0,
        blocks=(ProcedureState.APPROACH,),
    ),
    Unknown(
        key="rig.sensors.tactile.rate_hz",
        units="Hz",
        what="How fast the tactile sensor actually publishes.",
        why="This is the estimator's sample rate. It sets what breathing harmonics are observable "
            "at all, and the EKF's Q scaling assumes it.",
        how_to_measure="Timestamp arrivals on the bus for 30 seconds and take the median interval. "
                       "Record the jitter too — the EKF handles irregular dt, but the identifier's "
                       "FFT does not.",
        owner="sensing",
        placeholder=100.0,
        blocks=(ProcedureState.ESTIMATE,),
    ),

    # -- controls -----------------------------------------------------------
    Unknown(
        key="latency.tau_s",
        units="s",
        what="Sensor latency: from the skin actually being at a position to that value being "
             "available to the control loop.",
        why="First term of the forecast horizon h. Under-estimate it and every forecast is "
            "systematically early.",
        how_to_measure="Already automated: run `ct-phantom` and `ct-rig` together and use "
                       "`ct-compare`, which reports the cross-correlation lag between commanded "
                       "phantom motion and sensed motion. That lag is this number.",
        owner="controls",
        placeholder=0.02,
        blocks=(ProcedureState.INSERT, ProcedureState.ADVANCE),
    ),
    Unknown(
        key="latency.T_ins",
        units="s",
        what="How long the initial insertion drive actually takes, tip touching skin to target depth.",
        why="Last term of h, and usually the largest. The forecast has to look this far ahead "
            "because the skin keeps moving throughout the insertion.",
        how_to_measure="Command the insertion into a test block at the intended depth and speed and "
                       "time it from the telemetry log. Depends on both depth and needle velocity, so "
                       "re-measure if either changes.",
        owner="controls",
        placeholder=0.15,
        blocks=(ProcedureState.INSERT,),
    ),
    Unknown(
        key="servo.needle.plant",
        units="{K, wn, zeta}",
        what="Second-order model of the needle axis: DC gain, natural frequency, damping ratio.",
        why="The lead compensator is designed against this, and tau_cl(omega_r) is computed from the "
            "resulting closed loop. Until it is measured, tau_cl is a guess and h is wrong by "
            "however far off the guess is.",
        how_to_measure="Step-response identification on the needle axis, out of tissue: command a "
                       "small step, log commanded and measured position at full loop rate, and fit a "
                       "second-order model. A frequency sweep is better if the rig tolerates it.",
        owner="controls",
        placeholder={"K": 1.0, "wn": 60.0, "zeta": 0.7},
        blocks=(ProcedureState.INSERT,),
    ),
    Unknown(
        key="servo.needle.lead",
        units="{zero, pole, gain}",
        what="The lead compensator itself.",
        why="Determines how well the needle tracks the moving skin during INSERT, and therefore "
            "tau_cl. Note this is a design *output* — it follows from the plant above, and should "
            "be re-derived rather than hand-tuned once the plant is known.",
        how_to_measure="Design it against the identified plant for the phase margin you want at the "
                       "crossover you can afford, then confirm on hardware.",
        owner="controls",
        placeholder={"zero": 8.0, "pole": 80.0, "gain": 150.0},
        blocks=(ProcedureState.INSERT,),
    ),
    Unknown(
        key="procedure.loop_rate_hz",
        units="Hz",
        what="Control loop rate the host can actually sustain without missing deadlines.",
        why="Sets tau_c, and bounds how tightly the needle can track. Aspirational values here are "
            "worse than honest low ones.",
        how_to_measure="Run `ct-rig` on the target machine and read the deadline-miss count and p95 "
                       "tick time out of the telemetry summary. Raise the rate until misses appear, "
                       "then back off.",
        owner="controls",
        placeholder=200.0,
        blocks=(),
    ),

    # -- procedure ----------------------------------------------------------
    Unknown(
        key="procedure.approach.standoff_mm",
        units="mm",
        what="Gap between the needle tip and the skin at maximum inhale, at the end of APPROACH.",
        why="The clearance that keeps the needle off the phantom while the model is being "
            "identified. Max inhale is the binding case because that is when the skin is nearest.",
        how_to_measure="A judgement call rather than a measurement: small enough that the insertion "
                       "drive is short, large enough to absorb the tracking error. Start generous "
                       "and reduce once the estimator's real accuracy is known.",
        owner="procedure",
        placeholder=2.0,
        blocks=(ProcedureState.APPROACH,),
    ),
    Unknown(
        key="procedure.insert.initial_depth_mm",
        units="mm",
        what="How far the needle drives on the first insertion, past the skin surface.",
        why="Has to be deep enough that the needle is captured by tissue and can be left to float, "
            "and no deeper than necessary.",
        how_to_measure="Test insertions into the phantom; the depth at which the needle reliably "
                       "stays put when released to float.",
        owner="procedure",
        placeholder=10.0,
        blocks=(ProcedureState.INSERT,),
    ),
    Unknown(
        key="procedure.advance.total_depth_mm",
        units="mm",
        what="Final target depth past the skin surface.",
        why="Ends the procedure. Clinically driven, and bounded by needle_free_length_mm.",
        how_to_measure="From the target anatomy and the imaging that plans the insertion.",
        owner="clinical",
        placeholder=40.0,
        blocks=(ProcedureState.ADVANCE,),
    ),
    Unknown(
        key="procedure.advance.increment_mm",
        units="mm",
        what="How far the needle advances per breath during ADVANCE.",
        why="Trades total time against per-move tissue disturbance. Smaller increments mean more "
            "breaths but less displacement per move.",
        how_to_measure="Test insertions with tracking of the target: the largest increment that "
                       "does not visibly drag the target.",
        owner="procedure",
        placeholder=2.0,
        blocks=(ProcedureState.ADVANCE,),
    ),
    Unknown(
        key="procedure.advance.exhale_band_frac",
        units="fraction of breath excursion",
        what="How close to the bottom of the breathing waveform counts as 'at exhale' for firing. "
             "0.15 means the lowest 15% of the excursion.",
        why="The gate that implements 'only advance during exhale, because exhale is the more "
            "reproducible end of the cycle'. Too wide and moves happen mid-breath; too narrow and "
            "breaths get skipped waiting for a window that never opens.",
        how_to_measure="Start at 0.15 and widen if the log shows breaths being skipped. The "
                       "telemetry records every gate evaluation and why it failed.",
        owner="procedure",
        placeholder=0.15,
        blocks=(ProcedureState.ADVANCE,),
    ),
    Unknown(
        key="procedure.advance.max_forecast_std_mm",
        units="mm",
        what="Forecast standard deviation above which the gate refuses to fire.",
        why="This is what the EKF's covariance is *for* — the explicit uncertainty an LMS predictor "
            "cannot give you. It should be a fraction of the clinical tolerance, but the estimator's "
            "covariance also needs calibrating against realised error before the number means much.",
        how_to_measure="Two parts. Calibrate: run the forecast-variance check against realised error "
                       "on real traces and confirm the covariance is honest. Then set: some fraction "
                       "of the clinical tolerance epsilon.",
        owner="controls",
        placeholder=0.5,
        blocks=(ProcedureState.INSERT, ProcedureState.ADVANCE),
    ),

    # -- clinical -----------------------------------------------------------
    Unknown(
        key="procedure.clinical.tolerance_mm",
        units="mm",
        what="Clinical placement tolerance epsilon — how far off target the needle may finish.",
        why="The requirement everything else is sized against. Sets the forecast variance threshold "
            "and, ultimately, whether this approach is good enough at all.",
        how_to_measure="From the clinical collaborators, per target organ.",
        owner="clinical",
        placeholder=2.0,
        blocks=(),
    ),
    Unknown(
        key="procedure.clinical.target_amplitude_mm",
        units="mm",
        what="Expected peak-to-peak respiratory motion of the target, for the intended organ.",
        why="Sizes the whole problem: standoff, insertion depth, and how much the forecast has to buy.",
        how_to_measure="Literature for the organ, then confirm against the collected breathing "
                       "recordings once they are in hand.",
        owner="clinical",
        placeholder=20.0,
        blocks=(),
    ),
)


# ---------------------------------------------------------------------------
# Queries.
# ---------------------------------------------------------------------------


def by_owner(unknowns: Iterable[Unknown] | None = None) -> dict[str, list[Unknown]]:
    """Group by owner, in :data:`OWNERS` order, skipping owners with no entries."""
    items = list(unknowns if unknowns is not None else UNKNOWNS)
    grouped = {owner: [u for u in items if u.owner == owner] for owner in OWNERS}
    return {owner: entries for owner, entries in grouped.items() if entries}


def outstanding(cfg: dict[str, Any], unknowns: Iterable[Unknown] | None = None) -> list[Unknown]:
    """Entries still sitting at their placeholder in ``cfg``."""
    items = unknowns if unknowns is not None else UNKNOWNS
    return [u for u in items if u.is_placeholder(cfg)]


def blocking(
    cfg: dict[str, Any],
    states: Sequence[ProcedureState],
    unknowns: Iterable[Unknown] | None = None,
) -> list[Unknown]:
    """Outstanding entries that block any of ``states``.

    This is the set that makes a hardware run unsafe rather than merely uncalibrated.
    """
    wanted = set(states)
    return [u for u in outstanding(cfg, unknowns) if wanted & set(u.blocks)]


def gate_or_raise(
    cfg: dict[str, Any],
    states: Sequence[ProcedureState],
    *,
    simulated: bool,
    allow_placeholders: bool = False,
) -> list[Unknown]:
    """Refuse to run on hardware with placeholders that block the requested states.

    Returns the blocking entries rather than raising when the run is simulated or when
    the operator has explicitly opted in, so the caller can still warn about them.
    Placeholders are the entire point in simulation; on hardware they are how a needle
    ends up somewhere unintended.
    """
    offenders = blocking(cfg, states)
    if not offenders or simulated or allow_placeholders:
        return offenders
    lines = "\n".join(f"  {u.key:<44} {u.what}" for u in offenders)
    raise RuntimeError(
        f"{len(offenders)} unmeasured value(s) block this run on hardware:\n{lines}\n\n"
        "Measure them (`ct-unknowns` explains how), or pass --allow-placeholders if you "
        "know why that is safe here."
    )


def to_markdown(cfg: dict[str, Any] | None = None) -> str:
    """The team-facing document. ``cfg`` adds a column saying what is still outstanding."""
    out = [
        "# What we still need to measure",
        "",
        "Generated by `ct-unknowns --format md`. Do not edit by hand — edit "
        "[`src/ct/unknowns.py`](../src/ct/unknowns.py) and regenerate.",
        "",
        "Each row is a number the rig controller needs. The **key** is where it plugs into the "
        "config: once you have the value, set it there and nothing else has to change.",
        "",
    ]
    for owner, entries in by_owner().items():
        out += [f"## {owner}", ""]
        for u in entries:
            status = ""
            if cfg is not None:
                status = " — **still a placeholder**" if u.is_placeholder(cfg) else " — measured"
            out += [
                f"### `{u.key}`{status}",
                "",
                f"{u.what}",
                "",
                f"- **Units**: {u.units}",
                f"- **Why it matters**: {u.why}",
                f"- **How to measure**: {u.how_to_measure}",
                f"- **Placeholder in use**: `{u.placeholder}`",
            ]
            if u.blocks:
                blocked = ", ".join(s.value for s in u.blocks)
                out.append(f"- **Blocks on hardware**: {blocked}")
            out.append("")
    return "\n".join(out)
