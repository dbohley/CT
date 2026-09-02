# 004 — 2026-08-20 — motor-bench-can-protocols

## Goal

Get the rig's three motors (needle, base, phantom) each individually testable through a
slow, small-travel bench script — confirm direction, confirm the right CAN protocol per
motor, confirm they move safely — building on session 003's hardware bring-up. Scope grew
substantially mid-session once the needle motor's protocol turned out to be far less
settled than assumed.

## What changed

**Three standalone bench-test scripts** (`scripts/test_needle_motor.py`,
`scripts/test_base_motor.py`, `scripts/test_phantom_motor.py`) — each drives one motor
through a confirm-gated, `--dry-run`-able, small (5 mm default) extend/retract cycle at
low speed. None of these were ever run against real hardware by Claude — every real move
was run by the user, watching and listening, per an explicit standing rule this session.

**Needle (GL40II) and base (GL60II) settled on the "Gimbal Motor II position/velocity"
protocol**: CAN arbitration ID = `(mode << 8) | node_id` (`mode=1`), 8-byte frame packing
`position` (float32, rad) + `velocity` (float32, rad/s), little-endian, standard
(non-extended) ID. This is **not independently verified against any vendor document** —
it doesn't structurally match either of the two protocols that *are* vendor-confirmed for
this motor family (MIT mode's bit-packed impedance frame; servo mode's extended-ID VESC
commands) — but it is the **one configuration with direct, repeated, empirical proof of
working correctly**: clean extend/retract, correct direction once `DIRECTION_SIGN` was
fixed for each motor, no faults, across many runs. `MOVE_VELOCITY_RAD_S` in this protocol
is a genuine firmware-enforced speed limit — the motor's own trajectory controller
handles smoothness internally, unlike MIT mode.

**Phantom (AK60-6 V3.0) settled on CubeMars's real, vendor-documented servo-mode
protocol** (VESC-derived), via the already-existing, already-tested
`ct.hw.motors.cubemars_servo.CubeMarsServo` codec — reused, not reimplemented. Identified
by observing the motor **autonomously broadcasting VESC-style status frames on an
extended CAN ID with zero commands sent**, which matched that codec's addressing scheme
exactly and is something an MIT-mode motor would never do unprompted.

**A long, ultimately-abandoned MIT-mode detour for the needle motor.** CubeMars's own GUI
tool showed the needle's CAN ID as raw `0x02` (not the mode-shifted `0x102` the working
script used) and the motor configured on the GUI's "MIT" tab. This looked like strong
evidence MIT mode was the real protocol, so `test_needle_motor.py` was rewritten around
`ct.hw.motors.cubemars_mit.CubeMarsMIT`. Confirmed via the GUI directly moving the motor
successfully at `kp=0, kd=0.005, vel=5.0`. Multiple fix attempts followed — gain retuning,
giving `v_des` a genuine (not always-zero) velocity target so `kd` could act as a real
speed governor, reading the motor's actual reported position before ramping instead of
assuming a `0.0` start (to rule out stale post-abort controller state), replicating the
GUI's exact proven `kp`/`kd` at the GUI's proven velocity — **none produced clean,
repeatable motion**: results ranged from a loud garbled-payload noise (wrong frame format,
early on) to a real fast uncontrolled spin (~5-6 rad/s) that tripped an undervoltage fault
and a real power-supply current spike, to clean-but-too-weak-to-move, to an undocumented
fault code (`1`, not in the vendor's own fault table). **Root cause never conclusively
identified.** Decision: stop iterating and revert `test_needle_motor.py` fully back to the
original, proven Gimbal-protocol version, matching `test_base_motor.py`'s already-working
approach.

**A real bug found and fixed in shared library code**,
`ct.hw.motors.cubemars_mit.CubeMarsMIT.parse()`: a reply frame's first byte packs
`id | (err << 4)`, not a bare node id, per the vendor's own CAN protocol table (obtained
mid-session). The codec was reading the whole byte as `node_id`, silently turning fault
reports into nonsense IDs — a "node 146" reading from an early diagnostic session was
actually node 2 reporting an undervoltage fault, misdecoded. Fixed with a `MIT_FAULT_CODES`
mapping and a regression test (`tests/test_codecs.py::test_mit_reply_splits_fault_code_from_node_id`)
that pins the exact bug. Full suite: 295 passed (was 294).

**Diagnostic tooling built to gather evidence rather than guess**, all read-only or
motion-incapable: `scripts/probe_can_id.py` and `scripts/scan_can_bus.py` (send only the
non-motion `CLEAR_ERRORS` frame, scan a range of node IDs), `scripts/listen_needle_motor.py`
and `scripts/listen_phantom_motor.py` (never transmit, decode replies live), and
`scripts/read_needle_position.py` (sends only a genuine zero-effort MIT frame to elicit a
telemetry reply). This pattern — passively listen for autonomous traffic before sending
anything, and actively monitor replies during a real move rather than flying blind — is
what surfaced the servo-mode discovery, the fault-code bug, and the undervoltage/power
findings below.

**Hardware issues hit and resolved along the way:**
- An adapter that stopped transmitting entirely needed a literal USB unplug/replug — a
  motor-board power cycle alone did not clear it. The distinguishing symptom was the
  adapter's own status LED: healthy multi-flicker per run vs. one-flash-then-off vs.
  (worse) no flash at all.
- A stray duplicate `CAN_CHANNEL` assignment (second line silently overriding the first)
  appeared in `test_needle_motor.py`, `test_phantom_motor.py`, and `probe_can_id.py` at
  different points from manual edits — cost real debugging time before being spotted by
  inspection.
- The phantom motor's "no response at all, not even its usual autonomous broadcast" turned
  out to be a power connection issue after a physical bus swap, not a protocol problem.

**Physical CAN topology changed more than once this session.** Started with needle+base
sharing one CANable2 adapter (serial `20563976534B`) and phantom on its own (serial
`207635764E45`). Ended the session with needle+base moved onto the adapter originally
dedicated to phantom, and phantom swapped onto the original needle+base adapter — the
opposite of the starting layout. All three motors confirmed working by the user in this
final configuration.

## Files touched

| File | Change |
|---|---|
| `scripts/test_needle_motor.py` | Extensive back-and-forth (Gimbal protocol → raw-id → MIT mode, several fix attempts → reverted to Gimbal protocol); final state matches `test_base_motor.py`'s approach |
| `scripts/test_base_motor.py` | Drum radius and direction confirmed early; otherwise stable all session |
| `scripts/test_phantom_motor.py` | Rewritten from an initial (wrong) MIT-mode guess to `CubeMarsServo`; radius/direction confirmed; channel updated for the topology swap |
| `src/ct/hw/motors/cubemars_mit.py` | Fixed `parse()`/`encode_reply()` to split the reply's id/fault nibbles correctly; added `MIT_FAULT_CODES` |
| `tests/test_codecs.py` | Added `test_mit_reply_splits_fault_code_from_node_id`; extended the existing round-trip test to check the new `error` field |
| `scripts/listen_needle_motor.py`, `scripts/listen_phantom_motor.py` | New — read-only reply listeners for live diagnosis during a real move |
| `scripts/probe_can_id.py`, `scripts/scan_can_bus.py` | New — minimal-risk (non-motion) send-and-listen probes for CAN ID discovery |
| `scripts/read_needle_position.py` | New — read-only position/velocity/current/fault query via a genuine zero-effort MIT frame |

## Decisions and rationale

- **Position/servo-style protocols over MIT-mode impedance control for needle and
  phantom**, decided explicitly with the user rather than defaulted into. Two reasons:
  the EKF's entire output is a position trajectory (`s_hat`), so position-domain motor
  control is the natural match, not force/impedance control; and MIT mode has no
  built-in velocity limiting (the firmware-side safety property the working protocols
  have), which repeatedly produced faults and one genuine fast-spin incident on the
  bench. The backdrive/compliance requirement (needle must not stall between insertion
  increments) turned out to need only zero-torque disable, which both protocols provide
  identically — MIT mode's finer-grained "soft float" (`kp=0, kd=small`) capability isn't
  needed unless that requirement changes later.
- **The MIT-mode investigation was deliberately abandoned mid-diagnosis**, not resolved.
  Multiple theoretically-sound fixes were tried and none produced clean motion; rather
  than continuing to chase an unexplained failure under bench-test time pressure, the
  team reverted to the proven-working alternative. This is a real open question, not a
  closed one — see below.
- **Never actuate a motor from this session** was an explicit, consistently-honored rule:
  every script that can command motion was written and syntax-checked but never executed
  by Claude; only read-only or motion-incapable diagnostic scripts were run directly.

## Verification

```bash
conda activate CT
pytest -q
# 295 passed in ~41-43s (was 294 before this session's codec fix)

python -m py_compile scripts/test_needle_motor.py scripts/test_base_motor.py \
    scripts/test_phantom_motor.py scripts/listen_needle_motor.py \
    scripts/listen_phantom_motor.py scripts/probe_can_id.py scripts/scan_can_bus.py \
    scripts/read_needle_position.py src/ct/hw/motors/cubemars_mit.py
# all compile cleanly
```

| Check | Expected | Measured |
|---|---|---|
| Full test suite | passes | 295 passed |
| New fault-code regression test | pins the id/err nibble split | passes |
| All three motors move correctly | user-confirmed on real hardware | confirmed by the user after fixing the phantom's power connection (not independently verified by Claude — no hardware runs performed here) |

## Findings

- **A working script's arbitration ID/payload not matching any vendor-documented
  protocol doesn't mean it's wrong** — the Gimbal-protocol addressing used by needle/base
  doesn't structurally match either confirmed CubeMars mode (MIT or servo), yet is the
  only configuration with zero faults across many real runs, while the "correctly"
  reverse-engineered MIT-mode attempt faulted repeatedly despite matching the vendor's
  own spec table byte-for-byte. Empirical proof on real hardware outranks protocol-table
  matching when they disagree.
- **A CAN reply's status byte can silently encode more than one field.** The MIT-mode
  bug (id and fault code sharing one byte, only decoded as id) is the kind of error that
  produces plausible-looking-but-wrong output rather than a crash — worth grep-ing shared
  codecs for similar single-byte-multi-field patterns before trusting their decoded
  output uncritically.
- **"No error, no motion" from a script has at least three distinct real causes seen this
  session**, not one: pointed at the wrong bus/channel (adapter transmits fine, nothing on
  that bus is the intended motor), a genuinely too-weak commanded effort, and a stuck
  adapter that stopped even attempting to transmit. The adapter's own status LED pattern
  (multi-flicker vs. one-flash-then-off vs. no flash) was the most reliable fast
  discriminator between these once established.
- **A steady multimeter reading at a power supply does not rule out undervoltage
  faults.** A fast current transient (e.g. from an uncommanded fast spin) can sag voltage
  locally at the motor for milliseconds — well within what a motor controller's fault
  detection samples, far faster than a handheld meter can catch.

## Open questions

- **Why MIT mode never produced clean motion on the needle motor is unresolved.** Every
  individually-reasoned fix (gain retuning, real `v_des` tracking, actual-position-based
  homing, matching the GUI's exact proven values) failed to reproduce the GUI's own
  successful result. Worth real investigation later, not bench-test-pressure guessing,
  especially since MIT mode's zero-stiffness float may become genuinely necessary if the
  ADVANCE compliance requirement turns out to need more than plain disable.
- **Whether the Gimbal-protocol addressing (`(mode<<8)|node_id`, float32 payload) matches
  any real CubeMars documentation is still unconfirmed.** It works, repeatedly, but its
  origin (which manual section, which product line) was never independently verified the
  way MIT mode and servo mode were.
- **Physical CAN topology is not yet stable or documented outside this chat.** Two swaps
  happened this session alone; worth physically labeling cables/adapters by serial number
  and updating `CAN_CHANNEL` comments/`ct.unknowns` accordingly so this doesn't need
  re-discovering from scratch next session.
- **Phantom's power connection reliability is unconfirmed beyond "worked once after being
  checked."** Worth a few repeated runs to rule out an intermittent/marginal connection
  rather than a one-time fix.
- Carried from session 002/003, still open: which firmware the CubeMars motors are
  flashed with (session partially answers this per-motor now, but not via `unknowns.py`);
  target organ/motion amplitude; clinical tolerance; `forecast_variance` calibration.

## Next steps

1. Physically label the CAN adapters/cables by which motor they currently serve, and
   update `unknowns.py`/config comments to match the final topology, so the next session
   doesn't re-discover this from scratch.
2. Run a few more real needle/base/phantom cycles to build confidence the current
   configuration is stable, not just "worked once."
3. When there's time outside bench-test pressure, revisit the MIT-mode question on the
   needle motor with fresh eyes — the root cause is still genuinely unknown.
4. When ready to move from standalone bench scripts toward real rig integration, port
   these proven protocols into the `ct.hw.motors`/`ct.hw.config` layer (`MotorConfig`,
   `CANAxis`) so the rig controller can drive these motors through the same abstraction
   the simulated plant already uses, rather than the current one-off scripts.
