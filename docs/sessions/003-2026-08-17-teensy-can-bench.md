# 003 — 2026-08-17 — teensy-can-bench

## Goal

Get the `CT_sensors` Teensy 4.1 bench sketch (VL53L4CD ToF + AS5048B encoder, CAN1
output) building and flashing, then write a script in this repo that reads that data live
through the RH02 Plus USB-CAN-FD adapter and prints it to a terminal, as the next bring-up
step before this hardware gets wired into the rig controller proper.

## What changed

**`CT_sensors` (separate PlatformIO project, not this repo) now builds and flashes
cleanly.** The only real gap was a missing `lib_deps` entry for `vl53l4cd_class.h`
(`stm32duino/STM32duino VL53L4CD`) — `FlexCAN_T4` is bundled with the Teensy Arduino
framework and needed nothing. An earlier `Arduino.h`-not-found error was just the
Teensy toolchain/framework packages still downloading on first project open; they're
fully installed now. Built, uploaded, and verified over serial: ToF and encoder readings
are live and updating.

**Added `ct-sensor-bench`**, a new script in this repo
([sensor_bench.py](../../src/ct/cli/sensor_bench.py)) that opens the RH02 via
`ct.hw.bus.canfd_rh02.RH02Bus` (reused as-is, not reimplemented) and decodes the sketch's
specific 8-byte frame — `uint16` ToF mm, `float32` encoder-derived cm, `int16` angle
centidegrees — printing one line per frame. Deliberately **not** routed through
`RigSession`/`configs/rig_bench.yaml`: that config targets the rig's eventual wiring (1
Mbit/s, separate CAN IDs per sensor), which is not what this sketch sends today (500
kbps, everything bundled onto ID 5). This script talks to what's actually on the wire.

**`python-can` toolchain installed in the `CT` conda env** (was not previously present —
it's an optional extra, `pip install -e ".[can]"`). Also installed `gs-usb`, `pyserial`,
and (via Homebrew) `libusb`, while working out which backend the RH02 needs.

**Identified the RH02's actual python-can backend — this answers an open unknown.** It
enumerates on macOS as a **CANable2** (Openlight Labs / `github.com/normaldotcom/canable2`
firmware, VID `0x16d0` / PID `0x117e`). A `gs_usb` (candleLight) device scan found zero
matching devices — this is not that firmware. A raw serial probe against
`/dev/cu.usbmodem20563976534B1` confirmed it speaks the LAWICEL/**slcan** protocol
directly: a `V` command returned `16e7497-dirty github.com/normaldotcom/canable2.git`.
So `interface=slcan`, `channel=<its /dev/cu.usbmodem... path>` is the pairing
`ct.unknowns`'s `rig.buses.sensing.interface`/`.channel` entries have been waiting on —
see Open questions below on formally recording this.

**Blocked on physical CAN wiring.** With `ct-sensor-bench` connected and listening, and
the Teensy independently confirmed still actively calling `can1.write()` every loop (its
own serial console kept printing normally throughout), **zero frames arrived at the
RH02** over several seconds of testing (expected ~5/s). Diagnosis below.

## Files touched

| File | Change |
|---|---|
| `/Users/declanbohley/Documents/PlatformIO/Projects/CT_sensors/platformio.ini` | added `lib_deps = stm32duino/STM32duino VL53L4CD@^1.0.5` and `monitor_speed = 115200` (separate repo, not `CT`) |
| `src/ct/cli/sensor_bench.py` | new — `ct-sensor-bench`, live CAN frame reader/decoder for the bench sketch |
| `pyproject.toml` | registered `ct-sensor-bench = "ct.cli.sensor_bench:main"` |
| CT conda env | `pip install -e ".[can]"`, `pip install gs-usb pyserial`; `brew install libusb` (not repo files, but needed to reproduce) |

## Decisions and rationale

- **Reused `RH02Bus`/`build_bus_from_config` rather than talking to `python-can`
  directly.** It already does the right thing (receive thread off the main loop into a
  `FrameQueue`/`Mailbox`, per rule 4) — no reason to duplicate it for a bench tool.
- **Kept the bench script outside `RigSession`.** The sketch's CAN framing (bitrate,
  single shared ID, packed multi-field payload) is intentionally provisional and does not
  match `configs/rig_bench.yaml`'s target wiring. Routing this through the rig config
  would either silently misrepresent the sketch's real behavior or require prematurely
  deciding the final per-sensor CAN framing. Reconciling the two is future work (see Next
  steps), not something to paper over now.
- **`slcan`, not `gs_usb`, for the RH02.** Determined by direct evidence (version-string
  probe), not assumption — the USB descriptor alone (CANable2-branded) was suggestive but
  not conclusive, since CANable-family boards ship with either candleLight or slcan
  firmware depending on build.

## Verification

```bash
# CT_sensors build + upload (from that project's directory)
~/.platformio/penv/bin/pio run
# ========================= [SUCCESS] Took 7.90 seconds =========================
# FLASH: code:22296, data:6328, headers:8236   free for files:8089604
# RAM1: variables:17472, code:19344, padding:13424   free for local variables:474048
# RAM2: variables:12416  free for malloc/new:511872

~/.platformio/penv/bin/pio run -t upload
# ========================= [SUCCESS] Took 2.39 seconds =========================

# Serial verification (raw pyserial read, since `pio device monitor` needs a real tty)
# ToF distance: 177-180 mm (live, changing)
# AS5048B zeroed angle: ~0.00 deg (encoder untouched, correctly near zero)

# RH02 probing (CT conda env)
python -c "from gs_usb.gs_usb import GsUsb; print(len(GsUsb.scan()))"
# 0  -> not a gs_usb/candleLight device

# raw LAWICEL probe over /dev/cu.usbmodem20563976534B1 @ 115200
# sent b'V\r' -> received b'16e7497-dirty github.com/normaldotcom/canable2.git\r'

conda activate CT
ct-sensor-bench --interface slcan --channel /dev/cu.usbmodem20563976534B1 --bitrate 500000
# connecting: interface=slcan channel=/dev/cu.usbmodem20563976534B1 bitrate=500000 can_id=5
# connected. waiting for frames (Ctrl+C to stop)...
# (no frames in 5+ seconds, while the Teensy's own serial console kept printing readings)
```

| Check | Expected | Measured |
|---|---|---|
| `CT_sensors` builds | exit 0 | exit 0, 7.90s |
| `CT_sensors` uploads | exit 0 | exit 0, 2.39s |
| ToF/encoder live over serial | changing values | ToF 177-180mm live; angle ~0.00deg (untouched) |
| RH02 backend | unknown going in | `slcan`, not `gs_usb` — confirmed by version-string probe |
| CAN frames reach RH02 | ~5/s on ID 5 | **0** over 5+s, despite confirmed active Teensy TX |

## Findings

- **The `Arduino.h` build error was a race with first-time package download, not a real
  problem.** Once `framework-arduinoteensy`/`tool-teensy`/`toolchain-gccarmnoneeabi-teensy`
  finished installing, the same source built without any change to that part.
- **The RH02 Plus's python-can pairing is `interface=slcan`, `channel=<its serial
  device path>`.** This directly answers the `rig.buses.sensing.interface`/`.channel`
  entries in [unknowns.py](../../src/ct/unknowns.py) — worth promoting there once a real
  frame has actually been received end to end (see Open questions; not promoted yet
  because it's unconfirmed with live traffic).
- **A transmitting CAN node with no other node acknowledging on the bus fails silently
  from the application's point of view.** If the RH02's transceiver isn't actually active
  on the bus, the Teensy's FlexCAN peripheral would see ACK errors, and enough of those
  push it into bus-off — `main.cpp` never checks error state, so nothing about the
  sketch's own behavior would look any different. This is a real alternative explanation
  to a pure wiring gap, and worth ruling out once physical wiring is confirmed correct.

## Open questions

- **Is there a CAN transceiver between the Teensy's CAN1 pins (22 TX / 23 RX — TTL logic,
  not differential CAN) and the RH02's CAN-H/CAN-L?** This is the leading suspect for why
  zero frames arrive. If pins 22/23 go straight to the RH02 with nothing in between, that
  alone explains it. Needs a physical check — carried to next steps, not resolved here.
- Is there a shared ground between the Teensy and the RH02, and termination resistors
  (120Ω) at each end of the bus? Secondary suspects if a transceiver turns out to already
  be present.
- **`rig.buses.sensing.interface`/`.channel` in `unknowns.py`**: strong candidate answer
  now (`slcan` / the RH02's `/dev/cu.usbmodem...` path), but held back from being marked
  resolved until a real frame is actually received — a value that looks right without
  ever seeing traffic is exactly the kind of unverified placeholder `unknowns.py` exists
  to flag.
- Carried from session 002, still open: which firmware the CubeMars motors are flashed
  with; target organ/motion amplitude; clinical tolerance; insertion depth/velocity;
  `forecast_variance` calibration.

## Next steps

1. Check for a CAN transceiver chip in the physical path between the Teensy's CAN1 pins
   and the RH02; add one (with shared ground and termination) if it's missing.
2. Re-run `ct-sensor-bench --interface slcan --channel /dev/cu.usbmodem20563976534B1
   --bitrate 500000` once wiring is fixed; confirm live ToF/angle values track the
   Teensy's own serial output.
3. Once frames are confirmed arriving, promote the `slcan` interface/channel finding into
   `unknowns.py`'s `rig.buses.sensing` entries.
4. Longer term, not blocking: reconcile the bench sketch's CAN framing (500 kbps, shared
   ID 5, packed 3-field payload) with `configs/rig_bench.yaml`'s target wiring (1 Mbit/s,
   separate `tactile`/`tof` CAN IDs 256/257, single-scalar frames) when this moves from
   bench test to actual rig integration.
