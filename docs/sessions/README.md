# Session log

One document per working session: what changed, why, what was measured, what is next.
Newest last.

Start a new one by copying [000-template.md](000-template.md) to
`NNN-YYYY-MM-DD-slug.md`, then add a line here.

| # | Date | Session | Summary |
|---|---|---|---|
| 001 | 2026-08-01 | [bootstrap](001-2026-08-01-bootstrap.md) | Full pipeline from an empty repo: three swappable stages, four signal sources, FFT/OLS identification, harmonic EKF, forecasting, diagnostics, six CLI scripts, 152 tests. Found that the 95% K rule under-fits the RC-piecewise model, quantified the `R` residual-fallback failure mode, and corrected two formula typos in the reference doc. |
| 002 | 2026-08-15 | [rig control](002-2026-08-15-rig-control.md) | Scope deliberately widened from estimator to rig controller: CAN transport for the RH02 adapter, both CubeMars codecs, base and needle axes, ToF and tactile sensors, the four-state insertion procedure, the lead servo, the firing gate and the safety monitor — all runnable end to end against a simulated plant under `SimClock`, 294 tests. Added `unknowns.py`, the machine-checked list of the 41 physical constants still unmeasured. Found MIT mode's ±12.5 rad position limit, the `load/kp` steady-state insertion error, and that the tactile sensor's stroke must exceed the breathing excursion. |
| 003 | 2026-08-17 | [teensy-can-bench](003-2026-08-17-teensy-can-bench.md) | First real hardware: the `CT_sensors` Teensy 4.1 bench sketch (ToF + encoder over CAN1) built and flashed; added `ct-sensor-bench` to read it live through the RH02. Identified the RH02 as an `slcan` device (not `gs_usb`/candleLight) via direct probing, answering an open `unknowns.py` question. Blocked at the physical layer: zero CAN frames reach the RH02 despite the Teensy actively transmitting — leading suspect is a missing CAN transceiver between the Teensy's TTL-level CAN1 pins and the RH02's CAN-H/CAN-L. |
