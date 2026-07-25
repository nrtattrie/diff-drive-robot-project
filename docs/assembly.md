# Stage 2B Assembly and Commissioning Guide

This is the step-by-step procedure for
[Stage 2B](phases.md#stage-2b-incremental-bench-build-and-integration).
Electrical decisions come from the [architecture](architecture.md) and
[interface matrix](../hardware/power-signal-interface-matrix.md). Record actual
results in the
[Stage 2 validation log](../hardware/stage-2-validation-log.md); do not put
unit-specific readings in this reusable guide.

Work exactly one numbered gate at a time. Do not begin the next gate until the
current log entry is complete and approved. A failed gate remains unchecked,
the hardware is returned to the gate's safe end state, and the observed
failure is diagnosed before retesting.

## Non-Negotiable Safety Rules

- Disable every source output and verify de-energized state before changing
  wiring. Do not hot-plug motor, encoder, driver, DROK, or GPIO connections.
- Keep meter leads in `COM` and `V/ohm` for voltage, resistance, and continuity
  work. Never place a meter in current mode directly across a source.
- Use the NICE-POWER output-enable control and conservative current limits.
  Do not treat its display as calibrated evidence; corroborate voltage with
  the multimeter.
- Keep both wheels clear of the surface until Gate B18. Secure motors and
  wheels before applying motor power.
- Never intentionally stall a motor, short a driver output, or defeat the
  selected fuse/protection path.
- Connect the Due to only one powered USB host. Connect the Pi by only one
  power method. USB-C and header power must never be present together.
- Never connect a 5 V encoder output directly to a Due input.
- Keep Dupont, JST-SH, STEMMA QT, breadboard contacts, and the white terminal
  strips out of motor and source-current paths. The four-lead Pi arrangement
  is the one explicit exception and must pass Gates B22 and B23.
- Power resistors become burn hazards. Mount them on a nonflammable,
  heat-tolerant base, keep them guarded, and let them cool before handling.
- Stop immediately for smoke, odor, unexpected noise, loose wiring, cable
  snagging, unstable voltage, excessive heating, repeated resets, or a driver
  fault that is not part of the current controlled test.

## Tools and Test Materials

- NICE-POWER `SPS-C3010` 0–30 V, 0–10 A CC/CV supply
- Digital multimeter with DC voltage, resistance, and continuity modes
- Non-contact IR thermometer with a repeatable measurement spot
- FNIRSI HS-02A soldering kit, suitable solder, flux, and tip cleaner
- Wire stripper, small cutters, screwdrivers, and insulated hand tools
- 16 AWG copper bench/load wire and guarded terminal connections
- One 10 Ω, at least 25 W power resistor
- Two 2 Ω, at least 50 W power resistors
- Heat-safe resistor mounting base or heatsink
- 1 kΩ resistor for controlled EN/DIAG-low simulation
- Marker or tape for consistent shaft and temperature reference points
- Scale, calipers, ruler, and basic mechanical inspection tools
- Computer or Raspberry Pi capable of compiling and uploading Arduino Due code

## Standard Evidence

Every gate log entry records:

- date, operator, hardware marking/revision, and connection state;
- firmware commit/configuration and CAD revision where applicable;
- source voltage/current limit and meter identity;
- expected result, actual readings, and observations;
- photograph references where they clarify assembly or meter setup;
- pass/fail, deviations, corrective action, and owner approval.

Firmware created during these gates belongs under
`software/arduino/diffbot_due/`. The commissioning utility belongs at
`software/arduino/tools/commission_due.py`.

## Commissioning Serial Interface

The firmware uses a deliberately small text interface over the Due Programming
USB port at 115200 baud, 8N1. Each command is one ASCII line terminated by
newline:

```text
ARM
DISARM
SET <left_counts_per_second> <right_counts_per_second>
STOP
RESET_FAULT
STATUS
```

The commissioning build additionally accepts:

```text
TEST_PWM <M1|M2> <signed_duty_permille> <duration_ms>
```

The normal telemetry line is:

```text
STATE <milliseconds> <left_count> <right_count> <left_cps> <right_cps> <m1_mA> <m2_mA> <flags>
```

Implementation rules:

- cap input at 96 bytes and reject overflow, extra fields, missing fields,
  non-numeric values, and values outside configured limits;
- start disarmed with both PWM outputs low and both channels coasting;
- accept `ARM` only with zero targets and no latched/physical fault;
- accept `SET` only while armed and fault-free, and refresh the 500 ms
  watchdog only for an accepted `SET`;
- make `STOP` set both targets to zero through the bounded deceleration/coast
  path without disarming; make `DISARM` force zero/coast and require `ARM`
  before later motion;
- ramp a timed-out target to zero and coast; a later valid `SET` may recover;
- latch driver and encoder-stall faults until `RESET_FAULT` is received while
  disarmed, both targets are zero, and the physical fault is absent;
- run wheel PI control at 200 Hz and publish telemetry at 20 Hz;
- use signed encoder counts per second as the wheel target;
- enable raw-duty test commands only in a commissioning build, cap them by the
  active test limit and 95% absolute ceiling, require a duration, and
  automatically coast when the duration expires; and
- make the commissioning utility send `STOP`, request `STATUS`, and confirm
  state before it offers an operator any motion command.

No checksum or sequence layer is required initially. The parser and watchdog
provide the application safety boundary over the ordered USB link. Add another
integrity layer only if measured communication failures justify it.

## Bench, Due, and Encoder Path

### B1 — Inventory and Evidence Baseline

1. Lay out the active non-deferred baseline: Pi/NVMe/HAT/cooler, official Pi
   supply, Due, `#2507`, two `#4753` motors and harnesses, wheels/hubs/caster,
   `#735`, `#2204`, carrier board, `104` capacitor, DROK, BNO055 and `#954`
   cable, owned Adafruit `#2998` USB hub, Camera Module 3 and Standard-to-Mini
   cable, USB-A-to-micro-B cable, one 23 mm 2x20 GPIO extender, 16 AWG wire,
   14 AWG fuse holder/10 A fuse, ALITOVE splices, and intended low-current
   leads.
2. Record exact markings, quantity, visible damage, connector/header state,
   cable ends and approximate lengths, and all required soldering.
3. Confirm the `#2507`, `#735`, `#2204`, and user-reported owned `#2998` are
   the intended products. Keep the `#2991`, optional sensors, toggle, and white
   terminal strips outside the installed baseline.
4. Confirm the battery, mating connector, undervoltage method, and RPLidar are
   absent/deferred rather than inventory failures.

**Pass:** every in-scope item is positively identified or explicitly recorded
as a blocker; no ambiguous substitute is accepted.

**Safe end state:** all parts remain disconnected and unpowered.

### B2 — Bench Test Rig

1. Inspect the C3010 case, terminals, controls, mains lead, fan openings, and
   fuse area. With nothing connected, set current low, enable the output, and
   compare several conservative voltage settings with the multimeter.
2. Mount the three load resistors on the heat-safe base using 16 AWG leads and
   guarded connections. Measure and record each cold resistance.
3. Provide three mutually exclusive load configurations:
   - 10 Ω alone for about 0.5 A at 5.10 V;
   - one 2 Ω resistor alone for about 2.55 A;
   - both 2 Ω resistors in parallel for about 5.1 A.
4. The 10 Ω branch must be disconnected during the 5.1 A configuration;
   otherwise the demand is about 5.6 A.
5. Prove CC operation at a safe low voltage: connect one 2 Ω branch with the
   output disabled, set a 1 A limit, enable, and confirm the supply reduces
   voltage to hold approximately 1 A. Estimate current from measured
   load voltage divided by the recorded resistance.
6. Verify the output-enable control removes voltage. Inspect leads, resistors,
   and supply heating after the short test.
7. Establish fixed IR measurement locations and verify repeatable readings
   against ambient.

**Pass:** voltage readings are stable and credible, CC/CV and output enable
behave correctly, the load fixture has no exposed live conductors or loose
connections, and temperature can be recorded repeatably.

**Safe end state:** output disabled, voltage/current controls turned down, load
fixture disconnected and cooled.

### B3 — Due Toolchain and Safe Boot

1. Install Arduino CLI or IDE plus the Arduino SAM Boards core.
2. Select the Due Programming Port target
   (`arduino:sam:arduino_due_x_dbg`) and use only the Programming USB port.
3. Create the evolving firmware skeleton. Its first startup actions must make
   D6/D7 outputs LOW before direction/control initialization.
4. Compile, upload, reset, disconnect, and reconnect while motor power and
   driver wiring remain absent.
5. Verify the board enumerates and emits a versioned startup/status line.
6. Measure Due 5 V and 3.3 V relative to Due ground.

**Pass:** repeated upload/reset/reconnect succeeds, startup remains disarmed,
and both rails are stable and plausible.

**Safe end state:** Due may remain connected to one USB host; no external
power or loads are connected.

### B4 — Serial Safety Foundation

1. Implement the interface and parsing rules defined above.
2. Add an explicit state machine for disarmed, armed, watchdog stop, and
   latched fault.
3. Exercise valid commands, blank lines, overlong lines, missing/extra tokens,
   non-numeric targets, and out-of-range targets with motor power absent.
4. Send valid `SET` lines at 20 Hz, stop transmission, and record the watchdog
   transition after 500 ms.
5. Confirm malformed input never arms motion or refreshes the watchdog.
6. Add the Python commissioning utility and make it log timestamped telemetry
   to CSV without requiring ROS.

**Pass:** parser and state-machine tests match the interface, and loss of valid
commands reliably produces the zero-target/coast state.

### B5 — PWM Commissioning

1. Configure D6 and D7 from the Due hardware PWM peripheral at 20 kHz.
2. With all driver and motor wiring absent, connect D6 to temporary capture
   input D3 and measure period and high time using the separate timer channel.
3. Check several duties including 0%, a low commissioning value, 50%, and the
   95% ceiling. Repeat with D7 connected to D3.
4. Verify reset and disarm drive each PWM output LOW.
5. Remove the D3 loopback after testing.

**Pass:** both outputs report approximately 20 kHz, measured duty follows the
command within one percentage point, and all safe states produce LOW.

**Limitation:** this is same-controller commissioning evidence, not an
independent oscilloscope calibration. Record that limitation in the log.

### B6 — Encoder Translator Assembly

1. Inspect the owned Mega Prototype Shield V3 and confirm it provides suitable
   0.1-inch construction with no unexpected pre-connected rail pattern. Use a
   small solderable perfboard only if this inspection fails.
2. Mark socket pin 1 and install the `#2204` so the `#735` remains removable.
3. Wire the translator exactly as listed in the interface matrix:
   `DIR` to 3.3 V, `/OE` to ground, VCC to 3.3 V, ground to common logic
   ground, A1–A4 as inputs, B1–B4 as outputs, A5–A8 grounded, and B5–B8 open.
4. Mount the `104` capacitor directly across pins 20 and 10.
5. If retaining each motor's 1x6 encoder housing, cut two marked 1x6 male
   segments from the unused portion of the `#2507`-supplied 0.1-inch
   breakaway header and solder them to the carrier. Verify the actual motor
   housing cavity order and orientation before mating either connector. Leave
   the two positions corresponding to the removed/cut motor leads electrically
   unconnected and insulate any abandoned contact stubs.
6. Before inserting the translator, continuity-check every intended net and
   inspect for solder bridges, reversed socket orientation, or connected
   unused outputs.
7. Verify the future 5 V encoder rail is isolated from 3.3 V and all B-side/Due
   nets.

**Pass:** the unpowered carrier matches the fixed-wiring table with no
unintended continuity.

### B7 — Encoder Translator Powered Test

1. Insert the `#735` with power absent and confirm orientation.
2. Connect only Due USB, common ground, Due 3.3 V to translator VCC/control,
   and Due 5 V to one encoder supply.
3. Measure 3.3 V at translator VCC and 5 V at encoder blue/green.
4. Turn the encoder by hand and verify A-side signals switch between about
   0 V and 5 V while corresponding B-side signals remain between about
   0 V and 3.3 V.
5. Confirm no Due input is exposed to 5 V before connecting B1–B4 to the Due.

**Pass:** supplies and translated levels are correct and stable with no
cross-rail connection.

### B8 — Left Encoder Hand Test

1. Connect left A/B through B1/B2 to Due D2/D13.
2. Mark the output shaft or attached hub and zero the count.
3. Turn one full revolution slowly in each direction.
4. Record sign and count. Reverse rotation should reverse sign and return near
   the starting count.
5. Repeat several times while moving the harness gently to expose intermittent
   contacts.

**Pass:** each revolution produces 3200 quadrature counts, direction is
consistent, and no intermittent jumps or missing counts appear.

### B9 — Right Encoder Hand Test

Repeat B8 with right A/B through B3/B4 to Due D5/D4 while monitoring both
counters.

**Pass:** the right encoder meets the same count/direction criteria and neither
encoder produces activity on the other counter.

## Driver and Drivetrain

### B10 — Motor-Driver Assembly

1. With the board unpowered, inspect `#2507` markings, PCB damage, mounting
   holes, and included connectors.
2. Solder the three 2-pin terminal blocks and one 13-pin logic-header section.
   Do not install stackable headers or `ARDVIN=VOUT`.
3. Inspect all 19 joints and clean residue if required.
4. Continuity-check each terminal/header to its intended pad and check for
   adjacent bridges.
5. Verify VIN is not shorted to ground and motor outputs are not shorted to
   logic rails. Account for semiconductor/capacitor charging behavior rather
   than treating every momentary meter indication as a short.

**Pass:** the board is mechanically secure and matches the approved
general-purpose configuration.

### B11 — Motor-Driver Logic Test

1. Keep driver VIN and both motors disconnected.
2. With owned male-to-female Dupont leads, put male ends in the Due sockets and
   female ends on the `#2507` male header. Connect Due 3.3 V to `VDD`, join
   logic grounds, and connect D6–D11, D22/D23, and A0/A1 according to the
   matrix; twelve conductors are required in total.
3. Boot disarmed and verify D6/D7 remain LOW while direction lines follow only
   permitted test commands.
4. Record A0/A1 zero readings using 12-bit ADC mode.
5. Observe EN/DIAG with VIN absent. A disabled/fault indication is permitted
   because the driver motor supply is missing; do not require the powered
   normal state at this gate.

**Pass:** the Due remains stable, reset-safe PWM is preserved, ADC readings
are plausible, and no logic line exceeds 3.3 V.

### B12 — First Powered Motor Channel

1. Secure M1 with its wheel clear. Land its factory red/black motor leads
   directly in M1A/M1B.
2. Connect the C3010 to driver VIN/GND with adequate wire and common logic
   reference. Start near 7–8 V and a 0.5–1 A limit.
3. Enable motor power while firmware is disarmed. Verify EN/DIAG indicates the
   powered normal state and the motor remains stopped/coasting.
4. Arm and apply a low, time-bounded duty in one direction, stop, then repeat
   in the other direction.
5. Establish and record which polarity is positive for M1 and make encoder sign
   agree with that convention.
6. Compare A0-derived current using the nominal 0.140 V/A relationship with
   the supply display. Treat this as a plausibility check under PWM, not a
   precision calibration.
7. Test commanded coast and output disable before raising voltage or duty.

**Pass:** M1 turns smoothly both ways, direction/count sign agree, current
telemetry is plausible, and stop/disarm/reset produce coast without a reset or
fault.

### B13 — First Closed-Loop and Safety Test

1. Implement a 200 Hz PI wheel-speed loop using encoder counts. Start at low
   target rates and conservative duty/ramp limits.
2. Tune M1 for stable response without sustained oscillation or current near
   the continuous target while unloaded.
3. With the wheel raised, test boot, Due reset, firmware upload, serial
   disconnect, watchdog stop, reconnect, and recovery.
4. Simulate driver disable/fault by pulling M1 EN/DIAG low through 1 kΩ using a
   controlled temporary connection. Verify both targets coast and motion
   latches off. Remove the connection before `RESET_FAULT`.
5. To test missing-encoder response, power down, disconnect only the encoder
   signal path, restore power, and issue a low target. Verify the motor is
   coasted after the configured grace/detection interval. Power down before
   reconnecting the encoder.

**Pass:** closed-loop speed is stable and every safety transition matches the
selected recoverable-versus-latched policy without a mechanical stall.

### B14 — Modular Drivetrain Deck

1. Measure both accepted motors, hubs, wheels, caster, fasteners, and cable
   exit envelopes.
2. CAD a reusable deck with both motor mounts, caster mount, wheel clearance,
   temporary driver/Due mounting, a modular hole pattern, and tie points for
   both motor-power and USB tethers.
3. Print and inspect the deck before installing electronics.
4. Mount both motors, wheels, and caster; apply required threadlocker only
   where specified by the fastener/component instructions.
5. Verify free wheel rotation, ground clearance, fastener engagement, cable
   clearance, and strain relief.
6. Record physical wheel-center separation, nominal wheel radius, caster
   position, deck envelope, and assembled deck mass.

**Pass:** the deck rolls freely, nothing rubs or loosens, and the measured
geometry is recorded for later CAD/software synchronization.

### B15 — Second Powered Motor Channel

Mount/connect M2 and repeat B12 independently using M2, D7, D10/D11, D23, A1,
and the right encoder.

**Pass:** M2 meets the accepted M1 direction, current, coast, and reset
criteria with its own polarity recorded.

### B16 — Raised Two-Channel Test

1. Raise and secure both wheels.
2. Command each channel alone in both directions while monitoring both encoder
   counters and current channels.
3. Command matched and deliberately unmatched targets.
4. Repeat simultaneous start/stop and direction changes within conservative
   acceleration limits.
5. Monitor supply voltage, Due resets, USB behavior, encoder integrity, and
   EN/DIAG.

**Pass:** both channels remain independent, simultaneous operation is stable,
and no electrical-noise-induced count, fault, USB, or reset problem appears.

### B17 — Two-Channel PI Tuning

1. Apply one shared PI baseline to both channels.
2. Record steady rate, error, current, and transient behavior at several
   conservative forward/reverse targets.
3. Keep shared gains if both channels meet the same stability/tracking
   criteria. Add channel-specific values only when repeatable evidence shows
   the shared values cannot meet them.
4. Confirm the 95% absolute duty ceiling remains enforced.

**Pass:** both raised wheels track commanded rates without sustained
oscillation and the reason for shared or separate values is documented.

### B18 — Low-Speed Floor Commissioning

1. Choose a clear level lane and keep the Pi out of the control path. Use the
   commissioning computer, Due USB, and current-limited motor supply.
2. Secure both tethers to the deck so connector contacts cannot take a pull.
   One operator stays at the supply/output control and manages slack.
3. Begin well below 0.15 m/s with short forward, coast, and reverse commands.
4. Increase only after straight motion, direction, current, cable behavior,
   and stopping are predictable.
5. Tune acceleration, controlled deceleration, near-zero coast transition,
   stall arming command, startup grace, minimum encoder progress, and detection
   interval.
6. Do not use stationary pivots or a mechanical stall to tune protection.

**Pass:** the deck performs repeatable short shuttles without slip, oscillation,
cable loading, unexpected fault, or unsafe stopping.

## Pi, Peripherals, and Power

### B19 — Pi-to-Due Integration

1. Power the Pi from the official 27 W USB-C supply.
2. Disconnect any development-computer USB host from the Due, then connect one
   Pi USB-A port to the Due Programming port with the data-capable cable.
3. Run the commissioning utility from the Pi and log status/telemetry.
4. Exercise Pi-first and Due-first availability, Pi reboot, USB disconnect,
   reconnect, Due reset, and firmware upload.
5. Verify no stale command causes motion after any transition.

**Pass:** command and telemetry remain reliable, only one USB host powers the
Due, and every interruption produces the selected safe state.

### B20 — BNO055 Acceptance

1. With the Pi on official USB-C power, inspect/solder the BNO055 header if
   required.
2. Plug the owned `#2998` captive USB-A lead into one black Pi USB 2.0 port.
   Leave its 1.35 mm external-power jack empty, select and mark hub port 1,
   and keep the hub's single downstream-power switch on.
3. Plug the USB end of `#954` into hub port 1. Connect `#954` red to `VIN`,
   black to ground, white/RX to BNO SDA/TX, and green/TX to BNO SCL/RX. Strap
   BNO `3VO` to `PS1` with one owned female-to-female jumper; leave `PS0` at
   default.
4. Do not connect STEMMA I2C simultaneously.
5. Verify hub and `#954` enumeration, switch off/on recovery, UART
   communication at 115200 baud, self-test/system
   status, plausible stationary readings, and expected axis response when the
   board is turned by hand.
6. Measure observed update behavior without claiming final latency,
   calibration, or fusion suitability.

**Pass:** UART data and status are stable and the future mounting orientation
is documented. Navigation fusion remains deferred.

### B21 — Camera Acceptance

1. Power down the Pi before handling the FFC.
2. Verify the narrow 22-pin end goes to the Pi and the wider 15-pin end to the
   camera with contact orientation matching the connectors.
3. Boot on official USB-C power, verify camera detection, and stream a test
   image.
4. Confirm the intended forward/up orientation and cable bend/service
   envelope for CAD.

**Pass:** detection and streaming are repeatable with no FFC damage or
intermittent connection.

### B22 — DROK and Dummy-Load Acceptance

1. Record the exact DROK markings, terminal labels, adjustment type, physical
   condition, and whether input/output grounds are common.
2. Build the intended four-lead output harness using two equal short positive
   leads and two equal short grounds. Terminate them on a secured 2.54 mm
   header fixture representing Pi pins 2/4 and 6/14; do not use the Pi as the
   dummy-load connector.
3. Feed the DROK from the C3010 at a representative 3S voltage with a
   conservative current limit. With no output load, adjust to 5.10 V using the
   multimeter at the harness end.
4. Disable output, power-cycle repeatedly, and verify the same stable setting.
   Record that fast startup transients remain outside ordinary multimeter
   bandwidth unless the meter provides a suitable capture mode.
5. With output disabled between every reconfiguration, test:
   - 10 Ω alone for about 0.5 A;
   - one 2 Ω branch alone for about 2.55 A;
   - both 2 Ω branches in parallel for about 5.1 A, with 10 Ω disconnected.
6. At each level record converter input/output voltage, voltage at the harness
   end, calculated load current, and IR temperatures at fixed converter,
   connector, and resistor locations.
7. Hold the final load long enough to establish regulation and a credible
   temperature trend, up to 30 minutes. Stop for instability or continued
   unsafe heating.
8. Let the fixture cool, then secure/cover the DROK adjustment. Never adjust
   it later with the Pi connected.

**Pass:** the complete output harness sustains each load with correct polarity,
stable voltage, acceptable drop, no loose/hot contact, and no progressive
converter failure. Ordinary-meter bandwidth limitations are recorded rather
than hidden.

### B23 — Pi Header Power and Combined Load

1. Power everything down and remove Pi USB-C.
2. Continuity- and polarity-check the four-lead harness at the Pi connector
   before installation. Confirm one owned 23 mm 2x20 extension exposes the
   correct pins through the actual M.2 HAT+ stack and that the jumper ends mate
   securely. Connect positives to physical pins 2/4 and grounds to 6/14.
3. Feed the DROK through the 10 A fuse and selected 16 AWG/ALITOVE
   distribution from the C3010. Leave the motor branch disabled initially.
4. Enable power and exercise Pi boot, NVMe, Active Cooler, CPU load, Due,
   `#2998`/BNO055, and camera. Monitor Pi undervoltage/USB warnings, resets,
   hub enumeration/recovery, harness-end voltage, and connection temperatures.
5. Enable the motor branch and repeat during simultaneous raised-wheel starts
   and operation.
6. Enable `PSU_MAX_CURRENT=5000` only after the complete path passes, then
   verify the Pi reports the intended high-current mode.

**Pass:** no polarity error, undervoltage, unstable output, excessive drop,
heating, USB fault, or Pi/Due reset occurs under combined operation.

## Final Integration and Gate Closure

### B24 — Final Electronics-Tier CAD

1. Use measured component envelopes and accepted connection directions rather
   than catalog-only dimensions.
2. Provide service access to Pi, Due Programming USB/reset, `#2507` terminals,
   fuse, DROK adjustment cover, `#2998` switch/ports, BNO055, camera connector,
   and wire clamps.
3. Preserve Pi/NVMe/cooler airflow and separate motor/source-current routing
   from encoder and logic wiring.
4. Define and mark the BNO055 orientation and camera view.
5. Reserve noncommittal envelopes and mounting options for the deferred
   battery and RPLidar without selecting either component.
6. Review the CAD against the matrix before printing.

**Pass:** every installed component, connector, cooling path, and strain-relief
point is accounted for without assuming deferred hardware.

### B25 — Final Mechanical and Wiring Build

1. Print and inspect the electronics tier; reject warped, cracked, obstructed,
   or weak mounting features.
2. Mount the Pi/NVMe/cooler, Due, driver, translator carrier, DROK, `#2998`,
   BNO055, and camera with serviceable fasteners and electrical clearance.
3. Build the tethered harness from the interface matrix:
   bench source input to 10 A fuse and ALITOVE distribution, one 16 AWG branch
   to driver VIN, and one 16 AWG branch to DROK.
4. Keep the bench-source input removable because the battery connector remains
   deferred.
5. Route and secure logic, encoder, USB, camera, and UART leads. Keep the Due
   direct to one black Pi USB port, the `#2998` on the other black port with
   `#954` on marked hub port 1, and its external-power jack empty. No loose
   breadboard connection is allowed in the integrated build.
6. With all sources absent, verify ALITOVE topology, DROK ground relationship,
   fuse continuity, polarity, every point-to-point net, and absence of
   cross-rail shorts.
7. Tug-test every termination individually.

**Pass:** mechanical access and routing match CAD, every installed interface
matches the matrix, and the complete unpowered build passes inspection.

### B26 — Integrated Raised Regression

1. Raise and secure both wheels.
2. Apply power in the documented order and verify disarmed/coast startup.
3. Repeat all safety transitions: reset, upload, valid-command loss,
   recoverable watchdog, simulated driver fault, and missing-encoder stall.
4. Exercise both speed loops forward/reverse and simultaneous start/stop.
5. Monitor encoder integrity, current/fault telemetry, Pi/Due resets,
   undervoltage, direct USB and hub stability, BNO055 updates, camera stream,
   and connection temperatures.
6. Repeat multiple cold starts and orderly shutdowns.

**Pass:** the integrated build preserves every subsystem acceptance result and
introduces no reset, noise, thermal, wiring, or peripheral regression.

### B27 — Loaded Thermal Shuttle

1. Measure and record the actual complete Stage 2 mass. Do not add speculative
   battery/LiDAR ballast.
2. Use a clear, level lane with strain-relieved power leads, an operator
   managing slack, and immediate access to supply output disable.
3. Use approximately 0.15 m/s and repeat:
   - about 0.75 m forward;
   - five seconds coast;
   - about 0.75 m reverse;
   - five seconds coast.
4. Include occasional gentle differential-speed arcs; do not perform repeated
   stationary pivots.
5. Every five minutes record ambient, both motor-case temperatures, both
   driver-area temperatures, supply/harness voltage, per-channel current
   telemetry, faults, resets, and observed performance.
6. Run 30 minutes, then extend in 10-minute blocks up to 60 minutes until
   temperature rise is no more than 2 °C across the final 10 minutes.
7. Stop for cable snagging, looseness, odor/smoke, unexpected noise,
   fault/reset, Pi undervoltage, normal loaded current inconsistent with the
   1.38 A-per-motor target, continued temperature rise, or progressive
   performance loss.

**Pass:** normal loaded current remains consistent with the design target,
temperatures approach the defined stable trend, and no electrical, mechanical,
software, or connection failure occurs.

### B28 — Close the Hardware Gate

1. Transfer accepted polarity, PI, current, acceleration, coast, watchdog, and
   stall values into the tracked firmware configuration.
2. Export accepted chassis/tier STEP and STL files into `hardware/cad/`; keep
   SolidWorks sources in Google Drive according to project policy.
3. Measure the final wheel-center separation, nominal wheel radius, chassis
   envelope, and mass. Update the URDF and controller configuration from those
   values; leave empirical effective-radius/separation correction to Stage 3
   odometry calibration.
4. Expand both default and `use_ros2_control:=true` xacro paths and rebuild the
   ROS package.
5. Review every Stage 2A Q1–Q11 decision against the completed log. Update the
   architecture/matrix only for evidence-driven changes.
6. Confirm every B1–B28 log entry is complete and approved before marking the
   architecture bench validated.

**Pass:** firmware, CAD exports, software geometry, architecture, matrix,
phase tracker, and validation evidence all describe the same accepted robot.
