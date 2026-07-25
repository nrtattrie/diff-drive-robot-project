# Stage 2B Validation Log

This file records unit-specific evidence for the incremental build defined in
[the phase tracker](../docs/phases.md#stage-2b-incremental-bench-build-and-integration)
and performed with the [assembly guide](../docs/assembly.md).

**Current status:** B1 and B2 were accepted by the owner on 2026-07-25 based
on inspection and testing completed before repository closeout. B3 is next.
Stage 2A selections are not fully bench validated until B1–B28 pass.

## Log Rules

- Work and approve one gate at a time.
- Enter `PASS`, `FAIL`, or `BLOCKED`; do not use an empty observation as proof.
- Record raw readings and the setup that produced them. Do not record only a
  conclusion.
- Identify firmware commits/configurations and CAD revisions used for tests.
- Link photographs by repository-relative path if photographs are retained.
- Preserve failed attempts and corrective actions; append a retest rather than
  erasing evidence.
- A gate may be checked in `docs/phases.md` only after its owner-approval field
  below is complete.

## Build Identity

| Field | Value |
|---|---|
| Build identifier | TBD |
| Primary operator | Nathan |
| Stage 2 start date | 2026-07-25 |
| Firmware path | `software/arduino/diffbot_due/` |
| Commissioning utility | `software/arduino/tools/commission_due.py` |
| Drivetrain CAD revision | TBD |
| Electronics-tier CAD revision | TBD |
| Final Stage 2 mass | TBD |

## Instrument Register

| Instrument | Model/marking | Relevant range or mode | Acceptance/reference result |
|---|---|---|---|
| Bench supply | NICE-POWER `SPS-C3010` | 0–30 V, 0–10 A CC/CV | PASS B2 — previously tested and accepted by owner |
| Multimeter | Model not recorded | DC V, resistance, continuity | PASS B2 — previously tested and accepted by owner |
| IR thermometer | Model not recorded | Temperature and distance-to-spot behavior | PASS B2 — previously tested and accepted by owner |
| Scale | TBD | Robot mass | Pending B14/B27 |
| Calipers/ruler | TBD | Mechanical geometry | Pending B14/B28 |

## Master Gate Status

| Gate | Result | Approval date |
|---|---|---|
| B1 Inventory and evidence baseline | PASS | 2026-07-25 |
| B2 Bench test rig | PASS | 2026-07-25 |
| B3 Due toolchain and safe boot | Pending | — |
| B4 Serial safety foundation | Pending | — |
| B5 PWM commissioning | Pending | — |
| B6 Encoder translator assembly | Pending | — |
| B7 Encoder translator powered test | Pending | — |
| B8 Left encoder hand test | Pending | — |
| B9 Right encoder hand test | Pending | — |
| B10 Motor-driver assembly | Pending | — |
| B11 Motor-driver logic test | Pending | — |
| B12 First powered motor channel | Pending | — |
| B13 First closed-loop and safety test | Pending | — |
| B14 Modular drivetrain deck | Pending | — |
| B15 Second powered motor channel | Pending | — |
| B16 Raised two-channel test | Pending | — |
| B17 Two-channel PI tuning | Pending | — |
| B18 Low-speed floor commissioning | Pending | — |
| B19 Pi-to-Due integration | Pending | — |
| B20 BNO055 acceptance | Pending | — |
| B21 Camera acceptance | Pending | — |
| B22 DROK and dummy-load acceptance | Pending | — |
| B23 Pi header-power and combined-load acceptance | Pending | — |
| B24 Final electronics-tier CAD | Pending | — |
| B25 Final mechanical and wiring build | Pending | — |
| B26 Integrated raised regression | Pending | — |
| B27 Loaded thermal shuttle | Pending | — |
| B28 Close the hardware gate | Pending | — |

## B1 — Inventory and Evidence Baseline

**Result:** PASS — owner-attested acceptance  
**Date/operator:** 2026-07-25 / Nathan  
**Assembly procedure:** [B1](../docs/assembly.md#b1--inventory-and-evidence-baseline)

| Item | Quantity | Exact marking/revision | Connector/assembly state | Condition or blocker |
|---|---:|---|---|---|
| Active baseline | As documented in BOM REV 02 and the Stage 2A records | Owner confirmed the equipment is correct | Owner accepted existing state | No blocker reported |

**Photographs/notes:** Detailed inventory evidence was not requested or
retained; the owner confirmed the equipment is correct and directed B1
closeout.  
**Deviations or corrective action:** None reported.  
**Owner approval:** Nathan — approved 2026-07-25.

## B2 — Bench Test Rig

**Result:** PASS — owner-attested prior testing  
**Date/operator:** 2026-07-25 / Nathan  
**Assembly procedure:** [B2](../docs/assembly.md#b2--bench-test-rig)

| Test | Setup | Expected | Actual |
|---|---|---|---|
| Supply no-load voltage points | Prior owner test | Stable and meter-consistent | PASS per owner; readings not transcribed |
| 10 Ω cold resistance | Prior owner test | Near marked value | PASS per owner; reading not transcribed |
| 2 Ω branch A cold resistance | Prior owner test | Near marked value | PASS per owner; reading not transcribed |
| 2 Ω branch B cold resistance | Prior owner test | Near marked value | PASS per owner; reading not transcribed |
| CC test | Prior owner test | Voltage reduces to hold about 1 A | PASS per owner; readings not transcribed |
| Output enable | Prior owner test | Removes output | PASS per owner |
| IR repeatability | Prior owner test | Repeatable reading | PASS per owner; readings not transcribed |

**Safety/fixture inspection:** Accepted by owner based on prior testing.  
**Deviations or corrective action:** None reported.  
**Owner approval:** Nathan — approved 2026-07-25.

## B3 — Due Toolchain and Safe Boot

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B3](../docs/assembly.md#b3--due-toolchain-and-safe-boot)

| Evidence | Actual |
|---|---|
| Arduino CLI/IDE and SAM core versions | TBD |
| FQBN and Programming USB device | TBD |
| Firmware commit | TBD |
| Compile/upload result | TBD |
| Reset/reconnect observations | TBD |
| Due 5 V rail | TBD |
| Due 3.3 V rail | TBD |
| Startup/disarmed output state | TBD |

**Owner approval:** TBD

## B4 — Serial Safety Foundation

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B4](../docs/assembly.md#b4--serial-safety-foundation)

| Scenario | Expected state | Actual state/result |
|---|---|---|
| Boot | Disarmed/coast | TBD |
| Valid arm/stop/disarm | Defined transition | TBD |
| Valid `SET` at 20 Hz | Watchdog remains fresh | TBD |
| Stop valid `SET` traffic | Zero target/coast after 500 ms | TBD |
| Blank/overlong/malformed input | Rejected; no watchdog refresh | TBD |
| Out-of-range target | Rejected | TBD |
| CSV telemetry logging | Stable and parseable | TBD |

**Firmware/utility commits:** TBD  
**Owner approval:** TBD

## B5 — PWM Commissioning

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B5](../docs/assembly.md#b5--pwm-commissioning)

| Output | Commanded duty | Captured frequency | Captured duty | Reset/disarm LOW |
|---|---:|---:|---:|---|
| D6/M1PWM | 0% | TBD | TBD | TBD |
| D6/M1PWM | Low test | TBD | TBD | TBD |
| D6/M1PWM | 50% | TBD | TBD | TBD |
| D6/M1PWM | 95% | TBD | TBD | TBD |
| D7/M2PWM | 0% | TBD | TBD | TBD |
| D7/M2PWM | Low test | TBD | TBD | TBD |
| D7/M2PWM | 50% | TBD | TBD | TBD |
| D7/M2PWM | 95% | TBD | TBD | TBD |

**Same-controller measurement limitation recorded:** TBD  
**Owner approval:** TBD

## B6 — Encoder Translator Assembly

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B6](../docs/assembly.md#b6--encoder-translator-assembly)

| Inspection | Result/reading |
|---|---|
| Carrier/rail topology | TBD |
| Socket and pin-1 orientation | TBD |
| `DIR`, `/OE`, VCC, GND continuity | TBD |
| A1–A4/B1–B4 mapping | TBD |
| A5–A8 grounded; B5–B8 open | TBD |
| `104` capacitor placement | TBD |
| 5 V-to-3.3 V/Due isolation | TBD |
| Solder-joint inspection | TBD |

**Owner approval:** TBD

## B7 — Encoder Translator Powered Test

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B7](../docs/assembly.md#b7--encoder-translator-powered-test)

| Measurement | Expected | Actual |
|---|---|---|
| Encoder supply | About 5 V | TBD |
| Translator VCC | About 3.3 V | TBD |
| A-side LOW/HIGH | About 0/5 V | TBD |
| B-side LOW/HIGH | About 0/3.3 V | TBD |
| Highest voltage at a Due input | No 5 V exposure | TBD |

**Owner approval:** TBD

## B8 — Left Encoder Hand Test

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B8](../docs/assembly.md#b8--left-encoder-hand-test)

| Trial | Direction | Start count | End count | Delta | Harness movement issue |
|---:|---|---:|---:|---:|---|
| 1 | TBD | TBD | TBD | TBD | TBD |
| 2 | TBD | TBD | TBD | TBD | TBD |
| 3 | TBD | TBD | TBD | TBD | TBD |

**Accepted positive convention:** TBD  
**Owner approval:** TBD

## B9 — Right Encoder Hand Test

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B9](../docs/assembly.md#b9--right-encoder-hand-test)

| Trial | Direction | Right delta | Unwanted left delta | Harness issue |
|---:|---|---:|---:|---|
| 1 | TBD | TBD | TBD | TBD |
| 2 | TBD | TBD | TBD | TBD |
| 3 | TBD | TBD | TBD | TBD |

**Accepted positive convention:** TBD  
**Owner approval:** TBD

## B10 — Motor-Driver Assembly

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B10](../docs/assembly.md#b10--motor-driver-assembly)

| Inspection | Result |
|---|---|
| Board marking/revision | TBD |
| Three terminal blocks | TBD |
| 13-pin general-purpose header | TBD |
| Stackable headers absent | TBD |
| `ARDVIN=VOUT` absent | TBD |
| Joint/bridge inspection | TBD |
| Terminal/header continuity | TBD |
| VIN/GND and output isolation behavior | TBD |

**Owner approval:** TBD

## B11 — Motor-Driver Logic Test

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B11](../docs/assembly.md#b11--motor-driver-logic-test)

| Check | Actual |
|---|---|
| VDD and logic-ground voltage | TBD |
| D6/D7 reset/disarmed state | TBD |
| D8–D11 commanded behavior | TBD |
| D22/D23 with VIN absent | TBD |
| A0 12-bit zero reading | TBD |
| A1 12-bit zero reading | TBD |
| Due reset/USB behavior | TBD |

**Owner approval:** TBD

## B12 — First Powered Motor Channel

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B12](../docs/assembly.md#b12--first-powered-motor-channel)

| Parameter/scenario | Actual |
|---|---|
| Supply voltage/current limit | TBD |
| M1 terminal polarity for positive motion | TBD |
| Encoder sign agreement | TBD |
| EN/DIAG powered-normal state | TBD |
| Low-duty forward/reverse behavior | TBD |
| Unloaded supply current | TBD |
| A0/current-sense plausibility | TBD |
| Coast/disarm/reset behavior | TBD |

**Owner approval:** TBD

## B13 — First Closed-Loop and Safety Test

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B13](../docs/assembly.md#b13--first-closed-loop-and-safety-test)

| Scenario | Target/setting | Result and stop time |
|---|---|---|
| PI low-speed step | TBD | TBD |
| PI reverse step | TBD | TBD |
| Due reset | TBD | TBD |
| Firmware upload | TBD | TBD |
| USB loss/watchdog | 500 ms watchdog | TBD |
| EN/DIAG-low simulation | 1 kΩ to ground | TBD |
| Missing encoder | Low target, wheel raised | TBD |
| Latched-fault reset conditions | Disarmed/zero/cleared | TBD |

**Initial M1 PI and safety parameters:** TBD  
**Owner approval:** TBD

## B14 — Modular Drivetrain Deck

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B14](../docs/assembly.md#b14--modular-drivetrain-deck)

| Measurement/revision | Actual |
|---|---|
| CAD revision/source location | TBD |
| Export paths | TBD |
| Print material/profile | TBD |
| Wheel-center separation | TBD |
| Nominal wheel radius | TBD |
| Caster position/clearance | TBD |
| Deck envelope | TBD |
| Mounted drivetrain mass | TBD |
| Free rotation/fastener/strain-relief inspection | TBD |

**Owner approval:** TBD

## B15 — Second Powered Motor Channel

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B15](../docs/assembly.md#b15--second-powered-motor-channel)

| Parameter/scenario | Actual |
|---|---|
| Supply voltage/current limit | TBD |
| M2 terminal polarity for positive motion | TBD |
| Encoder sign agreement | TBD |
| EN/DIAG powered-normal state | TBD |
| Low-duty forward/reverse behavior | TBD |
| Unloaded current/current-sense plausibility | TBD |
| Coast/disarm/reset behavior | TBD |

**Owner approval:** TBD

## B16 — Raised Two-Channel Test

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B16](../docs/assembly.md#b16--raised-two-channel-test)

| Scenario | Voltage/current | Encoder/current observations | Reset/fault/noise result |
|---|---|---|---|
| M1 only | TBD | TBD | TBD |
| M2 only | TBD | TBD | TBD |
| Matched forward | TBD | TBD | TBD |
| Matched reverse | TBD | TBD | TBD |
| Deliberately unmatched | TBD | TBD | TBD |
| Simultaneous starts/stops | TBD | TBD | TBD |

**Owner approval:** TBD

## B17 — Two-Channel PI Tuning

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B17](../docs/assembly.md#b17--two-channel-pi-tuning)

| Target | M1 measured/error/current | M2 measured/error/current | Stability |
|---:|---|---|---|
| Low forward | TBD | TBD | TBD |
| Medium forward | TBD | TBD | TBD |
| Low reverse | TBD | TBD | TBD |
| Medium reverse | TBD | TBD | TBD |

**Accepted shared/per-channel parameters and rationale:** TBD  
**Owner approval:** TBD

## B18 — Low-Speed Floor Commissioning

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B18](../docs/assembly.md#b18--low-speed-floor-commissioning)

| Item | Accepted value/observation |
|---|---|
| Lane and tether arrangement | TBD |
| Initial/final test speed | TBD |
| Acceleration limit | TBD |
| Deceleration limit | TBD |
| Near-zero coast threshold | TBD |
| Stall arming command | TBD |
| Startup grace period | TBD |
| Minimum encoder progress | TBD |
| Detection interval | TBD |
| Slip/oscillation/stopping behavior | TBD |

**Owner approval:** TBD

## B19 — Pi-to-Due Integration

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B19](../docs/assembly.md#b19--pi-to-due-integration)

| Scenario | Device/port state | Result |
|---|---|---|
| Pi USB enumeration | TBD | TBD |
| Command/telemetry/CSV | TBD | TBD |
| Pi reboot | TBD | TBD |
| USB disconnect/reconnect | TBD | TBD |
| Due reset | TBD | TBD |
| Firmware upload | TBD | TBD |
| Stale-command check | TBD | TBD |

**Owner approval:** TBD

## B20 — BNO055 Acceptance

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B20](../docs/assembly.md#b20--bno055-acceptance)

| Evidence | Actual |
|---|---|
| Board/cable marking and header state | TBD |
| `#2998` revision, captive-lead length, and selected socket label | TBD |
| Hub external-power jack empty and switch behavior | TBD |
| UART wiring and PS1 strap | TBD |
| Hub/`#954` USB enumeration and UART baud | TBD |
| Hub switch off/on recovery | TBD |
| Self-test/system status | TBD |
| Stationary values | TBD |
| Hand-rotation axis response | TBD |
| Observed update behavior | TBD |
| Proposed mounting orientation | TBD |

**Owner approval:** TBD

## B21 — Camera Acceptance

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B21](../docs/assembly.md#b21--camera-acceptance)

| Evidence | Actual |
|---|---|
| Camera/cable marking | TBD |
| 22-pin Pi / 15-pin camera orientation | TBD |
| Detection result | TBD |
| Streaming result | TBD |
| Intermittent/cable test | TBD |
| Accepted view and mounting orientation | TBD |

**Owner approval:** TBD

## B22 — DROK and Dummy-Load Acceptance

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B22](../docs/assembly.md#b22--drok-and-dummy-load-acceptance)

| Item | Actual |
|---|---|
| Exact DROK marking/variant | TBD |
| Input/output-ground relationship | TBD |
| Input voltage/current limit | TBD |
| No-load harness-end set point | TBD |
| Repeated power-cycle range | TBD |
| Fast-transient measurement limitation | TBD |

| Load configuration | Duration | DROK output | Harness-end voltage | Calculated current | Converter/contact temperature trend |
|---|---:|---:|---:|---:|---|
| 10 Ω only | TBD | TBD | TBD | TBD | TBD |
| One 2 Ω only | TBD | TBD | TBD | TBD | TBD |
| Two 2 Ω parallel; 10 Ω absent | TBD | TBD | TBD | TBD | TBD |

**Adjustment secured/covered:** TBD  
**Owner approval:** TBD

## B23 — Pi Header Power and Combined Load

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B23](../docs/assembly.md#b23--pi-header-power-and-combined-load)

| Scenario | Harness-end voltage | Pi warning/reset | Connection temperature | Due/motor result |
|---|---:|---|---|---|
| Pi boot/NVMe | TBD | TBD | TBD | N/A |
| CPU load | TBD | TBD | TBD | TBD |
| Camera + `#2998`/BNO055 + Due + cooler | TBD | TBD | TBD | TBD |
| Both motors start raised | TBD | TBD | TBD | TBD |
| Both motors run raised | TBD | TBD | TBD | TBD |

**USB-C confirmed absent:** TBD  
**`PSU_MAX_CURRENT=5000` enabled/verified only after pass:** TBD  
**Owner approval:** TBD

## B24 — Final Electronics-Tier CAD

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B24](../docs/assembly.md#b24--final-electronics-tier-cad)

| Requirement | CAD provision/revision |
|---|---|
| Pi/NVMe/cooler airflow | TBD |
| Due USB/reset/service | TBD |
| `#2998` switch, ports, and captive-lead access | TBD |
| Driver terminal access/clearance | TBD |
| DROK access and adjustment cover | TBD |
| Source/motor vs logic routing | TBD |
| Strain relief/tie points | TBD |
| BNO055 orientation | TBD |
| Camera orientation/cable | TBD |
| Deferred battery/LiDAR envelope | TBD |

**Owner approval:** TBD

## B25 — Final Mechanical and Wiring Build

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B25](../docs/assembly.md#b25--final-mechanical-and-wiring-build)

| Inspection | Result |
|---|---|
| Print and mounting quality | TBD |
| ALITOVE internal topology | TBD |
| Fuse holder/fuse continuity | TBD |
| Source/motor branch wire and polarity | TBD |
| DROK/Pi branch wire and polarity | TBD |
| Logic/encoder/direct USB/hub/UART/camera routing | TBD |
| Cross-rail isolation | TBD |
| Individual termination tug tests | TBD |
| Service access/strain relief | TBD |

**Final harness photograph/reference:** TBD  
**Owner approval:** TBD

## B26 — Integrated Raised Regression

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B26](../docs/assembly.md#b26--integrated-raised-regression)

| Scenario | Expected | Actual |
|---|---|---|
| Cold start/startup order | Disarmed/coast | TBD |
| Due reset/upload | No unintended motion | TBD |
| Command loss | Ramp to zero/coast | TBD |
| Simulated driver fault | Both coast, latched | TBD |
| Missing encoder | Both coast, latched | TBD |
| Both speed loops/encoders | Stable and independent | TBD |
| Pi/Due power and USB | No warning/reset | TBD |
| BNO055 and camera | Stable | TBD |
| Motor-induced count/data noise | None observed | TBD |
| Connection temperatures | Stable | TBD |

**Owner approval:** TBD

## B27 — Loaded Thermal Shuttle

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B27](../docs/assembly.md#b27--loaded-thermal-shuttle)

**Final Stage 2 mass:** TBD  
**Lane/tether arrangement:** TBD  
**Commanded speed and cycle:** TBD  

| Elapsed min | Ambient °C | M1 case °C | M2 case °C | Driver 1 area °C | Driver 2 area °C | M1/M2 current | Voltage | Fault/reset/performance |
|---:|---:|---:|---:|---:|---:|---|---:|---|
| 0 | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| 5 | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| 10 | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| 15 | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| 20 | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| 25 | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| 30 | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| 40 if needed | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| 50 if needed | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| 60 if needed | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

**Final 10-minute temperature change:** TBD  
**Stop/acceptance assessment:** TBD  
**Owner approval:** TBD

## B28 — Close the Hardware Gate

**Result:** Pending  
**Date/operator:** TBD  
**Assembly procedure:** [B28](../docs/assembly.md#b28--close-the-hardware-gate)

| Closure item | Accepted revision/result |
|---|---|
| Firmware commit and calibration configuration | TBD |
| Commissioning utility commit | TBD |
| Drivetrain STEP/STL exports | TBD |
| Electronics-tier STEP/STL exports | TBD |
| SolidWorks source location/revision | TBD |
| Final wheel-center separation/radius | TBD |
| Final chassis envelope/mass | TBD |
| URDF update and both xacro expansions | TBD |
| Controller nominal geometry update | TBD |
| ROS package build | TBD |
| Q1–Q11 evidence review | TBD |
| Architecture/matrix evidence-driven changes | TBD |
| B1–B28 approvals complete | TBD |

**Final hardware-gate approval:** TBD
