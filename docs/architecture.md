# Robot Architecture

## Directive for AI Assistants

This document is the canonical record of architectural decisions, technical
constraints, and design rationale. See [the phase plan](phases.md) for task
status, [the Stage 2 decision record](../hardware/stage-2-hardware-decision-record.md)
for selection evidence, and
[the interface matrix](../hardware/power-signal-interface-matrix.md) for exact
power, pin, connector, wire, threshold, and safe-state definitions.

Use this document before proposing or implementing changes. Do not modify it
unless explicitly asked. Do not treat archived summaries as current design
authority.

**Architecture status as of 2026-07-23:** Stage 2A is complete. The architecture
below is the selected design for the incremental Stage 2B build and
commissioning sequence. It is not yet a bench-validated final build. If a
statement here conflicts with the interface matrix, use the matrix for
connection-level details and resolve the discrepancy before wiring.

## Phase 1 System Overview

Phase 1 is an indoor differential-drive robot intended to progress from
tethered driving to encoder odometry, SLAM, and Nav2 autonomous navigation.

```text
Nav2 / teleop / ROS 2
          |
          v
   Raspberry Pi 5
      |       |---------------- Camera Module 3 (CSI-2)
      |       |---------------- #2998 hub -- BNO055 (USB-to-UART)
      |       `---------------- future RPLidar A1M8 (USB)
      |
      `--- USB serial + power ---> Arduino Due
                                      |
                         +------------+------------+
                         |                         |
                         v                         v
                  Dual VNH5019               encoder inputs
                    #2507                  through 74LVC245
                         |                         ^
                         v                         |
                    two #4753 motors --------------+
```

The Pi owns ROS 2, navigation, perception, and high-level velocity commands.
The Due owns real-time motor output, encoder counting, command-loss handling,
and motor-fault response. The Pi and Due communicate over one data-capable USB
connection to the Due Programming port.

## Selected Hardware Baseline

| Component | Selected role | Stage 2A state |
|---|---|---|
| Raspberry Pi 5, 8 GB | Primary computer running Ubuntu 24.04 and ROS 2 Jazzy | Owned; operating |
| 1 TB NVMe SSD and M.2 HAT+ | Pi boot and project storage | Owned; operating |
| Raspberry Pi Active Cooler | Pi thermal management through the dedicated fan connector | Owned; operating |
| Arduino Due | Real-time motor controller and hardware quadrature counter | Owned; bench integration pending |
| Pololu Dual VNH5019 shield `#2507` | Two-channel motor driver, separately mounted rather than stacked as a shield | Received; identity owner-accepted at B1; assembly/bench validation pending |
| 2x Pololu `#4753` 12 V, 50:1 gearmotors | Left and right Phase 1 drive, with integrated encoders | Owned |
| 80 mm Pololu wheels and 6 mm hubs | Drive wheels | Owned |
| Adafruit `74LVC245` `#735` and 20-pin socket | Four-channel 5 V-to-3.3 V encoder translation | Received; identity owner-accepted at B1; assembly/validation pending |
| Adafruit BNO055 `#4646` | IMU for later odometry fusion | Owned; UART validation pending |
| Adafruit USB Mini Hub with Power Switch `#2998` | Bus-powered expansion for the low-current BNO055/`#954` branch; three downstream sockets remain spare | Owned; integration validation pending |
| Raspberry Pi Camera Module 3 | Forward camera | Owned with Pi 5-compatible cable |
| DROK adjustable buck converter | Mobile Pi 5 V supply | Owned; exact-unit and loaded validation pending |
| Official Raspberry Pi 27 W USB-C supply | Bench/tethered Pi supply | Owned |
| RPLidar A1M8 development kit | Planned Phase 1 SLAM/Nav2 sensor | Deferred to Stage 4 |
| 3S battery, charger, and mating connector | Future mobile energy source | Deferred to Stage 4 |
| Pololu G2 18v17 `#2991` | Reserved one-channel driver; not part of the Phase 1 baseline | Owned |
| Arduino Mega 2560 | Backup/prototyping MCU only | Owned |

[BOM REV 02](../hardware/Bill%20of%20Materials%20-%20Sheet%20REV%2002.pdf)
is the accepted procurement snapshot. Procurement status does not constitute
bench validation.

## Power Architecture

### Selected topology

```text
future 3S battery (11.1 V nominal, 12.6 V full)
  |
  +-- keyed battery connector (manual disconnect)
  |
  +-- 10 A ATO/ATC fuse near battery positive
        |
        `-- ALITOVE distribution
              |
              +-- 16 AWG --> #2507 VIN/GND --> left/right motors
              |
              `-- 16 AWG --> DROK set to 5.10 V --> Pi 5 header
                                                        |
                                                        `-- USB --> Due
                                                                     |
                                                                     +-- 5 V --> both encoders
                                                                     `-- 3.3 V --> 74LVC245 and #2507 VDD
```

### Bench and mobile modes

- **Bench/tethered:** use the official 27 W USB-C supply for the Pi. The Pi
  powers and communicates with the Due over USB. A current-limited bench
  supply powers the motor-driver branch during staged testing.
- **Combined-path validation:** after dummy-load acceptance and before mobile
  use, the current-limited bench supply substitutes for the future 3S battery
  at the protected distribution input. In that controlled test only, the same
  distribution feeds `#2507` and the DROK/Pi header path so startup interaction
  and reset behavior can be measured without selecting the battery.
- **Mobile:** the protected 3S branch feeds the owned DROK. Set and verify the
  DROK at `5.10 V` before connecting the Pi, then feed Pi physical pins 2 and 4
  with two short, equal positive leads and grounds 6 and 14 with two short,
  equal return leads.
- Never connect Pi USB-C power and header power simultaneously.
- The Due receives power and serial only through its Programming USB port.
  Do not power it through `VIN`, the barrel jack, or its 5 V/3.3 V pins.
- Connect the Due to only one powered USB host at a time.

The mobile Pi path must sustain 5 A at the Pi header without excessive voltage
drop, heating, startup overshoot, undervoltage indication, or resets.
`PSU_MAX_CURRENT=5000` may be enabled only after that complete path passes
Stage 2B. The future RPLidar's conservative USB startup allocation makes this
validation a prerequisite for mobile LiDAR use.

### Protection boundary

- Use one 10 A main fuse; do not add branch fuses or a fused distribution
  board to the current design.
- Use the future battery's keyed/polarized connector as the normal manual
  disconnect. The owned 1 A toggle is not in the battery-current path.
- Do not add a separate emergency-stop switch, reverse-polarity board, or
  extra `#2507` bulk capacitor.
- The `#2507` supplies reverse-voltage, thermal, over/undervoltage, and
  output-short protection for its motor branch.
- Battery-specific undervoltage protection or monitoring remains deferred with
  the exact battery selection and is required before Stage 4 mobile operation.

## Motor Drive and Safe-State Policy

The `#2507` is used as a general-purpose board, not an Arduino shield. Solder
the three included 2-pin terminal blocks and a 13-pin logic header. Do not
install the Arduino stackable headers or the `ARDVIN=VOUT` jumper.

| Concern | Selected behavior |
|---|---|
| Motor channels | `M1` left motor; `M2` right motor; final polarity established unloaded |
| Logic supply | Due 3.3 V to driver `VDD`; common logic ground |
| PWM | Due hardware PWM at 20 kHz |
| Voltage limit | Initial 95% duty ceiling on a 12.6 V full 3S source, about 12.0 V average |
| Continuous-current target | Approximately 1.38 A per motor |
| Transient reference | 5.5 A is theoretical motor stall, not a permitted operating state |
| Normal stop | Bounded deceleration to zero wheel speed, then coast near standstill |
| Command loss | Recoverable 500 ms Due-side watchdog; zero target followed by coast |
| Driver fault | Either channel fault coasts both channels and latches motion off |
| Encoder stall | Calibrated lack of encoder progress coasts both and latches motion off |
| Reset state | Driver PWM pull-downs keep both outputs high-impedance; firmware establishes PWM LOW before accepting motion |
| Current sense | `M1CS/M2CS` directly to Due A0/A1 through the board's existing 10 kΩ resistors; no external clamp |

Current sensing is telemetry and diagnostic evidence, not the sole stall
trigger. Stage 2B must calibrate acceleration, near-zero coast transition,
stall-detection timing, ADC scaling, and loaded thermal behavior.

## Encoder Architecture

Both `#4753` encoders are powered from the Due's regulated 5 V output. Each
encoder can draw up to 10 mA and produces 0-5 V A/B signals, which must never
connect directly to the Due.

One socketed Adafruit `74LVC245` runs from Due 3.3 V and translates all four
encoder signals from its A side to its B side:

| Encoder signal | Translator | Due hardware input |
|---|---|---|
| Left A, yellow | A1 to B1 | D2 / `TIOA0` |
| Left B, white | A2 to B2 | D13 / `TIOB0` |
| Right A, yellow | A3 to B3 | D5 / `TIOA6` |
| Right B, white | A4 to B4 | D4 / `TIOB6` |

Set `DIR` high, `/OE` low, tie unused A5-A8 inputs to ground, leave unused
B5-B8 outputs open, and place one owned `104`/0.1 uF ceramic capacitor directly
between IC VCC and ground. The
[interface matrix](../hardware/power-signal-interface-matrix.md#74lvc245-fixed-wiring)
contains the complete 20-pin wiring.

## Arduino Due Pin Allocation

| Due resource | Assignment |
|---|---|
| D2, D13 | Left encoder A/B hardware quadrature pair |
| D5, D4 | Right encoder A/B hardware quadrature pair |
| D6, D7 | M1/M2 20 kHz hardware PWM |
| D8, D9 | M1 INA/INB direction |
| D10, D11 | M2 INA/INB direction |
| D22, D23 | M1/M2 EN/DIAG, configured as inputs only |
| A0, A1 | M1/M2 current sense through onboard 10 kΩ resistors |
| Programming USB | Pi serial communication and Due power |

The stock shield mapping and stock library pin assumptions are not
authoritative. Firmware must implement this allocation.

## Sensor and Peripheral Interfaces

| Device | Connection | Power and status |
|---|---|---|
| BNO055 | One black Pi USB 2.0 port through the bus-powered `#2998` hub to the owned Adafruit USB-to-TTL cable `#954`; BNO SDA/TX to cable white/RX, cable green/TX to BNO SCL/RX; BNO `3VO` strapped to `PS1` | Cable red 5 V to `VIN`, black to ground; keep hub switch on and external-power jack unused; owned, bench validation pending |
| Camera Module 3 | Pi CAM/DISP using the owned Standard-to-Mini cable: narrow 22-pin Pi end and wider 15-pin camera end | Pi-managed; owned |
| RPLidar A1M8 | Future development-kit USB adapter directly to a Pi USB port | Deferred purchase; include up to about 0.45 A working and 0.70 A startup in USB budget |

The owned INA219, APDS9960, HC-SR04, white terminal strips, and 1 A toggle have
no installed baseline function. Adding any of them requires
a defined interface and matrix revision. The HC-SR04 is not required for
Phase 1 navigation and its 5 V Echo output must not be connected directly to a
3.3 V input.

## Wiring, Grounding, and Assembly

- Use 16 AWG stranded copper for the battery trunk and both battery-side
  branches.
- Use the selected 14 AWG inline ATO/ATC holder with a 10 A fuse.
- Use the ALITOVE connectors only after continuity confirms the required
  internal topology. Tug-test every termination and reject any heating or
  loosening during Stage 2B.
- Land the battery feed and motor factory red/black leads directly in the
  `#2507` terminal blocks. Do not use JST-SH, STEMMA QT, Dupont, a breadboard,
  or the white terminal strips in a motor or battery-current path.
- The accepted Pi-power implementation uses two short, equal Elegoo positive
  leads and two short, equal ground leads, each no longer than about 12 inches.
  Their undocumented gauge/contact rating is an explicit Stage 2B rejection
  criterion.
- Use owned JST/STEMMA cables or Elegoo jumpers for low-current power and
  signals downstream of the Pi.
- Battery negative distributes separately to the driver power ground and DROK
  input ground. Motor return current must not pass through Pi, Due, encoder,
  translator, USB, or jumper-wire ground paths.
- Due, driver logic, translator, and both encoders share a logic reference.

The project uses commercial off-the-shelf products assembled by the user.
Soldering supplied headers and ordinary wiring is acceptable. Hired custom
assembly is outside scope, and crimping is not assumed.

## Software Architecture

- **OS/ROS:** Ubuntu 24.04 Desktop and ROS 2 Jazzy on the Pi 5.
- **Navigation:** Nav2 with `slam_toolbox`.
- **Drive integration:** `ros2_control` and `diff_drive_controller`; current
  configuration is validated in simulation, while physical hardware transport
  remains to be implemented.
- **Pi-to-Due transport:** custom 115200-baud, 8N1, newline-delimited ASCII
  protocol through `pyserial`, not micro-ROS. Commands are `ARM`, `DISARM`,
  `SET <left_cps> <right_cps>`, `STOP`, `RESET_FAULT`, and `STATUS`.
  Telemetry carries time, encoder totals/rates, current estimates, and
  state/fault flags. Parsing is length-, shape-, and range-checked; no
  application checksum or sequence layer is selected unless evidence shows it
  is needed over the ordered USB link.
- **Embedded wheel control:** the Due runs the Phase 1 wheel PI loops at
  200 Hz and emits commissioning telemetry at 20 Hz. A valid `SET` refreshes
  the recoverable 500 ms watchdog; driver and encoder-stall faults remain
  latched until an explicit safe reset.
- **Odometry:** wheel encoders first; later combine encoder odometry and BNO055
  IMU data through `robot_localization`.
- **Languages:** Python for ROS integration; C++/Arduino code where real-time
  behavior or performance requires it.

### Build commands

```bash
cd software/ros2_ws
colcon build
colcon build --packages-select <pkg>
source install/setup.bash
colcon test
colcon test --packages-select <pkg>
```

## Development Environment

- VS Code Remote SSH from the MacBook to the Pi.
- CAD on the MacBook in SolidWorks; 3D printing on the Bambu X1-C.
- CAD source files remain in Google Drive; repository hardware files are
  exported STL/STEP artifacts.
- Pi boots from the NVMe SSD and hosts the ROS 2 workspace.
- The Pi's desktop supports RViz, Gazebo, and `rqt`; Pi simulation should
  remain lightweight.

## Project Structure

- `docs/` — architecture, phases, guides, and historical session records.
- `hardware/` — current BOM, decision evidence, interface matrix, CAD exports,
  and datasheets.
- `software/ros2_ws/` — ROS 2 workspace and packages.
- `software/arduino/diffbot_due/` — evolving Due firmware.
- `software/arduino/tools/` — non-ROS commissioning utility.
- `scripts/` — project utilities.

## Phase 2 Reuse Boundary

The owned `#4753` motors, their encoders, and the 80 mm wheels/hubs are required
Phase 2 reuse while they remain plausible. The
[preliminary dynamic model](../hardware/phase-2-dynamic-model.md) found no
reason to replace them.

No other Phase 1 component, Phase 2 physical property, chassis decision,
controller choice, sensor, or power decision is assumed to transfer. Phase 2
self-balancing remains a separate later design exercise and does not constrain
unfinished Phase 1 choices.

## Stage 2B Build and Validation Boundary

Stage 2A selected a coherent architecture; it did not prove the actual
hardware. Stage 2B now combines construction, firmware, CAD, and validation in
one downstream sequence. The [phase plan](phases.md) owns its 28 approval
gates, the [assembly guide](assembly.md) owns the reusable procedure, and the
[validation log](../hardware/stage-2-validation-log.md) owns measured evidence.

Stage 2B must:

1. accept the physical inventory, current-limited supply, dummy-load fixture,
   and measurement boundary before powered assembly;
2. bring up the Due, serial safety state machine, 20 kHz PWM, translator, and
   both encoders before motor power;
3. validate one unloaded motor/encoder channel, closed-loop PI control, reset
   safety, the 500 ms watchdog, and non-destructive fault/stall responses;
4. permit a modular drivetrain deck only after the one-channel gate, then
   validate both mounted channels and low-speed tethered motion;
5. validate Pi/Due USB, BNO055, camera, the DROK, and the complete 5 A Pi
   header path before final electronics-tier CAD;
6. build only the accepted final tethered harness and mounting layout, then
   pass integrated raised and loaded thermal regressions; and
7. synchronize calibrated firmware, CAD exports, URDF/controller nominal
   geometry, architecture, matrix, and phase evidence before closing the
   hardware gate.

No gate may claim measurements outside the available instrument bandwidth.
In particular, the Due D3 PWM loopback is an internal commissioning check, and
ordinary multimeter readings do not establish high-speed transient behavior.

## Deferred Decisions

- Exact 3S battery chemistry/capacity, charger, keyed connector SKU, and
  undervoltage method: Stage 4.
- RPLidar A1M8 purchase: Stage 4.
- Battery/LiDAR mounting details remain provisional envelopes in Stage 2B;
  their exact mounting is completed with those deferred components in Stage 4.
- Any optional HC-SR04, APDS9960, or INA219 integration: only if a demonstrated
  need appears. The owned `#2998` hub is now assigned only to the BNO055 branch.
- Phase 2 mass distribution, center of mass, inertia, controller, IMU
  suitability, power system, and chassis: Phase 2 work, not Phase 1 scope.

## Explicitly Superseded Claims

The following statements in archived summaries are invalid and must not be
reintroduced:

- one Pololu `#2991` is a dual motor driver;
- the `#4753` encoders can be powered from Due 3.3 V;
- a 5.1 V supply is valid for Due `VIN`;
- the Due or Pi should be powered from the `#2507`;
- thin JST-SH/STEMMA/Dupont leads are acceptable for motor or battery power;
- the owned 1 A toggle is the robot's battery switch; or
- a Phase 1 hardware choice is automatically proven suitable for the unbuilt
  Phase 2 robot.
