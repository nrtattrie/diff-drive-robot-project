**DIRECTIVE FOR AI ASSISTANTS:** This document tracks phase plans and current task status. It does NOT contain architectural decisions — see `architecture.md` for hardware, software stack, and key decisions. Work only one unchecked item at a time and stop for approval after each. Use checkboxes to track progress. If this document conflicts with a user request, flag it rather than silently overriding.

**Doc hierarchy:** `architecture.md` = canonical technical decisions. `phases.md` = canonical phase plan and task tracking. `docs/sessions/` = historical record of past sessions (do not update).

## Project Overview

Differential drive mobile robot (Phase 1) followed by a self-balancing controls
phase (Phase 2). The owned motors and 80 mm wheels/hubs are required reuse while
plausible; Q2 makes no full-platform transfer assumption.

**Current status:** Phase 1 — Foundation. Stage 1 complete. Stage 2 in progress.

---

## Phase 1: Foundation — Differential Drive Robot (Months 1–4)

**Goal:** Autonomous mobile robot with Nav2 + SLAM

**Skills:** ROS 2 architecture, SLAM (slam_toolbox), Nav2, odometry + sensor fusion, motor control (PID), hardware-software integration

### Stage 0: Hardware Procurement

- [x] Inventory existing hardware — RPi 5 (8GB), M.2 HAT+, cooler, Arduino Due, Arduino Mega 2560, Elegoo kit sensors/components
- [x] Order RPi accessories — 27W USB-C PSU, Camera Module 3, USB Mini Hub (Adafruit)
- [x] Original motors purchased — Pololu 37D 50:1 with encoder (#4753 x2); Phase 1 use and preliminary Phase 2 reuse remain plausible
- [x] Original motor driver purchased — one Pololu G2 High-Power 18v17 (#2991); the historical dual-channel assumption is invalid
- [x] IMU selected — Adafruit BNO055 9-DOF STEMMA QT (#4646)
- [x] MCU decision — Arduino Due as primary (84 MHz ARM, 3.3V native, hardware quadrature encoder); Mega 2560 as backup
- [x] Original power components selected — historical architecture is now invalidated and must pass the Stage 2 hardware gate
- [x] Original procurement BOM completed — Pololu $227.06 + Adafruit $96.59 + Amazon $39.46 = **$363.11**; not a validated final-system BOM
- [x] All orders placed and received — motors, driver, wheels, hubs, caster, JST cables, IMU, GPIO headers, STEMMA QT cables, INA219, APDS9960, toggle switch, USB-TTL cable, buck converter, threadlocker, terminal blocks, lever connectors
- [ ] LiDAR purchase — RPLidar A1M8 (~$100) — deferred to Stage 4
- [ ] Battery purchase — 3S LiPo 11.1V 2200–3000mAh + charger (~$50) + XT60 connectors (~$3) — deferred to Stage 4

### Stage 1: Software-Only (no hardware needed)

- [x] URDF — model robot geometry (chassis, wheels, caster), visualize via Foxglove
- [x] tf2 — understand coordinate frames (`base_link → odom → map`)
- [x] Gazebo Harmonic simulation — spawn URDF, drive with `teleop_twist_keyboard`
  - `gazebo.launch.py` uses Gazebo's built-in `DiffDrive` plugin as the lightweight baseline.
  - Command path: `teleop_twist_keyboard -> ros_gz_bridge -> Gazebo DiffDrive -> Gazebo odometry`.
- [x] `ros2_control + diff_drive_controller` — configured and live-verified in simulation
  - `gazebo_ros2_control.launch.py` starts `gz_ros2_control`, the controller spawners, and Foxglove support.
  - `twist_to_twist_stamped.py` bridges teleop's `Twist` commands to the controller's `TwistStamped` input.

### Stage 2: Hardware Architecture + Validation + Chassis + Arduino

**Gate:** Do not build from the historical power diagram. Complete Stage 2A
before powered assembly, then follow the Stage 2B gates in order. A modular
drivetrain deck may be built only after one motor/encoder channel passes; do
not freeze the final electronics layout until the downstream power and
peripheral gates pass. See `hardware/component-selection-audit.md`.

#### Stage 2A: Resolve Hardware Questions in Order

Work through these questions sequentially. A purchase is not a completed answer;
record the manufacturer evidence, selected implementation, required connectors,
and bench acceptance test for each decision.

Use `hardware/stage-2-hardware-decision-record.md` for the current evidence,
candidate feedback, and provisional recommendations. Update that record as each
question is resolved; keep this file as the authoritative ordered checklist.

- [x] Audit motor/driver/IMU/wheel selection history — supplied chat and v1-v7 archives reviewed; motor-SKU mismatch explained and BNO055 ROS 2 rationale independently checked
- [x] **Q1 — Assembly boundary and physical inventory:** use commercial off-the-shelf products assembled by the user; soldering supplied headers and ordinary wiring is acceptable, hired/custom assembly is out of scope, and crimping is not assumed. The owned wiring inventory is classified in the decision record.
  - [x] Owned `#2991` connector state confirmed — headers are included but loose; they can now be user-soldered, although the one-channel board remains a poor two-motor solution
  - [x] Assembly boundary confirmed — hobbyist assembly of off-the-shelf parts is acceptable; professional third-party or custom factory assembly is not
  - [x] Soldering capability added — FNIRSI HS-02A toolbox kit with 100 W USB-C power block purchased
  - [x] Wiring/connector inventory completed — owned JST-SH/STEMMA and jumper
    leads cover signal and sensor power and the factory motor leads are usable;
    Q9 later assigned the owned ALITOVE splices and 16 AWG battery-side wire,
    which BOM REV 02 now records as owned
- [x] **Q2 — Phase 2 reuse boundary:** preliminary [wheeled-inverted-pendulum screening](../hardware/phase-2-dynamic-model.md) supports the plausibility of the owned `#4753` motors and 80 mm wheels/hubs. Nathan requires those components to be reused while they remain plausible, so no replacement motors or wheels are planned. All other Phase 2 properties and reuse decisions are intentionally deferred; Q2 does not select or size the Q3 motor driver.
- [x] **Q3 — Two-channel motor drive:** Pololu Dual VNH5019 Motor Driver Shield
  `#2507` selected and ordered on 2026-07-23, received, and owner-accepted at
  B1. Install it as a separately mounted
  general-purpose board with three terminal blocks and a 13-pin logic header
  user-soldered; use 3.3 V `VDD`, connect the board's 10 kΩ-isolated
  current-sense outputs to Due A0/A1, use the Q4 20 kHz PWM/current-sense
  policy, provide system-level timeout/stall/fuse protection, and keep all
  thin owned wire out of motor power. The complete five-condition
  implementation boundary is recorded in the
  [decision record](../hardware/stage-2-hardware-decision-record.md#mandatory-implementation-conditions).
- [x] **Q4 — Motor protection and control policy:** for indoor, substantially
  level Phase 1 operation, use a 1.38 A-per-motor continuous design target,
  20 kHz hardware PWM with an initial 95% duty cap on a 12.6 V full 3S source,
  controlled deceleration followed by coast, a recoverable 500 ms command
  watchdog, and latched coast responses for driver faults and encoder-detected
  stalls. Do not impose an unsupported current-time trip curve or fixed
  temperature limits; Stage 2B calibrates the acceleration/stall parameters and
  validates loaded current and thermal stability. The complete policy and
  reference comparison are in the
  [decision record](../hardware/stage-2-hardware-decision-record.md#q4-resolution-motor-protection-and-control-policy).
- [x] **Q5 — Encoder interface:** power both `#4753` encoders from a regulated
  5 V rail and translate their four outputs through one socketed Adafruit
  `74LVC245` product `#735`. The received `#735` and `#2204` socket were
  owner-accepted at B1. Use the Due's D2/D13 and
  D5/D4 hardware quadrature-decoder pairs; keep the separately mounted `#2507` off its
  conflicting default shield pins. Q6 supplies the final mobile 5 V source,
  Q9 selects the physical cables, and Q10 fixes the complete pin matrix.
  Stage 2B verifies levels and count integrity through maximum allowed speed.
- [x] **Q6 — Arduino Due power:** use one data-capable connection to the Due
  Programming USB port for both power and serial—normally from a Raspberry Pi
  USB host port, or from one development computer during isolated bench work.
  The Due supplies the encoder 5 V rail and the translator/driver-logic 3.3 V
  rail. Do not power through `VIN`, the barrel jack, or the 5 V/3.3 V pins, and
  do not connect two powered USB hosts simultaneously. The Q7 mobile Pi power
  budget includes this branch, subject to Stage 2B load validation.
- [x] **Q7 — Raspberry Pi power:** use the official 27 W USB-C supply for
  bench/tethered work. For mobile use, feed a Q8-protected 3S branch through
  the owned DROK set to 5.10 V and into the Pi 5 V/ground header; the Pi then
  powers the Due over USB. Do not add a Pi-specific power board or connect
  USB-C and header power simultaneously. Enable `PSU_MAX_CURRENT=5000` only
  after Stage 2B proves the complete path can sustain 5 A with stable voltage,
  safe startup, acceptable temperature, and no resets under combined loads.
- [x] **Q8 — Power protection policy:** use one 10 A standard ATO/ATC fuse in
  the battery-positive feed. Use no branch fuses, fused distribution block,
  separate emergency-stop switch, added reverse-polarity board, or additional
  driver capacitor. The keyed battery connector will be the manual power
  disconnect; Q9 selects its physical implementation. Battery-specific
  undervoltage protection or monitoring is deferred with the battery purchase
  and must be resolved before Stage 4 mobile operation. The owned 1 A toggle is
  not part of the battery-current path.
- [x] **Q9 — Physical power distribution and owned-BOM gap check:** use 16 AWG
  copper for the battery trunk and fused battery-side branches, the selected
  14 AWG inline ATO/ATC holder with the Q8 10 A fuse, the battery's eventual
  keyed/polarized connector as the disconnect, and the owned ALITOVE splices
  where their continuity-checked topology supports the required junction.
  Feed the Pi header from the DROK with two short Elegoo positive jumpers and
  two short Elegoo ground jumpers; use owned JST/STEMMA cables or Elegoo
  jumpers for the low-current power and signals downstream of the Pi. The
  `#2507` terminal blocks and motor factory leads complete the motor
  terminations. Q9 is closed by owner direction; gauge/contact uncertainty,
  voltage drop, tug security, heating, and full-load operation remain explicit
  Stage 2B acceptance checks rather than reasons to leave the selection open.
- [x] **Q10 — Power/signal interface matrix:** the
  [authoritative matrix](../hardware/power-signal-interface-matrix.md) records
  voltage, normal/transient current, signal direction, logic threshold,
  channel count, connector/wire, protection, shutdown state, and controlling
  device for every baseline interface. It fixes the conflict-free Due pin
  allocation, direct 10 kΩ-isolated current-sense path, BNO055 UART path,
  owned camera connection, and planned RPLidar USB/power budget. It adds no
  new procurement item. Bench evidence remains in Stage 2B.
- [x] **Q11 — Update procurement records:** Nathan accepted
  [BOM REV 02](../hardware/Bill%20of%20Materials%20-%20Sheet%20REV%2002.pdf)
  as sufficient on 2026-07-23. It adds the current Due, `#2507`, encoder
  interface, wire, and fuse purchases; the prior PDF is preserved in
  `hardware/_archive`. No further BOM cleanup is required for Stage 2A.

#### Stage 2B: Incremental Bench Build and Integration

Use the [assembly guide](assembly.md) for the procedure and
[Stage 2 validation log](../hardware/stage-2-validation-log.md) for measured
evidence. Work one unchecked gate at a time and stop for owner approval after
each. A gate passes only when its assembly-guide acceptance criteria and log
entry are complete.

##### Bench, Due, and encoder path

- [x] **B1 — Inventory and evidence baseline:** identify the active,
  non-deferred hardware; record markings, condition, connector state, and
  required assembly
- [x] **B2 — Bench test rig:** accept the NICE-POWER C3010 and multimeter,
  construct the guarded resistor-load fixture, verify basic CV/CC behavior,
  and obtain repeatable non-contact temperature measurement
- [ ] **B3 — Due toolchain and safe boot:** install the Arduino SAM/Due
  toolchain, compile/upload the firmware skeleton, and verify USB, reset-safe
  coast state, serial output, and 5 V/3.3 V rails
- [ ] **B4 — Serial safety foundation:** implement the strict line-oriented
  command parser, arm/disarm state, telemetry, malformed-input handling, and
  recoverable 500 ms valid-command watchdog with motor power absent
- [ ] **B5 — PWM commissioning:** verify D6 and D7 individually at 20 kHz and
  expected duty through a temporary D3 timer-capture loopback, then remove the
  test connection
- [ ] **B6 — Encoder translator assembly:** lay out and solder the socketed
  `74LVC245` carrier, bypass capacitor, rails, straps, and headers; pass
  unpowered continuity and isolation inspection
- [ ] **B7 — Encoder translator powered test:** apply logic power only and
  verify the 5 V encoder side, 3.3 V translator side, fixed control straps, and
  safe Due-side levels
- [ ] **B8 — Left encoder hand test:** verify sign, channel order, stable
  counts, and 3200 quadrature counts per output-shaft revolution
- [ ] **B9 — Right encoder hand test:** repeat the count test and verify that
  neither encoder creates counts on the other channel

##### Driver and drivetrain

- [ ] **B10 — Motor-driver assembly:** solder the `#2507` terminal blocks and
  13-pin general-purpose header; pass unpowered continuity, polarity,
  terminal-security, and isolation checks
- [ ] **B11 — Motor-driver logic test:** apply logic power without motor power
  and verify PWM-low reset state, direction, EN/DIAG, and current-sense zero
  readings
- [ ] **B12 — First powered motor channel:** bring up M1 unloaded around
  7–8 V with a 0.5–1 A supply limit; establish polarity and validate
  low-duty direction, encoder sign, current-sense plausibility, and coast
- [ ] **B13 — First closed-loop and safety test:** add M1 PI speed control and
  verify reset, upload, USB loss, watchdog recovery, simulated fault, and
  missing-encoder stall response without intentionally stalling the motor
- [ ] **B14 — Modular drivetrain deck:** measure the accepted parts, CAD and
  print the reusable deck, mount both motors/wheels/caster with strain relief,
  and record physical wheel geometry and deck mass
- [ ] **B15 — Second powered motor channel:** validate M2 individually using
  the accepted M1 sequence
- [ ] **B16 — Raised two-channel test:** verify independent direction/speed,
  simultaneous startup, encoder isolation, current telemetry, and absence of
  Due resets with both wheels clear
- [ ] **B17 — Two-channel PI tuning:** tune the shared PI baseline and
  quantify left/right tracking; use channel-specific values only if measured
  mismatch requires them
- [ ] **B18 — Low-speed floor commissioning:** use a short, strain-relieved
  tethered shuttle to tune acceleration, deceleration, near-zero coast, and
  stall parameters through the commissioning interface

##### Pi, peripherals, and power

- [ ] **B19 — Pi-to-Due integration:** with the Pi on its official USB-C
  supply, validate the Programming-USB command/telemetry link, reboot,
  reconnect, reset, and commissioning-log behavior
- [ ] **B20 — BNO055 acceptance:** validate UART wiring and mode strap,
  self-test/status, axis response, and update behavior; leave full navigation
  calibration and fusion deferred
- [ ] **B21 — Camera acceptance:** validate Camera Module 3 detection,
  streaming, cable orientation, and proposed viewing direction
- [ ] **B22 — DROK and dummy-load acceptance:** identify and set the exact
  DROK to 5.10 V, repeat power cycles, and test the complete four-lead Pi
  harness at approximately 0.5 A, 2.55 A, and 5.1 A while recording voltage
  drop and temperature behavior
- [ ] **B23 — Pi header-power and combined-load acceptance:** with USB-C
  disconnected, validate Pi boot/NVMe/CPU/peripheral loads from the DROK, then
  repeat through simultaneous motor startup/run without undervoltage,
  unstable output, connector heating, or resets

##### Final integration and gate closure

- [ ] **B24 — Final electronics-tier CAD:** design the modular tier with
  service access, cooling, terminal clearance, strain relief, defined IMU and
  camera placement, and reserved battery/LiDAR envelopes
- [ ] **B25 — Final mechanical and wiring build:** print/mount the tier and
  build the secured tethered harness from the interface matrix; pass
  continuity, polarity, topology, and tug checks before power
- [ ] **B26 — Integrated raised regression:** validate startup order, resets,
  uploads, watchdog/fault/stall behavior, both speed loops and encoders,
  current telemetry, Pi load, BNO055, camera, and motor-induced noise
- [ ] **B27 — Loaded thermal shuttle:** run the documented final level-floor
  duty cycle at the actual Stage 2 mass; confirm normal loaded current remains
  consistent with the 1.38 A-per-motor target, temperature approaches the
  defined stable trend, and no reset, fault, connection, or performance
  failure occurs
- [ ] **B28 — Close the hardware gate:** transfer accepted calibration,
  export CAD, synchronize URDF/controller nominal geometry with the built
  robot, rerun ROS model/build checks, review Q1–Q11 against the evidence, and
  promote the architecture to bench-validated status

### Stage 3: Real Hardware Driving (tethered power, no battery)

- [ ] Serial bridge ROS 2 node — Python/`pyserial` to translate between ROS 2 topics and Arduino serial
- [ ] Teleop on real robot — keyboard → `/cmd_vel` → motors spin → robot moves
- [ ] Odometry calibration — tune encoder-based odometry, compare commanded vs. actual motion

### Stage 4: Autonomous Navigation (requires LiDAR + battery)

- [ ] Purchase RPLidar A1M8 (~$100) and 3S LiPo 11.1V + charger (~$50) +
  XT60 connectors (~$3); select and verify the battery-specific undervoltage
  protection or monitoring method before mobile operation
- [ ] Mount LiDAR and battery on chassis
- [ ] RPLidar ROS 2 driver — publish laser scans
- [ ] slam_toolbox — drive around, build a map
- [ ] Nav2 — autonomous path planning and navigation in mapped environment
- [ ] IMU fusion — combine BNO055 + wheel odometry via `robot_localization` (EKF)

**Note:** HC-SR04 ultrasonic useful as supplementary close-range obstacle/cliff sensor but cannot replace LiDAR for SLAM/Nav2.

### Immediate Tasks

**Completed:**

- [x] Install VS Code with Remote SSH extension on Mac
- [x] Set up GitHub repo structure
- [x] Flash Ubuntu 24.04 LTS Desktop (64-bit) onto NVMe SSD
- [x] Configure Pi 5 to boot from NVMe
- [x] Install ROS 2 Jazzy
- [x] Configure VS Code Remote SSH workflow + SSH alias ("pi")
- [x] Clone project repo onto Pi
- [x] Initialize ros2_ws with `colcon build`
- [x] Run ROS 2 talker/listener demo successfully
- [x] Complete ROS 2 Jazzy CLI tutorials
- [x] Read ROS 2 Jazzy documentation overview (concepts, architecture)
- [x] Complete first 4 ROS 2 Jazzy tutorials on Pi
- [x] Set up VS Code SSH key authentication (passwordless)
- [x] Create first custom ROS 2 package (`my_package` tutorial, then `diffbot_description`)
- [x] Set up Foxglove Studio remote visualization (foxglove_bridge on Pi, Foxglove app on Mac)
- [x] Complete URDF (chassis, wheels, caster)
- [x] Gazebo Harmonic simulation with built-in `DiffDrive`
- [x] Gazebo Harmonic simulation with `ros2_control + diff_drive_controller`
- [x] `twist_to_twist_stamped.py` — teleop compatibility bridge
- [x] `start_foxglove_display.sh` — static model display helper

---

## Phase 2: Controls & Wow Factor — Self-Balancing Robot (Months 5–8)

**Goal:** Convert diff-drive base to two-wheeled inverted pendulum

**Skills:** Advanced controls (PID/LQR/MPC), system dynamics and modeling, real-time feedback, simulation-to-real deployment

**Why:** Visually impressive, ~1-2% of portfolios have this vs. 30-40% with diff-drive. Uses same ROS 2 infrastructure from Phase 1.

---

## Phase 3: Manipulation — Mobile Manipulator or Robot Arm (Months 9–12)

**Goal:** Vision-guided pick-and-place system

**Skills:** Forward/inverse kinematics, MoveIt2, computer vision (OpenCV, object detection), trajectory planning, force control basics

**Options:**

- A) Standalone 4–6 DOF robot arm with vision-guided grasping
- B) Add gripper to mobile robot for "fetch-and-place" demo

---

## Phase 4: Advanced Features (Month 12+, Optional)

Choose based on target companies:

- Multi-robot coordination
- Reinforcement learning for robotics
- Outdoor autonomy with GPS
- Quadruped robot
