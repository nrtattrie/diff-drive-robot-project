# Diff-Drive Robot Project

A ROS 2 autonomous mobile robot project built in stages: differential-drive
navigation first, followed by a separately validated self-balancing controls
phase. The project emphasizes simulation, embedded motor control, encoder
odometry, sensor fusion, SLAM, Nav2, and disciplined hardware bring-up.

## Project Status

**Phase 1 — Foundation**

- Stage 1 software simulation is complete.
- The URDF, tf2 model, Gazebo Harmonic `DiffDrive`, and
  `ros2_control + diff_drive_controller` paths have been live-verified.
- Stage 2A hardware architecture is complete; Stage 2B now builds and validates
  the robot one subsystem at a time.
- A modular drivetrain deck follows one-channel acceptance. Final
  electronics-tier CAD remains gated on the two-channel, peripheral, and
  Pi-power checks.
- LiDAR and battery procurement remain deferred until the navigation hardware
  stage.

The historical hardware plan contained invalid assumptions, including treating
one Pololu `#2991` as a dual motor driver. Do not build from archived summaries
or the invalidated power diagram. See the
[component selection audit](hardware/component-selection-audit.md) and
[phase plan](docs/phases.md).

## Current Platform

### Compute and control

- Raspberry Pi 5, 8 GB
- Ubuntu 24.04 Desktop
- ROS 2 Jazzy
- NVMe boot through Raspberry Pi M.2 HAT+
- Arduino Due as the planned real-time motor/encoder controller
- Arduino Mega 2560 retained as a backup/prototyping controller

### Owned actuation and sensing

- 2× Pololu 50:1 37D 12 V gearmotors with 64 CPR encoders (`#4753`)
- 80 mm Pololu wheels and 6 mm mounting hubs
- Pololu Dual VNH5019 Motor Driver Shield (`#2507`) received for the two motor
  channels; physical identity was owner-accepted at B1, while assembly and
  powered bench acceptance remain pending
- 1× owned Pololu G2 High-Power Motor Driver 18v17 (`#2991`), reserved for
  later one-channel use rather than combined with the selected driver
- Adafruit BNO055 IMU
- Adafruit `#2998` bus-powered USB 2.0 hub, assigned to the BNO055/`#954`
  branch with the Due kept on a direct Pi port
- Raspberry Pi Camera Module 3
- HC-SR04 and other kit sensors for supplementary experiments
- RPLidar A1-class LiDAR planned for the autonomous-navigation stage

### Hardware constraints under validation

- Use commercial off-the-shelf boards, connectors, and wire. User soldering of
  supplied headers and ordinary wiring is acceptable; custom/professional
  assembly is outside the hobbyist project scope. Crimping is not assumed, so
  prefer solder, screw-terminal, lever-clamp, or pre-terminated connections as
  appropriate to the circuit.
- The encoders will use a regulated 5 V rail and one socketed Adafruit
  `74LVC245` product `#735` to translate their four outputs to the Due's 3.3 V
  hardware quadrature inputs; physical assembly and count-integrity bench
  acceptance remain pending.
- The Due will receive both power and serial through its Programming USB port,
  normally from the Pi, and will supply encoder 5 V and translator/driver-logic
  3.3 V. Bench current and reset behavior remain to be validated.
- Mobile Raspberry Pi power will use a protected 3S branch through the owned
  DROK set to 5.10 V and into the Pi 5 V/ground header; regulation, thermal,
  startup, and combined-load acceptance remain pending. Q8 selects one 10 A
  main fuse without additional branch-protection hardware. Q9 selects 16 AWG
  battery-side wiring, the 14 AWG inline fuse holder, continuity-checked
  ALITOVE distribution, and four short parallel Elegoo leads for Pi power;
  owned JST/STEMMA or Elegoo leads cover low-current wiring downstream of the
  Pi. The completed Q10 matrix fixes the final connection and pin allocation,
  while Stage 2B must reject any connection with excessive drop, heating,
  looseness, or reset behavior.
- The owned motors and 80 mm wheels are required Phase 2 reuse while plausible;
  preliminary dynamic screening supports that plausibility. All other Phase 2
  properties and reuse decisions remain intentionally deferred.

## Software Stack

- ROS 2 Jazzy on Ubuntu 24.04
- Gazebo Harmonic
- `ros2_control` and `diff_drive_controller`
- Nav2 and `slam_toolbox` planned for autonomous navigation
- `robot_localization` planned for encoder/IMU fusion
- Python for ROS integration and C++/Arduino code where real-time performance
  matters

## Repository Structure

```text
diff-drive-robot-project/
├── docs/                   Project architecture, phases, guides, and audits
├── hardware/               BOM, CAD exports, and hardware references
├── media/                  Images and other media
├── scripts/                Development and launch helpers
└── software/
    ├── arduino/            Embedded firmware
    └── ros2_ws/            ROS 2 workspace
```

## Build the ROS 2 Workspace

```bash
cd software/ros2_ws
colcon build
source install/setup.bash
```

For the verified simulation workflows, follow the
[Gazebo simulation quickstart](docs/gazebo_sim_quickstart.md).

## Roadmap

1. Execute the ordered Stage 2B assembly gates: Due and encoders, one motor
   channel, modular drivetrain, then both channels.
2. Validate the Pi/Due link, BNO055, camera, DROK, and complete Pi-power path.
3. Build the final electronics tier and tethered harness, then close the
   hardware gate with integrated and loaded thermal evidence.
4. Drive the real differential-drive robot and calibrate odometry.
5. Add LiDAR, SLAM, Nav2, and IMU fusion.
6. Model and validate the separate self-balancing conversion before committing
   Phase 2 hardware.

## Documentation

- [Architecture](docs/architecture.md)
- [Phase plan and task tracker](docs/phases.md)
- [Component selection audit](hardware/component-selection-audit.md)
- [Stage 2 hardware decision record](hardware/stage-2-hardware-decision-record.md)
- [Phase 1 power and signal interface matrix](hardware/power-signal-interface-matrix.md)
- [Stage 2 validation log](hardware/stage-2-validation-log.md)
- [Phase 2 preliminary dynamic model](hardware/phase-2-dynamic-model.md)
- [Gazebo simulation quickstart](docs/gazebo_sim_quickstart.md)
- [Assembly guide](docs/assembly.md)
- [Hardware BOM PDF](hardware/Bill%20of%20Materials%20-%20Sheet%20REV%2002.pdf)

## License

MIT License. See [LICENSE](LICENSE).
