## Directive for AI Assistants

This document is the canonical record of architectural decisions, technical constraints, and design rationale. See `phases.md` for phase plans and task tracking. See `docs/sessions/` for historical session records.

Use this document to understand the intended design before proposing solutions or making changes. Do not modify it unless explicitly asked. Treat decisions recorded here as settled — do not re-litigate or suggest alternatives unless asked to revisit a specific decision. When a question arises that this document answers, cite it rather than reasoning from scratch.

---

## Hardware

- **Compute:** Raspberry Pi 5 (8GB), Ubuntu 24.04 Desktop, ROS 2 Jazzy LTS
- **Motor control:** Arduino Due (3.3V ARM, hardware quadrature encoders); Mega 2560 as backup
- **Sensors:** RPLidar A1, BNO055 IMU, HC-SR04 ultrasonic, RPi Camera Module 3
- **Motors:** 2x Pololu 37D 50:1 with encoders, Pololu G2 18v17 driver
- **Power:** 3S LiPo → DROK buck converter → 5.1V for RPi via GPIO
- **Connection strategy:** No soldering — barrier terminal strips + lever connectors only

## Software Stack

- ROS 2 Jazzy on Ubuntu 24.04 (non-negotiable — Pi 5 requires Noble)
- Nav2 for autonomous navigation, slam_toolbox for mapping
- ros2_control for motor control, odometry fusion (encoders + IMU)
- Python primary, C++ for performance-critical nodes
- Arduino IDE for motor controller firmware

## Build Commands

```bash
cd software/ros2_ws
colcon build                    # build all packages
colcon build --packages-select <pkg>  # build single package
source install/setup.bash       # source workspace
colcon test                     # run all tests
colcon test --packages-select <pkg>   # test single package
```

## Project Structure

- `docs/` — project documentation, BOM, learning log
- `hardware/` — CAD exports (STL/STEP), electronics references, datasheets
- `software/ros2_ws/` — ROS 2 workspace (packages go in `src/`)
- `software/arduino/` — Arduino firmware for motor controller
- `scripts/` — utility scripts

## Development Environment

- VS Code Remote SSH from MacBook to RPi 5
- CAD on MacBook (SolidWorks), 3D printing on Bambu X1-C
- CAD source files stored in Google Drive, only exports (STL/STEP) in repo
- Pi accessible at 192.168.0.99 — SSH alias `ssh pi`
- Pi booting from 1TB NVMe SSD (939GB available, 3-5x faster than SD)
- `.bashrc` auto-sources `/opt/ros/jazzy/setup.bash` and workspace `install/setup.bash`
- Repo cloned to `~/diff-drive-robot-project` on Pi
- Desktop GUI available for RViz, Gazebo, rqt when connected to monitor
- Simulation: Pi 5 GPU limited — lightweight only; heavy simulation deferred to Phase 3+

## Power Architecture

```
3S LiPo (11.1V)
  ├──→ Pololu G2 18v17 motor driver VIN (direct 11.1V to motors)
  └──→ DROK buck converter → 5.1V
        ├──→ RPi 5 (via GPIO 5V pins)
        ├──→ Arduino Due (via VIN pin)
        │     └──→ Due onboard regulator → 3.3V internally
        │           ├──→ Due I/O pins (3.3V logic)
        │           └──→ 3.3V output pin → encoder power (so they output 3.3V)
        └──→ Adafruit STEMMA QT sensors via VIN (onboard regulators)
```

| Component | Operating Voltage | Notes |
|---|---|---|
| Raspberry Pi 5 | 5.1V | Via DROK buck → GPIO 5V pins |
| Arduino Due | 3.3V native | 5V on any I/O pin permanently kills it |
| Pololu G2 18v17 driver | 6.5–30V motor, 1.8–5.5V logic | 3.3V logic fully compatible |
| Pololu 37D encoders | 3.5–20V supply | Output voltage = supply voltage; powered at 3.3V → 3.3V outputs |
| BNO055 IMU (Adafruit) | 3.3V–5V via VIN | Onboard regulator, voltage-agnostic |
| INA219 current sensor (Adafruit) | 3.3V–5V via VIN | Onboard regulator, voltage-agnostic |
| APDS9960 gesture sensor (Adafruit) | 3.3V–5V via VIN | Onboard regulator, voltage-agnostic |
| HC-SR04 ultrasonic | 5V power | Trigger accepts 3.3V; **Echo outputs 5V** (see open questions) |
| RPi Camera Module 3 | 3.3V | Powered by RPi CSI connector |

**Key insight:** Encoders powered from Due's 3.3V pin → 3.3V outputs → zero level shifting needed throughout. G2 18v17 logic accepts 1.8–5.5V, so 3.3V works directly.

## Key Decisions

1. **Language:** Python primary, C++ for embedded/performance-critical
2. **Platform:** Raspberry Pi 5 (8GB) as primary dev environment
3. **OS/ROS:** Ubuntu 24.04 Desktop + ROS 2 Jazzy LTS (non-negotiable — Pi 5 requires Noble; Desktop for RViz/Gazebo/rqt)
4. **MCU:** Arduino Due (84 MHz ARM Cortex-M3, 3.3V native, hardware quadrature encoder via TC0 timer, already owned)
   - Fallback: ESP32 DevKitC (~$15) — same 3.3V logic, same serial protocol, minimal firmware changes
   - Mega 2560 retained as backup/prototyping only
5. **Arduino comms:** Custom serial protocol via `pyserial` (not micro-ROS) — simpler to debug; can migrate to micro-ROS later as portfolio refactor
6. **Motors:** Pololu 37D 50:1 with encoders (#4753) × 2 — 5.9x torque margin Phase 1, 1.26x Phase 2, 0.63 m/s both phases
7. **Motor driver:** Pololu G2 High-Power 18v17 (4.5A continuous per channel)
8. **Wheels:** 80mm diameter for both Phase 1 and Phase 2 (no swap between phases)
9. **IMU:** Adafruit BNO055 9-DOF STEMMA QT (#4646) — better ROS 2 ecosystem support than BNO085; built-in sensor fusion
10. **Battery:** 3S LiPo (11.1V nominal)
11. **Buck converter:** DROK 12A adjustable (5.3–32V to 1.2–32V) — adjustable to 5.1V, 2.4× headroom over RPi 5A requirement
12. **Encoder power strategy:** Power from Due's 3.3V output pin → 3.3V signals throughout, zero level shifting
13. **Connections:** No soldering/crimping — barrier terminal strips + lever connectors + jumper wires only
14. **Storage:** NVMe SSD boot (not SD card)
15. **Target robot mass:** 1.5kg with low center of gravity (critical for Phase 2 balancing)

## Open Questions

- HC-SR04 Echo pin (5V output): read from RPi GPIO directly vs. resistor divider (2.2k + 3.3k) to Due
- LiDAR: RPLidar A1M8 vs. alternatives (deferred to Phase 1 Month 2–3)
- Battery capacity: 2200mAh vs. 3000mAh (runtime vs. weight)
- Battery connector: XT60 vs. Deans vs. Anderson Powerpole
- Chassis dimensions and mounting locations (pending motor arrival for CAD)
- Raspberry Pi AI Kit (Hailo-8L): reconsider at Phase 2 Month 4+ when CV needed (requires M.2 slot trade-off with NVMe)
