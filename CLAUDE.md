# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Differential drive mobile robot (Phase 1) converting to self-balancing robot (Phase 2). Same hardware platform for both phases. Currently in hardware selection/documentation phase — no application code exists yet.

**Read `/docs/robotics_project_summary_v6.md` first** — single source of truth for all project decisions.

## Architecture

- **Compute:** Raspberry Pi 5 (8GB), Ubuntu 24.04 Desktop, ROS 2 Jazzy LTS
- **Motor control:** Arduino Mega 2560 (low-level I/O, encoders)
- **Sensors:** RPLidar A1, BNO055 IMU, HC-SR04 ultrasonic, RPi Camera Module 3
- **Motors:** 2x Pololu 37D 50:1 with encoders, Pololu G2 18v17 driver
- **Power:** 3S LiPo → DROK buck converter → 5.1V for RPi via GPIO
- **Connection strategy:** No soldering — barrier terminal strips + lever connectors only

## Software Stack (Planned)

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
