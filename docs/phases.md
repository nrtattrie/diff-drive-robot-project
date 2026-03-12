**DIRECTIVE FOR AI ASSISTANTS:** This document tracks phase plans and current task status. It does NOT contain architectural decisions — see `architecture.md` for hardware, software stack, and key decisions. Work only one unchecked item at a time and stop for approval after each. Use checkboxes to track progress. If this document conflicts with a user request, flag it rather than silently overriding.

**Doc hierarchy:** `architecture.md` = canonical technical decisions. `phases.md` = canonical phase plan and task tracking. `docs/sessions/` = historical record of past sessions (do not update).

## Project Overview

Differential drive mobile robot (Phase 1) converting to self-balancing robot (Phase 2). Same hardware platform for both phases.

**Current status:** Phase 1 — Foundation. Stage 1 in progress (rebuilding from scratch; prior files were Claude-generated). Stage 2 not started.

---

## Phase 1: Foundation — Differential Drive Robot (Months 1–4)

**Goal:** Autonomous mobile robot with Nav2 + SLAM

**Skills:** ROS 2 architecture, SLAM (slam_toolbox), Nav2, odometry + sensor fusion, motor control (PID), hardware-software integration

### Stage 0: Hardware Procurement

- [x] Inventory existing hardware — RPi 5 (8GB), M.2 HAT+, cooler, Arduino Due, Arduino Mega 2560, Elegoo kit sensors/components
- [x] Order RPi accessories — 27W USB-C PSU, Camera Module 3, USB Mini Hub (Adafruit)
- [x] Motor selection + torque calculations — Pololu 37D 50:1 with encoder (#4753 x2)
- [x] Motor driver selected — Pololu G2 High-Power 18v17 (#2991)
- [x] IMU selected — Adafruit BNO055 9-DOF STEMMA QT (#4646)
- [x] MCU decision — Arduino Due as primary (84 MHz ARM, 3.3V native, hardware quadrature encoder); Mega 2560 as backup
- [x] Power architecture finalized — DROK 12A buck → 5.1V; 3S LiPo → motors direct; 3.3V logic throughout; zero level shifting
- [x] Full BOM finalized — Pololu $227.06 + Adafruit $96.59 + Amazon $39.46 = **$363.11**
- [x] All orders placed and received — motors, driver, wheels, hubs, caster, JST cables, IMU, GPIO headers, STEMMA QT cables, INA219, APDS9960, toggle switch, USB-TTL cable, buck converter, threadlocker, terminal blocks, lever connectors
- [ ] LiDAR purchase — RPLidar A1M8 (~$100) — deferred to Stage 4
- [ ] Battery purchase — 3S LiPo 11.1V 2200–3000mAh + charger (~$50) + XT60 connectors (~$3) — deferred to Stage 4

### Stage 1: Software-Only (no hardware needed)

- [ ] URDF — model robot geometry (chassis, wheels, caster, sensor frames), visualize via Foxglove
- [ ] tf2 — understand coordinate frames (`base_link → odom → map`)
- [ ] Gazebo Harmonic simulation — spawn URDF, drive with `teleop_twist_keyboard`
- [ ] ros2_control + diff_drive_controller — configured in simulation; twist_relay.py bridges Twist→TwistStamped

### Stage 2: Chassis + Arduino (can overlap with Stage 1)

- [ ] CAD & print chassis — mount points for motors, caster, RPi, Arduino, buck converter
- [ ] Arduino firmware — custom serial protocol: receive motor commands, send encoder counts
- [ ] Wiring — motors → driver → Arduino → RPi, buck converter → RPi GPIO 5V

### Stage 3: Real Hardware Driving (tethered power, no battery)

- [ ] Serial bridge ROS 2 node — Python/`pyserial` to translate between ROS 2 topics and Arduino serial
- [ ] Teleop on real robot — keyboard → `/cmd_vel` → motors spin → robot moves
- [ ] Odometry calibration — tune encoder-based odometry, compare commanded vs. actual motion

### Stage 4: Autonomous Navigation (requires LiDAR + battery)

- [ ] Purchase RPLidar A1M8 (~$100) and 3S LiPo 11.1V + charger (~$50) + XT60 connectors (~$3)
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
- [x] Complete URDF (chassis, wheels, caster, sensor frames)
- [x] Gazebo Harmonic simulation with ros2_control + diff_drive_controller
- [x] twist_relay.py — Twist→TwistStamped bridge for teleop compatibility
- [x] start_sim.sh — sim quickstart script

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
