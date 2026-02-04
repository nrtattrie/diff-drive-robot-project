# Nathan Tattrie - Robotics Career Pivot Project Summary

> **DIRECTIVE FOR AI ASSISTANTS:** This document is the single source of truth for all project decisions. Before answering ANY question, internalize the full document — hardware specs (8GB Pi 5 requires Desktop GUI for RViz/Gazebo learning), software stack (Ubuntu 24.04 Desktop + ROS 2 Jazzy — non-negotiable), phase goals, and decision history. Every recommendation must be consistent with what is documented here. If you are uncertain about a detail, re-read the relevant section before responding. Do not give generic advice. Do not skim. Trace how decisions connect across sections — hardware choices affect software choices affect phase goals. If something in this document conflicts with a user request, flag it rather than silently overriding.

**Last Updated:** February 4, 2026
**Status:** Phase 1 - Foundation (Development Environment Complete, Beginning ROS 2 Learning)

---

## Background & Context

### Professional Profile
- **Current Role:** Senior Mechanical NPI Engineer at Roam Robotics (Feb 2021 - Present)
- **Previous:** Product Design Engineer at Ford Motor Company (Aug 2019 - Jan 2021)
- **Education:** BS Mechanical Engineering, University of Michigan (2019), Minor in Entrepreneurial Studies
- **Location:** NYC (planning to relocate to San Francisco ~May 2027)
- **Current Salary:** $130k

### Key Strengths to Leverage
- Full product development lifecycle experience (PRD → ERD → Test Matrix → Verification)
- Hardware design, DFM, and manufacturing knowledge
- Test fixture design and V&V testing (FDA medical device experience)
- Cross-functional leadership and systems integration
- CAD expertise (SolidWorks), rapid prototyping (Bambu X1-C)

### Software/Technical Background
- Harvard CS50 completed
- Stanford Deep Learning Specialization (Coursera) completed
- Data Structures & Algorithms in C++ (University of Michigan)
- Currently doing daily LeetCode practice
- Primary language going forward: Python (C++ for embedded/performance-critical)

### Career Goal
Pivot from pure mechanical engineering to robotics/mechatronics engineering with strong software skills. Target roles that combine hardware and software, differentiating from both pure CS grads (weak hardware) and pure ME grads (weak software).

---

## Hardware Available

### Development & Fabrication
- MacBook Pro (2021) - documentation, CAD, SSH development
- iPad with OneNote - daily documentation
- Bambu X1-C 3D printer
- Google Drive + GitHub repository established

### Compute & Control
- Raspberry Pi 5 (8GB) with M.2 HAT+ and cooler
- Raspberry Pi NVMe 1TB SSD - **INSTALLED & CONFIGURED** (booting from NVMe)
- Raspberry Pi 27W USB-C Power Supply (ordered - essential for Pi 5 operation with peripherals)
- Raspberry Pi Camera Module 3 (ordered - for Phase 2/3 computer vision)
- USB Mini Hub with Power Switch from Adafruit (ordered - peripheral management)
- Arduino Mega 2560 (from Elegoo kit) - motor control, encoders, low-level I/O
- Arduino DUE V02G - backup/future use (faster processor for complex control)
- Arduino Mega Prototype Shield v3 - clean wiring for motor driver

**Note on M.2 HAT:** The M.2 slot can hold either an NVMe SSD or the Raspberry Pi AI Kit (Hailo-8L accelerator), but not both simultaneously. Decision was made to prioritize NVMe for fast storage during Phase 1-3 (navigation, SLAM, motor control). AI Kit can be reconsidered in Phase 2 (Month 4+) when computer vision becomes relevant. At that point, options include: booting from SD card, using a USB-to-NVMe adapter, or getting a second Pi.

### From Elegoo Mega R3 Starter Kit (Useful for Robot)
- Ultrasonic sensor (HC-SR04) - obstacle detection
- IR sensors - cliff detection or line following
- Servo motors - camera pan/tilt mount (later)
- LEDs, buttons, buzzer - status indicators, debugging
- Breadboard, jumper wires - prototyping

### Hardware Finalized - Ready to Order! (~$305 total)

**POLOLU ORDER ($227.06):**
- 3× JST SH-Style Cable, 5-Pin, Single-Ended Female, 12cm (#5530) - $4.77
- 3× JST SH-Style Cable, 5-Pin, Female-Female, 16cm (#5534) - $8.25
- 1× Pololu G2 High-Power Motor Driver 18v17 (#2991) - $44.95
- 2× 37D 50:1 Metal Gearmotor with 64 CPR Encoder (#4753) - $141.90
  - Stall torque: 0.75 N·m at 12V (~0.69 N·m at 11.1V)
  - No-load speed: 200 RPM at 12V (~185 RPM at 11.1V)
  - 64 CPR quadrature encoder (256 counts per revolution)
- 1× Universal Aluminum Mounting Hub for 6mm Shaft (#1083, 2-pack) - $12.95
- 1× Pololu Wheel 80×10mm Pair - Black (#1430) - $8.75
- 1× Pololu Ball Caster with 3/4" Plastic Ball (#954) - $5.49

**ADAFRUIT ORDER ($96.59 with tax/shipping):**
- 1× Insulated Silicone Rework Mat - 34cm x 23cm (#3536) - $9.95
- 1× GPIO Header for Raspberry Pi - Tall 2×20 Female Header (#1992) - $2.95
- 1× GPIO Stacking Header for Pi A+/B+/Pi 2/Pi 3 - Extra-long 2×20 (#2223) - $2.50
- 1× Adafruit 9-DOF BNO055 IMU Fusion Breakout - STEMMA QT (#4646) - $29.95
- 3× STEMMA QT Cable - 100mm Long (#4210) - $2.85
- 2× STEMMA QT Cable - 200mm Long (#4401) - $2.50
- 1× INA219 High Side DC Current Sensor - 26V ±3.2A - STEMMA QT (#904) - $9.95
  - Monitor battery current draw and power consumption
  - Optimize runtime and detect issues
  - I2C interface via STEMMA QT
- 1× Mini Panel Mount SPDT Toggle Switch (#3221) - $0.95
  - Master power switch for robot
- 1× APDS9960 Proximity, Light, RGB, and Gesture Sensor - STEMMA QT (#3595) - $7.50
  - Gesture recognition for interactive control
  - Proximity detection
  - Fun feature for demos
- 1× USB to TTL Serial Cable - Debug/Console Cable for Raspberry Pi (#954) - $9.95
  - Essential for debugging when SSH fails
  - Serial console access to RPi

**AMAZON ORDER ($39.46):**
- 1× Loctite 243 Blue Threadlocker - Medium Strength, 50ml - $6.99
- 1× ALITOVE 3-Port Lever Wire Connectors (10pcs, 28-12 AWG) - $7.99
- 1× Weewooday 12-Position Barrier Terminal Strips (10pcs, 10A 380V) - $6.99
- 1× DROK 12A Buck Converter (5.3-32V to 1.2-32V, adjustable, LCD display) - $17.49

**TOTAL ORDERS: $363.11**

**Design Performance Validated:**
- Phase 1 Navigation (80mm wheels, 1.5kg): 5.9x torque margin ✓
- Phase 1 Speed: ~0.63 m/s (perfect for 0.3-0.7 m/s target range)
- Phase 2 Balancing (80mm wheels, 1.5kg, low CoG): 1.26x torque margin ✓
- Phase 2 Speed: ~0.63 m/s
- Current draw: ~7A peak for both motors (motor driver rated 4.5A continuous per channel) ✓
- Power distribution: Buck converter 12A >> 5A RPi requirement ✓

**Still Needed for Mobile Operation (Phase 1 Month 2-3):**
- RPLidar A1M8 2D LiDAR Scanner (~$100)
- 3S LiPo Battery 11.1V 2200-3000mAh + Charger (~$50)
- XT60 connectors for battery (~$3)

**From Existing Inventory:**
- Jumper wires (M-M, M-F, F-F) from Elegoo Mega kit ✓
- Arduino Mega 2560 (optional for motor control offload)
- Breadboards for prototyping
- Basic electronics components (resistors, LEDs, etc.)

---

## 12-Month Learning Path

### Phase 1: Foundation - Differential Drive Robot (Months 1-4)
**Goal:** Build autonomous mobile robot with Nav2 + SLAM

**Skills to Acquire:**
- ROS 2 architecture (nodes, topics, services, actions)
- SLAM (slam_toolbox)
- Nav2 (path planning, costmaps, recovery behaviors)
- Odometry and sensor fusion
- Motor control (PID tuning)
- Hardware-software integration

**Differentiation Strategy:**
- Document like a product development effort (requirements → design → test → verification)
- Custom 3D-printed chassis showcasing ME design skills
- Clean code with professional documentation
- Blog posts explaining design decisions

**Note:** Diff-drive alone won't differentiate - this is foundation only, not portfolio centerpiece.

### Phase 2: Controls & Wow Factor - Self-Balancing Robot (Months 5-8)
**Goal:** Convert diff-drive base to two-wheeled inverted pendulum

**Skills to Acquire:**
- Advanced controls (PID/LQR/MPC)
- System dynamics and modeling
- Real-time feedback systems
- Simulation-to-real deployment

**Why This Matters:**
- Visually impressive (balancing like Segway)
- Only ~1-2% of portfolios have this vs 30-40% with diff-drive
- Demonstrates controls theory knowledge
- Uses same ROS 2 infrastructure from Phase 1

### Phase 3: Manipulation - Mobile Manipulator or Robot Arm (Months 9-12)
**Goal:** Vision-guided pick-and-place system

**Skills to Acquire:**
- Forward/inverse kinematics
- MoveIt2 integration
- Computer vision (OpenCV, object detection)
- Trajectory planning
- Force control basics

**Options:**
- A) Standalone 4-6 DOF robot arm with vision-guided grasping
- B) Add gripper to mobile robot for "fetch-and-place" demo

### Phase 4: Advanced Features (Month 12+, Optional)
Choose based on target companies:
- Multi-robot coordination (warehouse/delivery roles)
- Reinforcement learning for robotics (research-heavy companies)
- Outdoor autonomy with GPS (agricultural/construction/defense)
- Quadruped robot (maximum wow factor, rare in portfolios)

---

## Skills Priority (Based on Job Market Analysis)

### Tier S - Must Have (85-95% of job postings):
- ROS/ROS 2
- C++ and Python
- Autonomous Navigation (SLAM, path planning)
- Computer Vision
- Linux/Ubuntu

### Tier A - High Value (45-60% of postings):
- Manipulation (kinematics, MoveIt)
- Sensor fusion (Kalman filters, EKF)
- Machine learning (PyTorch/TensorFlow)
- Simulation (Gazebo, Isaac Sim)

### Tier B - Nice to Have (15-30% of postings):
- Multi-robot systems
- Controls theory (PID, MPC)
- Docker/Kubernetes
- Cloud robotics

---

## Immediate Tasks

### Completed:
- [x] Install VS Code with Remote SSH extension on Mac
- [x] Set up GitHub repo structure for project
- [x] Watch Articulated Robotics videos (ongoing reference)
- [x] Flash Ubuntu 24.04 LTS (Noble) Desktop (64-bit) onto NVMe SSD
- [x] Configure Pi 5 to boot from NVMe
- [x] Install ROS 2 Jazzy (LTS version, officially supported on Ubuntu 24.04)
- [x] Configure VS Code Remote SSH workflow
- [x] Set up SSH alias ("pi") on Mac for easy connection
- [x] Clone project repository onto Pi
- [x] Initialize ros2_ws workspace with colcon build
- [x] Run ROS 2 talker/listener demo successfully

### In Progress:
- [ ] Complete ROS 2 Jazzy CLI tutorials: https://docs.ros.org/en/jazzy/Tutorials.html
- [ ] Read ROS 2 Jazzy documentation overview (concepts, architecture)
- [ ] Complete first 4 ROS 2 Jazzy tutorials on the Pi

### Upcoming:
- [ ] Create first custom ROS 2 package for robot
- [ ] Begin URDF tutorial (robot description)
- [ ] Set up VS Code SSH key authentication (passwordless)

---

## Development Environment

**Primary Setup (OPERATIONAL):**
- Raspberry Pi 5 (8GB) running Ubuntu 24.04.3 LTS Desktop (Noble) + ROS 2 Jazzy
- Booting from 1TB NVMe SSD (939GB available)
- SSH from MacBook using VS Code Remote SSH (primary workflow)
- SSH alias configured: `ssh pi` connects to Pi at 192.168.0.99
- Desktop GUI available for RViz, Gazebo, rqt tools when plugged into monitor
- All ROS 2 development happens on Pi
- Project repo cloned to: `~/diff-drive-robot-project`
- ROS 2 workspace: `~/diff-drive-robot-project/software/ros2_ws`

**Environment Configuration:**
- `.bashrc` sources `/opt/ros/jazzy/setup.bash` automatically
- `.bashrc` sources workspace `install/setup.bash` automatically
- Git configured with user identity

**MacBook Used For:**
- Writing code (VS Code Remote)
- Documentation and blog posts
- Watching tutorials
- CAD design (SolidWorks/Fusion 360)

**Simulation Strategy:**
- Desktop GUI enables running RViz and Gazebo directly on Pi 5 with monitor
- Pi 5 GPU is limited — lightweight simulations for basic testing and visualization
- Heavy simulation work deferred until Phase 3+ or dedicated Linux machine
- Focus on real hardware testing during Phase 1-2

**Future Addition (Month 3+ if needed):**
- Dedicated Linux machine for heavy simulation (Gazebo)
- Used desktop or mini PC ($200-400)

---

## Portfolio Strategy

The goal is three complementary projects that tell a progression story:

1. **"The Reliable Workhorse"** - Diff-drive robot with Nav2
   - Shows industry-standard fundamentals
   - Gets past HR/ATS screens

2. **"The Differentiator"** - Self-balancing robot
   - Shows advanced controls and creative problem-solving
   - Visually impressive for demos

3. **"The Specialist"** - Mobile manipulator or arm
   - Shows manipulation + vision integration
   - Demonstrates full-stack robotics capability

**Documentation Approach:**
- Treat each project like professional product development
- Write actual requirements documents
- Create test procedures and show verification data
- This differentiates from typical hobbyist GitHub projects

---

## Target Job Market

**Primary Focus (1,700+ openings):**
- Mobile robot companies
- Warehouse automation
- Autonomous vehicles
- Delivery robots
- Defense robotics
- Service robots
- Agricultural robots

**Companies to Target:**
- Boston Dynamics, Agility Robotics, Figure AI, Apptronik
- Amazon Robotics, Locus Robotics, Fetch Robotics
- Waymo, Cruise, Aurora, Nuro
- Clearpath, Formant, InOrbit

**Competitive Advantage:**
- Strong hardware + strong software = rare combination
- Professional product development experience (unlike most portfolio projects)
- NPI and V&V background differentiates from CS grads

---

## Key Decisions Made

1. **Language:** Python primary, C++ for embedded/performance
2. **Platform:** Raspberry Pi 5 as primary dev environment
3. **ROS Version:** ROS 2 Jazzy (LTS) on Ubuntu 24.04 LTS Desktop (Noble)
   - Desktop (not Server): GUI needed for RViz, Gazebo, rqt during learning; 8GB RAM sufficient
4. **First Project:** Differential drive robot (foundation)
5. **Second Project:** Self-balancing robot (differentiation)
6. **Documentation Style:** Professional product development approach
7. **Power Supply:** Raspberry Pi 27W USB-C supply (essential for stable operation with peripherals)
8. **Motors:** Pololu 37D Metal Gearmotor 50:1 with Encoder (Part #4753) - 2× units
   - Sufficient for both Phase 1 navigation and Phase 2 balancing
   - Target mass: 1.5kg with low center of gravity for Phase 2
   - 70% safety factor (typical for hobby robotics)
9. **Motor Driver:** Pololu Dual G2 High-Power 18v17 (4.5A continuous per channel)
10. **IMU:** Adafruit BNO055 9-DOF STEMMA QT (#4646) - $29.95
    - Better ROS 2 ecosystem support (mature drivers, extensive documentation)
    - STEMMA QT plug-and-play interface
11. **Wheels:** 80mm diameter for BOTH Phase 1 and Phase 2
    - Simplifies design (no wheel swap between phases)
    - 5.9x torque margin for Phase 1, 1.26x for Phase 2
    - Speed: 0.63 m/s (perfect for 0.3-0.7 m/s target)
12. **Battery:** 3S LiPo (11.1V nominal) - confirmed for motor voltage compatibility
13. **Buck Converter:** DROK 12A Adjustable (5.3-32V to 1.2-32V) with LCD
    - Adjustable to 5.1V for RPi 5 requirements
    - 12A continuous >> 5A RPi requirement (2.4× headroom)
    - Handles voltage sag and current spikes
14. **Connection Hardware (No Soldering/Crimping):**
    - Barrier terminal strips for permanent chassis connections
    - ALITOVE 3-port lever connectors for temporary/testing
    - Jumper wires from Elegoo kit for GPIO connections
15. **Storage:** Boot from NVMe SSD (not SD card)
    - Better performance (3-5x faster read/write)
    - Better reliability for development workloads
    - 939GB available storage

**Terminology Note:** "Differential drive" = two independently driven wheels steering by speed difference (like Turtlebot). "Direct drive" in industry typically refers to gearless actuators for backdrivability (used in legged robots). Use "differential drive mobile robot" in portfolio and interviews.

**Design Philosophy:** Components selected based on detailed torque/speed calculations with inspiration from ELEGOO Tumbller (30:1 motors, 67mm wheels, ~1.5kg). Our 50:1 motors with 80mm wheels provide better mechanical advantage while maintaining similar performance envelope. All connections designed to avoid soldering/crimping for rapid iteration.

---

## Open Questions / Future Decisions

- LiDAR specific model: RPLidar A1M8 vs alternatives (deferred to Month 2-3)
- Battery capacity selection: 2200mAh vs 3000mAh (runtime vs weight trade-off)
- Battery connector type: XT60 vs Deans vs Anderson Powerpole
- Chassis detailed dimensions and mounting locations (CAD design pending motor arrival)
- Raspberry Pi AI Kit: reconsider at Month 4+ when CV is needed (requires M.2 slot trade-off)
- Whether to add quadruped as 4th project
- Specific companies to prioritize for applications

---

## Resources

**Tutorials:**
- Articulated Robotics YouTube series (ongoing reference)
- ROS 2 Jazzy official documentation: https://docs.ros.org/en/jazzy/
- ROS 2 Jazzy tutorials: https://docs.ros.org/en/jazzy/Tutorials.html
- Nav2 documentation

**Hardware Suppliers:**
- Adafruit (USB hub, various electronics)
- Pololu (motors, motor drivers)
- TBD based on component selection

**GitHub Repository:**
- https://github.com/nrtattrie/diff-drive-robot-project

---

## Progress Log

### February 4, 2026 - DEVELOPMENT ENVIRONMENT COMPLETE!
- **NVMe SSD Setup (arrived early!):**
  - Downloaded Ubuntu 24.04.3 LTS Desktop image directly on Pi via wget
  - Flashed image to NVMe using `dd` while booted from SD card
  - Configured Pi 5 EEPROM boot order to prioritize NVMe (`BOOT_ORDER=0xf416`)
  - Successfully booting from NVMe with full 939GB available
  - Removed SD card dependency
- **SSH & Remote Development Configured:**
  - Set up SSH server on Pi (openssh-server)
  - Created SSH config alias on Mac (`~/.ssh/config` with Host "pi")
  - Pi accessible at 192.168.0.99 on local network
  - VS Code Remote SSH working (required enabling Local Network permission on macOS)
- **ROS 2 Jazzy Installation:**
  - Configured locale (en_US.UTF-8) for ROS 2 compatibility
  - Added ROS 2 apt repository with GPG key verification
  - Installed `ros-jazzy-desktop` package (includes RViz, Gazebo integration, rqt, demos)
  - Added `source /opt/ros/jazzy/setup.bash` to `.bashrc`
  - Verified installation with `ros2 doctor` - all 5 checks passed
- **Workspace Setup:**
  - Installed git on Pi and configured user identity
  - Cloned project repository to `~/diff-drive-robot-project`
  - Initialized ROS 2 workspace with `colcon build`
  - Added workspace setup to `.bashrc`
- **First ROS 2 Demo:**
  - Successfully ran talker/listener demo (`ros2 run demo_nodes_cpp talker/listener`)
  - Demonstrated publish/subscribe communication pattern
  - Ready to begin official ROS 2 tutorials

### January 31, 2026 - OS/ROS 2 Decision Revised & Raspberry Pi 5 First Boot
- **CRITICAL REVISION: Ubuntu 24.04 LTS (Noble) + ROS 2 Jazzy replaces Ubuntu 22.04 / ROS 2 Humble**
  - Ubuntu 22.04 (Jammy) is NOT officially supported on Raspberry Pi 5
  - Pi 5 official Ubuntu support begins at 24.04; running 22.04 requires kernel hacks (transplanting 23.10 kernel)
  - ROS 2 Jazzy is the current LTS release (May 2024), targeting Ubuntu 24.04 with native aarch64 deb packages
  - Jazzy supported until May 2029 (5-year LTS cycle)
  - Articulated Robotics tutorials target Humble, but core ROS 2 concepts (nodes, topics, ros2_control, Nav2, SLAM) are identical across Humble/Jazzy — minor command/package name differences only
  - Previous decision (Jan 24) was based on Articulated Robotics recommending Ubuntu MATE 20.04, which was then "upgraded" to 22.04/Humble without accounting for Pi 5 hardware compatibility
- **Raspberry Pi 5 First Boot:**
  - Flashed 64GB SD card using Raspberry Pi Imager v2.0.3
  - OS: Ubuntu Desktop 24.04.3 LTS (64-bit) via "Other general-purpose OS" → Ubuntu
  - Desktop chosen over Server: 8GB RAM provides sufficient headroom (~500MB-1GB for GUI), and Desktop GUI is needed for RViz, Gazebo visualization, and rqt tools during learning
  - Can disable desktop environment later if RAM becomes constrained during autonomous operation
  - Pre-configured: hostname, WiFi (home network SSID), user account, SSH enabled (password auth)
  - Skipped Raspberry Pi Connect (unnecessary for local development)
  - NVMe SSD can remain plugged in during boot — Pi 5 defaults to SD card boot priority
- **Decision: Committed to ROS 2 from Phase 1 onward**
  - Considered splitting: RPi OS for Phase 1, ROS 2 for Phase 2 — rejected
  - Phase 1 goals (Nav2 + SLAM) require ROS 2 packages; splitting would mean rebuilding software stack mid-project
  - Compound learning: Phase 1 ROS 2 foundation (nodes, topics, tf2, URDF, launch files) directly supports Phase 2 controls work
  - Portfolio coherence: ROS 2 throughout tells a professional robotics story vs. hobbyist scripts

### January 26, 2026 (Afternoon) - ALL COMPONENTS FINALIZED!
- **MILESTONE: Ready to place orders!**
- **Shopping Carts Finalized:**
  - Pololu: $227.06 (motors, driver, wheels, hubs, caster, cables)
  - Adafruit: $96.59 (IMU, GPIO headers, STEMMA QT cables, debugging tools, sensors)
  - Amazon: $39.46 (buck converter, threadlocker, terminal blocks, lever connectors)
  - **Total: $363.11**
- **Adafruit Additions (using gift card):**
  - INA219 current sensor for monitoring battery/power consumption
  - USB to TTL serial cable for debugging RPi when SSH fails
  - APDS9960 gesture sensor for interactive robot control (fun demo feature)
  - Mini panel mount toggle switch for master power control
  - Total additions: ~$28 worth of debugging tools and interactive sensors
- **Power System Finalized:**
  - DROK 12A adjustable buck converter selected over Pololu D24V90F5
  - Reasoning: Need adjustable voltage for RPi 5's 5.1V requirement
  - 12A continuous provides 2.4× headroom over 5A RPi requirement
  - Handles current spikes and voltage sag without throttling
- **Connection Strategy (No Soldering/Crimping):**
  - Weewooday barrier terminal strips (10pcs) for permanent chassis mounting
  - ALITOVE 3-port lever connectors (10pcs) for temporary/testing connections
  - Jumper wires from existing Elegoo Mega kit for GPIO connections
  - All connections can be made with screwdrivers and wire strippers only
- **Wheel Decision Simplified:**
  - Using 80mm wheels for BOTH Phase 1 and Phase 2 (not swapping)
  - Saves $8-10 and eliminates mechanical change between phases
  - Phase 1 performance: 5.9x torque margin, 0.63 m/s speed ✓
  - Phase 2 performance: 1.26x torque margin, 0.63 m/s speed ✓
- **IMU Confirmation:**
  - BNO055 chosen over BNO085 for better ROS 2 ecosystem support
  - More mature drivers, better documentation, proven in similar projects
  - STEMMA QT version (#4646) at $29.95
- **Buck Converter Deep Dive:**
  - Analyzed RPi 5 power requirements: 5A nominal, spikes to 5A+
  - Determined 5A buck insufficient due to efficiency losses and thermal derating
  - Selected 12A adjustable for proper headroom and 5.1V capability
  - Will wire directly to GPIO pins (5V + GND) with thick, short wires

### January 26, 2026 (Morning) - MAJOR MILESTONE: Motor and IMU Selection Finalized!
- **Motor Selection Process:**
  - Researched ELEGOO Tumbller specifications (30:1 motors, 67mm wheels, ~1.5kg)
  - Performed detailed torque calculations for both Phase 1 (navigation) and Phase 2 (balancing)
  - Evaluated Pololu 37D series: 19:1, 30:1, 50:1, 70:1, 131:1 gear ratios
  - **Decision: Pololu 37D 50:1 with encoders (Part #4753)**
    - Phase 1 (65mm wheels, 1.5kg): 7.3x torque margin ✓
    - Phase 2 (80mm wheels, 1.5kg, low CoG): 1.26x torque margin ✓
    - Speed: 0.51 m/s (Phase 1) and 0.63 m/s (Phase 2) - perfect for target range
  - **Motor Driver: Pololu Dual G2 High-Power 18v17**
    - 4.5A continuous per channel (handles ~7A peak load from both motors)
- **IMU Selection:**
  - Evaluated BNO055 vs BNO085 vs MPU6050
  - **Decision: Adafruit BNO055 9-DOF STEMMA QT (#4646) - $29.95**
    - Built-in sensor fusion (saves weeks of filter tuning)
    - STEMMA QT plug-and-play connection
    - Critical for Phase 2 balancing (need accurate pitch/angular velocity)
    - Good for Phase 1 odometry fusion with wheel encoders
    - Better ROS 2 support than BNO085
- **Wheel Strategy:**
  - Phase 1: 65mm diameter (matches speed requirements)
  - Phase 2: 80mm diameter (provides better mechanical advantage for balancing)
  - Wheels are cheap compared to motors - can swap between phases
- **Design Constraints Refined:**
  - Target mass: 1.5kg (down from 3kg conservative estimate)
  - Low center of gravity crucial for Phase 2 (battery placement at bottom)
  - Safety factor: 70% of motor stall torque (relaxed from 50% for hobby use)
- **GPIO Access Solution:**
  - Need GPIO stacking header for M.2 HAT+ to access pins for IMU connection
  - Will use STEMMA QT to female jumper cable for I2C connection

### January 24, 2026
- **Hardware Updates:**
  - Ordered Raspberry Pi 27W USB-C Power Supply (essential for stable Pi 5 operation)
  - Ordered Raspberry Pi Camera Module 3 (for Phase 2/3 computer vision)
  - Ordered USB Mini Hub with Power Switch from Adafruit
- **Software Stack Decisions:**
  - ~~Previously confirmed Ubuntu 22.04 LTS (Jammy) / ROS 2 Humble — REVISED (see Jan 31 entry)~~
  - **Now using Ubuntu 24.04 LTS (Noble) + ROS 2 Jazzy (LTS, supported until May 2029)**
  - Ubuntu 22.04 is not officially supported on Raspberry Pi 5; official support starts at 24.04
  - ROS 2 Jazzy is the current LTS and targets Ubuntu 24.04 with native aarch64 deb packages
- **Learning Plan:**
  - Continuing to watch Articulated Robotics videos as reference
  - Starting ROS 2 Jazzy tutorials: https://docs.ros.org/en/jazzy/Tutorials.html
  - Focus on conceptual learning until NVMe SSD arrives (~Feb 5)
- **Component Selection Status:**
  - Motor selection deferred until after Ubuntu setup and ROS 2 fundamentals
  - Will revisit motor calculations once development environment is operational

### January 18, 2026
- Hardware acquired: RPi 5, M.2 HAT+, cooler, 1TB NVMe (arriving Feb 5)
- Project summary document created
- Development environment plan established
- Clarified terminology: using "differential drive" (not "direct drive") for mobile robot base
- VS Code + Remote SSH extension installed on Mac
- GitHub repo structure set up
- Inventoried existing hardware: Arduino Mega, DUE, Prototype Shield, Elegoo kit sensors
- Defined design constraints: 3kg max mass, 65mm wheels, 0.3-0.7 m/s speed, indoor use
- Motor and driver selection in progress (Pololu 37D family, gear ratio TBD)
