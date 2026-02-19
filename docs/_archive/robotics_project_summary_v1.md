# Nathan Tattrie - Robotics Career Pivot Project Summary

**Last Updated:** January 18, 2026  
**Status:** Phase 0 - Setup & Preparation

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

- MacBook Pro (2021) - documentation, CAD, SSH development
- iPad with OneNote - daily documentation
- Raspberry Pi 5 (8GB) with M.2 HAT+ and cooler
- Raspberry Pi NVMe 1TB SSD (arriving ~Feb 5, 2026)
- Bambu X1-C 3D printer
- Elegoo Mega R3 Starter Kit (Arduino)
- Google Drive + GitHub repository established

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

## Immediate Tasks (Before SSD Arrives - Feb 5)

### This Week:
- [ ] Install VS Code with Remote SSH extension on Mac
- [ ] Read ROS 2 Humble documentation overview
- [ ] Watch Articulated Robotics videos 1-3
- [ ] Set up GitHub repo structure for project

### When SSD Arrives:
- [ ] Flash Ubuntu 24.04 Server (64-bit) onto NVMe SSD
- [ ] Install ROS 2 Jazzy
- [ ] Configure VS Code Remote SSH workflow
- [ ] Complete first 4 ROS 2 tutorials

---

## Development Environment

**Primary Setup:**
- Raspberry Pi 5 running Ubuntu 24.04 + ROS 2 Jazzy
- SSH from MacBook using VS Code Remote SSH
- All ROS 2 development happens on Pi

**MacBook Used For:**
- Writing code (VS Code Remote)
- Documentation and blog posts
- Watching tutorials
- CAD design (SolidWorks/Fusion 360)

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
3. **ROS Version:** ROS 2 Jazzy on Ubuntu 24.04
4. **First Project:** Differential drive robot (foundation)
5. **Second Project:** Self-balancing robot (differentiation)
6. **Documentation Style:** Professional product development approach

**Terminology Note:** "Differential drive" = two independently driven wheels steering by speed difference (like Turtlebot). "Direct drive" in industry typically refers to gearless actuators for backdrivability (used in legged robots). Use "differential drive mobile robot" in portfolio and interviews.

---

## Open Questions / Future Decisions

- Exact form factor for differential drive robot chassis
- Sensor selection (LIDAR model, cameras)
- Motor/actuator selection for differential drive base
- Whether to add quadruped as 4th project
- Specific companies to prioritize for applications

---

## Resources

**Tutorials:**
- Articulated Robotics YouTube series
- ROS 2 official documentation: https://docs.ros.org/en/jazzy/
- Nav2 documentation

**Hardware Suppliers:**
- TBD based on component selection

**GitHub Repository:**
- [Link to be added]

---

## Progress Log

### January 18, 2026
- Hardware acquired: RPi 5, M.2 HAT+, cooler, 1TB NVMe (arriving Feb 5)
- Project summary document created
- Development environment plan established
- Clarified terminology: using "differential drive" (not "direct drive") for mobile robot base
