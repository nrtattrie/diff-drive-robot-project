# Autonomous Mobile Robot

A differential drive mobile robot built with ROS 2 Jazzy, featuring autonomous navigation, SLAM mapping, and obstacle avoidance.

![Robot Image](media/images/hero.jpg)
*Coming soon: Hero image of completed robot*

## Project Status

🚧 **In Development** - Week 1: Hardware Setup

**Current Milestone:** Installing ROS 2 on Raspberry Pi 5

## Hardware

Differential drive robot powered by a Raspberry Pi 5 and Arduino Mega 2560, with LIDAR, IMU, and camera sensors. See the [Bill of Materials](docs/BOM.md) for the full parts list and sourcing details.

## Software Stack

- **OS:** Ubuntu 24.04 Server
- **Framework:** ROS 2 Jazzy
- **Navigation:** Nav2
- **Mapping:** slam_toolbox
- **Control:** ros2_control
- **Languages:** Python 3, C++

## Features

- [x] Project structure and documentation
- [ ] ROS 2 installation and setup
- [ ] Teleoperation (keyboard control)
- [ ] Odometry from wheel encoders
- [ ] SLAM mapping
- [ ] Autonomous navigation
- [ ] Obstacle avoidance
- [ ] Person following *(future)*

## Documentation

- [Bill of Materials](docs/BOM.md)
- [Assembly Guide](docs/assembly.md) *(coming soon)*
- [Software Setup](docs/software-setup.md) *(coming soon)*
- [Learning Log](docs/learning-log.md)
- [Troubleshooting](docs/troubleshooting.md) *(coming soon)*

## Repository Structure
```
autonomous-mobile-robot/
├── docs/                   # Documentation
├── hardware/               # Hardware design files
│   ├── cad/               # CAD exports (STL, STEP)
│   ├── electronics/       # Schematics, wiring diagrams
│   └── datasheets/        # Component datasheets
├── software/              # Software source code
│   ├── ros2_ws/          # ROS 2 workspace
│   └── arduino/          # Arduino firmware
├── media/                 # Images and videos
└── scripts/              # Utility scripts
```

## Getting Started

### Prerequisites
- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12+
- Arduino IDE (for motor controller)

### Installation
```bash
# Clone repository
git clone https://github.com/YOUR_USERNAME/autonomous-mobile-robot.git
cd autonomous-mobile-robot

# Build ROS 2 workspace
cd software/ros2_ws
colcon build
source install/setup.bash
```

*Detailed setup instructions coming soon*

## Build Log

- **2026-01-17:** Project started, hardware ordered, repository initialized
- **2026-01-18:** Raspberry Pi 5 setup, ROS 2 installation *(planned)*

## Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Articulated Robotics Tutorials](https://articulatedrobotics.xyz/)

## License

MIT License - See [LICENSE](LICENSE) for details

## Author

Nathan Renn Tattrie
- GitHub: [@YOUR_USERNAME](https://github.com/YOUR_USERNAME)
- LinkedIn: [Your Profile](https://linkedin.com/in/nathan-renn-tattrie)
- Email: nrtattrie@gmail.com

---

**⭐ Star this repo if you find it useful!**
