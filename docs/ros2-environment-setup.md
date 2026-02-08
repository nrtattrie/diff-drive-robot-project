# ROS 2 Environment Setup

Quick reference for configuring the ROS 2 development environment on this project.

## Sourcing the ROS 2 Installation

The primary setup step is sourcing the ROS 2 underlay. This sets all required environment variables automatically:

```bash
# For Humble (Ubuntu 22.04):
source /opt/ros/humble/setup.bash

# For Jazzy (Ubuntu 24.04):
source /opt/ros/jazzy/setup.bash
```

Add the appropriate line to `~/.bashrc` so it runs on every new terminal.

## Environment Variables (set automatically by sourcing)

| Variable             | Value   | Description                          |
|----------------------|---------|--------------------------------------|
| `ROS_VERSION`        | `2`     | ROS major version                    |
| `ROS_PYTHON_VERSION` | `3`     | Python major version used by ROS     |
| `ROS_DISTRO`         | `humble` or `jazzy` | ROS 2 distribution name |

These are set by `setup.bash` — do not set them manually.

## ROS_DOMAIN_ID

Controls DDS network isolation. Nodes only discover peers on the same domain ID.

- **Default:** `0` (used when unset)
- **Valid range:** `0` to `101` on Linux (higher values risk ephemeral port conflicts)
- **When to change:** shared networks (labs, classrooms) where multiple users run ROS 2
- **Single-user development:** leaving it unset (default `0`) is fine

```bash
# Only if needed for network isolation:
export ROS_DOMAIN_ID=42
```

## ROS_LOCALHOST_ONLY

Restricts DDS communication to localhost. Useful for development on a single machine where you don't want to interact with other networked ROS 2 systems:

```bash
export ROS_LOCALHOST_ONLY=1
```

## Verifying Your Environment

```bash
# Check all ROS-related environment variables:
printenv | grep -i ROS

# Expected output (example for Humble):
# ROS_VERSION=2
# ROS_PYTHON_VERSION=3
# ROS_DISTRO=humble

# Verify ROS 2 CLI works:
ros2 doctor --report
```

## Workspace Overlay

After sourcing the underlay, source your project workspace overlay:

```bash
cd ~/diff-drive-robot-project/software/ros2_ws
colcon build
source install/setup.bash
```

The workspace overlay must be sourced **after** the underlay in every terminal.

## Distro Compatibility

| ROS 2 Distro | Ubuntu Version | EOL        |
|---------------|---------------|------------|
| Humble        | 22.04 LTS     | May 2027   |
| Jazzy         | 24.04 LTS     | May 2029   |

Choose one distro and stay consistent across your development machine and Raspberry Pi.
