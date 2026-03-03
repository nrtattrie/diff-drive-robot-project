# Gazebo Simulation — Quick Start

---

## Option A — One command (recommended)

`scripts/start_sim.sh` opens all three terminals at once using tmux.

```bash
~/diff-drive-robot-project/scripts/start_sim.sh
```

You land on the **teleop** window with keyboard focus. Switch between windows:

| Action | Keys |
|--------|------|
| Switch to sim window | `Ctrl+B` then `1` |
| Switch to relay | `Ctrl+B` then `2` |
| Switch to foxglove | `Ctrl+B` then `3` |
| Switch to teleop | `Ctrl+B` then `4` |
| Detach (leave running) | `Ctrl+B` then `d` |
| Re-attach later | run `start_sim.sh` again |
| Kill everything | `Ctrl+B` then `:kill-session` |

Requires tmux: `sudo apt install tmux`

---

## Option B — Manual (three separate SSH terminals)

Each terminal must source the workspace first:
```bash
source /opt/ros/jazzy/setup.bash
source ~/diff-drive-robot-project/software/ros2_ws/install/setup.bash
```

| # | Purpose | Command |
|---|---------|---------|
| 1 | Simulation | `ros2 launch diffbot_description gazebo.launch.py` |
| 2 | Twist relay | `python3 ~/diff-drive-robot-project/scripts/twist_relay.py` |
| 3 | Foxglove bridge | `ros2 run foxglove_bridge foxglove_bridge` |
| 4 | Teleop (drive) | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |

Note: The relay (window 2) is needed because `diff_drive_controller` in ROS 2 Jazzy
requires `TwistStamped`, but `teleop_twist_keyboard` publishes plain `Twist`.

---

## Start order

Terminal 1 first. Wait until both controllers show `active`:
```bash
ros2 control list_controllers
# joint_state_broadcaster[...] active
# diffbot_base_controller[...]  active
```
Then start Terminals 2 and 3 in any order.

Terminal 3 (teleop) must stay **focused**. Use `i/j/l/,` to drive; `k` to stop.

---

## Foxglove WebSocket URL

The Pi's IP address can change. Get the current IP:
```bash
hostname -I   # use the first address shown
```

Connect Foxglove Studio to: `ws://<ip>:8765`

---

## Shutdown

Ctrl+C in each terminal (or `:kill-session` in tmux). Reverse order: teleop → foxglove → sim.
