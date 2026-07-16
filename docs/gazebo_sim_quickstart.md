# Gazebo Simulation Quickstart

This starts the current lightweight Gazebo Harmonic simulation for the rough
diff-drive base.

This workflow uses multiple terminals. The Gazebo launch command keeps running;
that is normal.

## Terminal 1: Launch Gazebo

```bash
cd ~/diff-drive-robot-project/software/ros2_ws
colcon build --packages-select diffbot_description
source install/setup.bash
ros2 launch diffbot_description gazebo.launch.py
```

By default this runs Gazebo server-only with no popup:

```text
-s -r empty.sdf
```

To open the Gazebo GUI instead, stop Terminal 1 with `Ctrl+C`, then relaunch:

```bash
ros2 launch diffbot_description gazebo.launch.py gz_args:="-r empty.sdf"
```

If you are SSH'd into the Pi from a Mac, the Gazebo GUI may open on the Pi's
desktop, not on the Mac. Use the server-only launch for headless testing.

The launch also starts `foxglove_bridge`, so Foxglove Studio on the Mac can
connect to:

```text
ws://100.105.3.35:8765
```

In Foxglove's 3D panel, use:

```text
Fixed frame: odom
```

The robot should move in Foxglove after teleop commands are sent.

## Terminal 2: Drive With Teleop

```bash
cd ~/diff-drive-robot-project/software/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/diffbot/cmd_vel
```

Use the normal `teleop_twist_keyboard` keys:

```text
i = forward
, = backward
j = turn left
l = turn right
k = stop
```

The remap is needed because Gazebo's diff-drive plugin listens on:

```text
/model/diffbot/cmd_vel
```

## Optional: Confirm Motion From Odometry

In a third terminal:

```bash
cd ~/diff-drive-robot-project/software/ros2_ws
source install/setup.bash
ros2 topic echo /model/diffbot/odometry --field pose.pose.position
```

When the robot moves, the reported position should change.

## What This Stage Uses

- `gazebo.launch.py` starts Gazebo, spawns the URDF, and bridges command/odometry topics.
- The URDF uses Gazebo's built-in `DiffDrive` system plugin.
- `odom_to_tf.py` converts Gazebo odometry into the `odom -> base_link` transform for Foxglove.
- Gazebo joint states are bridged to `/joint_states` so wheel transforms can update.
- This does not use `ros2_control` yet. That is the next Stage 1 box.

## Shutdown

Press `Ctrl+C` in the teleop terminal, then `Ctrl+C` in the Gazebo launch terminal.
