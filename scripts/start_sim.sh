#!/bin/bash
# Start the diffbot Gazebo simulation in four tmux windows.
#
# Usage:  ./scripts/start_sim.sh
#
# Windows created:
#   sim      — Gazebo + ros2_control (Terminal 1)
#   relay    — Twist → TwistStamped relay (Terminal 2)
#   foxglove — Foxglove WebSocket bridge (Terminal 3)
#   teleop   — Keyboard drive control (Terminal 4)
#
# Switch windows : Ctrl+B  then  1 / 2 / 3 / 4
# Detach session : Ctrl+B  then  d   (simulation keeps running)
# Kill session   : Ctrl+B  then  :kill-session
#
# Requires: tmux  (sudo apt install tmux)

SESSION="diffbot"

SETUP="source /opt/ros/jazzy/setup.bash && \
       source $HOME/diff-drive-robot-project/software/ros2_ws/install/setup.bash"

# ── If already attached to this session, just switch back to it ──────────────
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already running — attaching."
    tmux attach-session -t "$SESSION"
    exit 0
fi

# ── Window 1: simulation ─────────────────────────────────────────────────────
tmux new-session -d -s "$SESSION" -n "sim"
tmux send-keys -t "$SESSION:sim" \
    "$SETUP && ros2 launch diffbot_description gazebo.launch.py" Enter

# ── Window 2: Twist relay ─────────────────────────────────────────────────────
# diff_drive_controller requires TwistStamped; teleop publishes plain Twist.
# This relay converts between them.
tmux new-window -t "$SESSION" -n "relay"
tmux send-keys -t "$SESSION:relay" \
    "$SETUP && python3 $HOME/diff-drive-robot-project/scripts/twist_relay.py" Enter

# ── Window 3: Foxglove bridge ─────────────────────────────────────────────────
tmux new-window -t "$SESSION" -n "foxglove"
tmux send-keys -t "$SESSION:foxglove" \
    "$SETUP && ros2 run foxglove_bridge foxglove_bridge" Enter

# ── Window 4: teleop ──────────────────────────────────────────────────────────
tmux new-window -t "$SESSION" -n "teleop"
tmux send-keys -t "$SESSION:teleop" \
    "$SETUP && ros2 run teleop_twist_keyboard teleop_twist_keyboard" Enter

# ── Attach on the teleop window (needs keyboard focus to drive) ───────────────
tmux select-window -t "$SESSION:teleop"
tmux attach-session -t "$SESSION"
