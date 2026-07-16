#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_WS="$REPO_ROOT/software/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "Missing ROS setup file: $ROS_SETUP" >&2
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u

cd "$ROS_WS"
mkdir -p "$ROS_WS/log/ros"
export ROS_LOG_DIR="$ROS_WS/log/ros"

colcon build --packages-select diffbot_description
set +u
source "$ROS_WS/install/setup.bash"
set -u

cleanup() {
  if [[ -n "${DISPLAY_PID:-}" ]] && kill -0 "$DISPLAY_PID" 2>/dev/null; then
    kill "$DISPLAY_PID" 2>/dev/null || true
    wait "$DISPLAY_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 launch diffbot_description display.launch.py &
DISPLAY_PID=$!

sleep 2
if ! kill -0 "$DISPLAY_PID" 2>/dev/null; then
  wait "$DISPLAY_PID"
fi

echo
echo "Robot display is running."
echo "Starting Foxglove bridge on port 8765."

if command -v tailscale >/dev/null 2>&1; then
  TAILSCALE_IP="$(tailscale ip -4 2>/dev/null | head -n 1 || true)"
  if [[ -n "$TAILSCALE_IP" ]]; then
    echo "Foxglove URL: ws://$TAILSCALE_IP:8765"
  fi
fi

HOST_IP="$(hostname -I 2>/dev/null | awk '{print $1}' || true)"
if [[ -n "$HOST_IP" ]]; then
  echo "LAN URL:       ws://$HOST_IP:8765"
fi

echo
echo "Press Ctrl+C to stop the robot display and Foxglove bridge."
echo

ros2 run foxglove_bridge foxglove_bridge
