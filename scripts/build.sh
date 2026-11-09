#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
PY="${OUTDOOR_SERVICE_PYTHON:-python3}"
# Build only Atlas RPC/MCP bindings, never a ROS IDL overlay.
command -v rbnx >/dev/null || { echo 'rbnx is required for contract generation' >&2; exit 2; }
rbnx codegen -p "$PKG" --mcp --python "$PY"
set +u; source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"; set -u
for package in nav2_planner nav2_controller nav2_regulated_pure_pursuit_controller nav2_velocity_smoother nav2_lifecycle_manager pointcloud_to_laserscan; do
  ros2 pkg prefix "$package" >/dev/null || { echo "Missing ROS package: $package" >&2; exit 2; }
done
export PYTHONPATH="$(rbnx path robonix-api):${PYTHONPATH:-}"
"$PY" -c 'import numpy, yaml, PIL, rclpy, robonix_api, sensor_msgs_py, tf2_ros'
ros2 pkg prefix rtabmap_util >/dev/null
"$PY" -c 'import scipy, shapely, cv2, message_filters'
mkdir -p "$PKG/rbnx-build/camera_ws"
cd "$PKG/rbnx-build/camera_ws"
colcon build --base-paths "$PKG/src/outdoor_camera_layer" --packages-select outdoor_camera_layer
cd "$PKG"
"$PY" -m compileall -q outdoor_nav
mkdir -p rbnx-build
touch rbnx-build/.rbnx-built
