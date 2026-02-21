#!/bin/bash
# ──────────────────────────────────────────────────────────────────────
# startup_sensors.sh — Launch sensors + RViz2
#
# Starts D435i camera, WT901 IMU, EKF, and depthimage_to_laserscan
# in one terminal, and RViz2 with the sensors config in another.
#
# Usage:
#   ./scripts/startup_sensors.sh
#   ./scripts/startup_sensors.sh --no-rviz
# ──────────────────────────────────────────────────────────────────────
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

RVIZ_CONFIG="$WS_DIR/install/ros2_wit_d435/share/ros2_wit_d435/config/sensors_rviz.rviz"

echo "🚀 Starting sensors (D435i + WT901 + EKF)..."
echo "   Workspace: $WS_DIR"

# Launch sensors
ros2 launch ros2_wit_d435 sensors.launch.py baud_rate:=115200 &
SENSOR_PID=$!

# Launch RViz2 unless --no-rviz is passed
if [[ "$1" != "--no-rviz" ]]; then
    sleep 3  # Wait for topics to appear
    echo "🖥️  Starting RViz2..."
    rviz2 -d "$RVIZ_CONFIG" &
    RVIZ_PID=$!
fi

# Cleanup on Ctrl+C
cleanup() {
    echo ""
    echo "🛑 Shutting down..."
    kill $SENSOR_PID 2>/dev/null
    [[ -n "$RVIZ_PID" ]] && kill $RVIZ_PID 2>/dev/null
    wait 2>/dev/null
    echo "Done."
}
trap cleanup SIGINT SIGTERM

wait $SENSOR_PID
