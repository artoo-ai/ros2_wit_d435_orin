#!/bin/bash
# ──────────────────────────────────────────────────────────────────────
# startup_navigation.sh — Launch sensors + Nav2 navigation + RViz2
#
# Starts the full navigation stack: D435i camera, WT901 IMU, EKF,
# depthimage_to_laserscan, and Nav2 with a pre-built map.
#
# Usage:
#   ./scripts/startup_navigation.sh /path/to/map.yaml
#   ./scripts/startup_navigation.sh /path/to/map.yaml --no-rviz
# ──────────────────────────────────────────────────────────────────────
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

MAP_FILE="${1:?Usage: $0 /path/to/map.yaml [--no-rviz]}"
RVIZ_CONFIG="$WS_DIR/install/ros2_wit_d435/share/ros2_wit_d435/config/sensors_rviz.rviz"

if [[ ! -f "$MAP_FILE" ]]; then
    echo "❌ Map file not found: $MAP_FILE"
    exit 1
fi

echo "🧭 Starting navigation mode..."
echo "   Workspace: $WS_DIR"
echo "   Map file:  $MAP_FILE"

# Launch bringup in navigation mode
ros2 launch ros2_wit_d435 bringup.launch.py \
    mode:=navigation \
    baud_rate:=115200 \
    map:="$MAP_FILE" &
LAUNCH_PID=$!

# Launch RViz2 unless --no-rviz is passed
if [[ "$2" != "--no-rviz" ]]; then
    sleep 3
    echo "🖥️  Starting RViz2..."
    rviz2 -d "$RVIZ_CONFIG" &
    RVIZ_PID=$!
fi

# Cleanup on Ctrl+C
cleanup() {
    echo ""
    echo "🛑 Shutting down..."
    kill $LAUNCH_PID 2>/dev/null
    [[ -n "$RVIZ_PID" ]] && kill $RVIZ_PID 2>/dev/null
    wait 2>/dev/null
    echo "Done."
}
trap cleanup SIGINT SIGTERM

wait $LAUNCH_PID
