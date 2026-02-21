#!/bin/bash
# ──────────────────────────────────────────────────────────────────────
# startup_mapping.sh — Launch sensors + RTAB-Map SLAM + RViz2
#
# Starts the full mapping stack: D435i camera, WT901 IMU, EKF,
# depthimage_to_laserscan, and RTAB-Map SLAM in mapping mode.
#
# Usage:
#   ./scripts/startup_mapping.sh
#   ./scripts/startup_mapping.sh my_map_name
#   ./scripts/startup_mapping.sh my_map_name --no-rviz
# ──────────────────────────────────────────────────────────────────────
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

DB_NAME="${1:-my_map}"
RVIZ_CONFIG="$WS_DIR/install/ros2_wit_d435/share/ros2_wit_d435/config/sensors_rviz.rviz"

echo "🗺️  Starting SLAM mapping mode..."
echo "   Workspace: $WS_DIR"
echo "   Map name:  $DB_NAME"
echo "   Database:  ~/.ros/rtabmap/${DB_NAME}.db"

# Launch bringup in mapping mode
ros2 launch ros2_wit_d435 bringup.launch.py \
    mode:=mapping \
    baud_rate:=115200 \
    db_name:="$DB_NAME" &
LAUNCH_PID=$!

# Launch RViz2 unless --no-rviz is passed
if [[ "$1" != "--no-rviz" && "$2" != "--no-rviz" ]]; then
    sleep 3
    echo "🖥️  Starting RViz2..."
    rviz2 -d "$RVIZ_CONFIG" &
    RVIZ_PID=$!
fi

# Cleanup on Ctrl+C
cleanup() {
    echo ""
    echo "🛑 Shutting down... (RTAB-Map will save the database)"
    kill $LAUNCH_PID 2>/dev/null
    [[ -n "$RVIZ_PID" ]] && kill $RVIZ_PID 2>/dev/null
    wait 2>/dev/null
    echo "Map saved to: ~/.ros/rtabmap/${DB_NAME}.db"
    echo "Done."
}
trap cleanup SIGINT SIGTERM

wait $LAUNCH_PID
