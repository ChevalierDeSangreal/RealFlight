#!/bin/bash

# ============================================================================
# Vicon to Target Converter Startup Script
# ============================================================================
# This script starts vicon_to_target_node that converts Vicon position data
# to target position and velocity topics (/target/position and /target/velocity).
# 
# Usage:
#   ./start_vicon_to_target.sh [px4_topic_name] [mode]
#   px4_topic_name: PX4 VehicleOdometry topic name (default: /px4_3/fmu/out/vehicle_odometry)
#   mode: onboard (default) or sitl
# 
# Examples:
#   ./start_vicon_to_target.sh  # Uses default: /px4_3/fmu/out/vehicle_odometry
#   ./start_vicon_to_target.sh /px4_3/fmu/out/vehicle_odometry onboard
#   ./start_vicon_to_target.sh /px4_0/fmu/out/vehicle_odometry sitl
# ============================================================================

set -e

# Path Initialization
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPTS_DIR/.." && pwd)"

# Argument Parsing
PX4_TOPIC_NAME=${1:-/px4_3/fmu/out/vehicle_odometry}
MODE=${2:-onboard}

# Generate session name from topic name (replace / with _)
SESSION_NAME="vicon_to_target_$(echo $PX4_TOPIC_NAME | tr '/' '_' | tr '-' '_')"

# Validate Mode
if [[ "$MODE" != "onboard" && "$MODE" != "sitl" ]]; then
    echo "ERROR: Mode must be 'onboard' or 'sitl'"
    exit 1
fi


# Environment Setup Commands
ROS2_SETUP="export ROS_DOMAIN_ID=86 && source /opt/ros/humble/setup.bash && source $WORKSPACE_DIR/install/setup.bash"

# Kill existing session if it exists
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "Existing session '$SESSION_NAME' found. Killing it..."
    tmux kill-session -t "$SESSION_NAME"
fi

echo "======================================"
echo "Starting PX4 to Target Converter"
echo "======================================"
echo "PX4 Topic: $PX4_TOPIC_NAME"
echo "Mode: $MODE"
echo "Publishes: /target/position, /target/velocity"
echo "======================================"

# Create Session and start PX4 to Target Node
echo "Launching PX4 to Target Node..."
tmux new-session -d -s "$SESSION_NAME" -n "px4_converter" \
    "$ROS2_SETUP && ros2 launch track_test vicon_to_target.launch.py \
    px4_topic_name:=$PX4_TOPIC_NAME; exec bash"

echo ""
echo "--------------------------------------"
echo "Startup Successful!"
echo "--------------------------------------"
echo "Session: $SESSION_NAME"
echo "PX4 Topic: $PX4_TOPIC_NAME"
echo ""
echo "This node converts PX4 odometry data to target topics:"
echo "  - Subscribes: $PX4_TOPIC_NAME (PX4 VehicleOdometry)"
echo "  - Publishes: /target/position"
echo "  - Publishes: /target/velocity"
echo ""
echo "Note: Make sure PX4 is running and publishing to $PX4_TOPIC_NAME!"
echo "--------------------------------------"
echo "Attaching to session in 1 second..."
sleep 1

tmux attach-session -t "$SESSION_NAME"

