#!/bin/bash

# ============================================================================
# Vicon to Target Converter Startup Script
# ============================================================================
# This script starts vicon_to_target_node that converts Vicon position data
# to target position and velocity topics (/target/position and /target/velocity).
# 
# Usage:
#   ./start_vicon_to_target.sh [vicon_topic_name] [mode] [vicon_topic_type]
#   vicon_topic_name: Vicon topic name (default: /vicon/pose)
#   mode: onboard (default) or sitl
#   vicon_topic_type: PoseStamped (default) or TransformStamped
# 
# Examples:
#   ./start_vicon_to_target.sh /vicon/drone1/pose onboard PoseStamped
#   ./start_vicon_to_target.sh /vicon/object1/transform sitl TransformStamped
# ============================================================================

set -e

# Path Initialization
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPTS_DIR/.." && pwd)"

# Argument Parsing
VICON_TOPIC_NAME=${1:-/vicon/pose}
MODE=${2:-onboard}
VICON_TOPIC_TYPE=${3:-PoseStamped}

# Generate session name from topic name (replace / with _)
SESSION_NAME="vicon_to_target_$(echo $VICON_TOPIC_NAME | tr '/' '_' | tr '-' '_')"

# Validate Mode
if [[ "$MODE" != "onboard" && "$MODE" != "sitl" ]]; then
    echo "ERROR: Mode must be 'onboard' or 'sitl'"
    exit 1
fi

# Validate Topic Type
if [[ "$VICON_TOPIC_TYPE" != "PoseStamped" && "$VICON_TOPIC_TYPE" != "TransformStamped" ]]; then
    echo "ERROR: Vicon topic type must be 'PoseStamped' or 'TransformStamped'"
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
echo "Starting Vicon to Target Converter"
echo "======================================"
echo "Vicon Topic: $VICON_TOPIC_NAME"
echo "Topic Type: $VICON_TOPIC_TYPE"
echo "Mode: $MODE"
echo "Publishes: /target/position, /target/velocity"
echo "======================================"

# Create Session and start Vicon to Target Node
echo "Launching Vicon to Target Node..."
tmux new-session -d -s "$SESSION_NAME" -n "vicon_converter" \
    "$ROS2_SETUP && ros2 launch track_test vicon_to_target.launch.py \
    vicon_topic_name:=$VICON_TOPIC_NAME \
    vicon_topic_type:=$VICON_TOPIC_TYPE; exec bash"

echo ""
echo "--------------------------------------"
echo "Startup Successful!"
echo "--------------------------------------"
echo "Session: $SESSION_NAME"
echo "Vicon Topic: $VICON_TOPIC_NAME ($VICON_TOPIC_TYPE)"
echo ""
echo "This node converts Vicon data to target topics:"
echo "  - Subscribes: $VICON_TOPIC_NAME"
echo "  - Publishes: /target/position"
echo "  - Publishes: /target/velocity"
echo ""
echo "Note: Make sure Vicon system is running and publishing to $VICON_TOPIC_NAME!"
echo "--------------------------------------"
echo "Attaching to session in 1 second..."
sleep 1

tmux attach-session -t "$SESSION_NAME"

