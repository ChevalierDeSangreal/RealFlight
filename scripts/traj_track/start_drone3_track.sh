#!/bin/bash

# ============================================================================
# Drone 3: Track Test Startup Script (Target Tracker)
# ============================================================================
# This script starts drone3 with track_test node that subscribes to target
# position and velocity from /target/position and /target/velocity topics
# (published by drone0's traj_test node).
# 
# Usage:
#   ./start_drone3_track.sh [mode]
#   mode: onboard (default) or sitl
# ============================================================================

set -e

# Path Initialization
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPTS_DIR/.." && pwd)"

# Argument Parsing
MODE=${1:-onboard}
DRONE_ID=3
SESSION_NAME="drone3_track"

# Validate Mode
if [[ "$MODE" != "onboard" && "$MODE" != "sitl" ]]; then
    echo "ERROR: Mode must be 'onboard' or 'sitl'"
    exit 1
fi

# Environment Setup Commands
ROS2_SETUP="export ROS_DOMAIN_ID=86 && source /opt/ros/humble/setup.bash && source $WORKSPACE_DIR/install/setup.bash"
FSM_CONFIG="$WORKSPACE_DIR/src/offboard_state_machine/config/fsm_track.yaml"

# Set use_sim_time based on mode
if [[ "$MODE" == "sitl" ]]; then
    USE_SIM_TIME="true"
else
    USE_SIM_TIME="false"
fi

# Kill existing session if it exists
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "Existing session '$SESSION_NAME' found. Killing it..."
    tmux kill-session -t "$SESSION_NAME"
fi

echo "======================================"
echo "Starting Drone 3: Track Test (Target Tracker)"
echo "======================================"
echo "Drone ID: $DRONE_ID | Mode: $MODE"
echo "Subscribes: /target/position, /target/velocity"
echo "======================================"

# 1. Create Session and start State Machine (Left Pane - Pane 0)
echo "Launching State Machine..."
tmux new-session -d -s "$SESSION_NAME" -n "mission" \
    "$ROS2_SETUP && ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=$DRONE_ID mode:=$MODE config_file:=$FSM_CONFIG; exec bash"

# Small delay to ensure the first pane is initialized
sleep 0.5

# 2. Split window and start Track Test (Right Pane - Pane 1)
# IMPORTANT: use_target_topic:=true to subscribe to /target/position and /target/velocity
echo "Splitting window and launching Track Test Node..."
tmux split-window -h -t "$SESSION_NAME" \
    "$ROS2_SETUP && ros2 launch track_test track_test.launch.py \
    drone_id:=$DRONE_ID mode:=$MODE use_target_topic:=true; exec bash"

# 3. Optional: Equalize pane sizes
tmux select-layout -t "$SESSION_NAME" even-horizontal

echo ""
echo "--------------------------------------"
echo "Startup Successful!"
echo "--------------------------------------"
echo "Pane 0 (Left): State Machine (drone $DRONE_ID)"
echo "Pane 1 (Right): Track Test Node (drone $DRONE_ID)"
echo ""
echo "This drone subscribes to target position/velocity from:"
echo "  - /target/position (published by drone0)"
echo "  - /target/velocity (published by drone0)"
echo ""
echo "Note: Make sure drone0 is running and publishing target topics!"
echo "--------------------------------------"
echo "Attaching to session in 1 second..."
sleep 1

tmux attach-session -t "$SESSION_NAME"

