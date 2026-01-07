#!/bin/bash

# ============================================================================
# Track Test Startup Script (Static Target Mode)
# ============================================================================
# This script initializes the track test system within a tmux session.
# It splits the window into two panes:
#   - Left Pane: Track Test Node (Static Target Mode)
#   - Right Pane: Offboard State Machine (Focused)
# ============================================================================

set -e

# Path Initialization
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Argument Parsing
DRONE_ID=${1:-0}
MODE=${2:-onboard}
SESSION_NAME="track_stable"

# Validate Mode
if [[ "$MODE" != "onboard" && "$MODE" != "sitl" ]]; then
    echo "ERROR: Mode must be 'onboard' or 'sitl'"
    exit 1
fi

# Environment Setup Commands
ROS2_SETUP="export ROS_DOMAIN_ID=0 && source /opt/ros/humble/setup.bash && source $WORKSPACE_DIR/install/setup.bash"
FSM_CONFIG="$WORKSPACE_DIR/src/offboard_state_machine/config/fsm_track_stable.yaml"

# Kill existing session if it exists
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "Existing session '$SESSION_NAME' found. Killing it..."
    tmux kill-session -t "$SESSION_NAME"
fi

echo "======================================"
echo "Starting Track Test System (Static Target)"
echo "Drone ID: $DRONE_ID | Mode: $MODE"
echo "======================================"

# 1. Create Session and start Track Test (Left Pane - Pane 0)
# Static mode: use_target_topic:=false (tracks stationary target in front of drone)
echo "Launching Track Test Node (Static Target Mode)..."
tmux new-session -d -s "$SESSION_NAME" -n "mission" \
    "$ROS2_SETUP && ros2 launch track_test track_test.launch.py \
    drone_id:=$DRONE_ID mode:=$MODE use_target_topic:=false; exec bash"

# Small delay to ensure the first pane is initialized
sleep 0.5

# 2. Split window and start State Machine (Right Pane - Pane 1)
# tmux split-window automatically moves focus to the newly created pane
echo "Splitting window and launching State Machine..."
tmux split-window -h -t "$SESSION_NAME" \
    "$ROS2_SETUP && ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=$DRONE_ID mode:=$MODE config_file:=$FSM_CONFIG; exec bash"

# 3. Optional: Equalize pane sizes
tmux select-layout -t "$SESSION_NAME" even-horizontal

echo "--------------------------------------"
echo "Startup Successful!"
echo "Pane 0 (Left): Track Test Node (Static Target)"
echo "Pane 1 (Right): State Machine (Focused)"
echo "--------------------------------------"
echo "Attaching to session in 1 second..."
sleep 1

tmux attach-session -t "$SESSION_NAME"

