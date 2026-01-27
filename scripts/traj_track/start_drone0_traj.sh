#!/bin/bash

# ============================================================================
# Drone 0: Traj Test Startup Script (Target Publisher)
# ============================================================================
# This script starts drone0 with traj_test node that publishes target position
# and velocity to /target/position and /target/velocity topics.
# 
# Usage:
#   ./start_drone0_traj.sh [mode]
#   mode: onboard (default) or sitl
# ============================================================================

set -e

# Path Initialization
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPTS_DIR/.." && pwd)"

# Argument Parsing
MODE=${1:-onboard}
DRONE_ID=0
SESSION_NAME="drone0_traj"

# Validate Mode
if [[ "$MODE" != "onboard" && "$MODE" != "sitl" ]]; then
    echo "ERROR: Mode must be 'onboard' or 'sitl'"
    exit 1
fi

# Environment Setup Commands
ROS2_SETUP="export ROS_DOMAIN_ID=86 && source /opt/ros/humble/setup.bash && source $WORKSPACE_DIR/install/setup.bash"
FSM_CONFIG="$WORKSPACE_DIR/src/offboard_state_machine/config/fsm_traj.yaml"

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
echo "Starting Drone 0: Traj Test (Target Publisher)"
echo "======================================"
echo "Drone ID: $DRONE_ID | Mode: $MODE"
echo "Publishes: /target/position, /target/velocity"
echo "======================================"

# 1. Create Session and start State Machine (Left Pane - Pane 0)
echo "Launching State Machine..."
tmux new-session -d -s "$SESSION_NAME" -n "mission" \
    "$ROS2_SETUP && ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=$DRONE_ID mode:=$MODE config_file:=$FSM_CONFIG; exec bash"

# Small delay to ensure the first pane is initialized
sleep 0.5

# 2. Split window and start Trajectory Test (Right Pane - Pane 1)
echo "Splitting window and launching Traj Test Node..."
tmux split-window -h -t "$SESSION_NAME" \
    "$ROS2_SETUP && ros2 launch traj_test traj_test.launch.py \
    drone_id:=$DRONE_ID mode:=$MODE; exec bash"

# 3. Optional: Equalize pane sizes
tmux select-layout -t "$SESSION_NAME" even-horizontal

echo ""
echo "--------------------------------------"
echo "Startup Successful!"
echo "--------------------------------------"
echo "Pane 0 (Left): State Machine (drone $DRONE_ID)"
echo "Pane 1 (Right): Traj Test Node (drone $DRONE_ID)"
echo ""
echo "This drone publishes target position/velocity to:"
echo "  - /target/position"
echo "  - /target/velocity"
echo "--------------------------------------"
echo "Attaching to session in 1 second..."
sleep 1

tmux attach-session -t "$SESSION_NAME"

