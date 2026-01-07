#!/bin/bash

# ============================================================================
# Track Test Startup Script (Moving Target Mode)
# ============================================================================
# This script initializes the track test system within a tmux session.
# It splits the window into four panes:
#   - Pane 0: Offboard State Machine
#   - Pane 1: Target Publisher Node (circular motion)
#   - Pane 2: Track Test Node (Moving Target Mode, Focused)
#   - Pane 3: Data Recorder (ros2 bag record)
# ============================================================================

set -e

# Path Initialization
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Argument Parsing
DRONE_ID=${1:-0}
MODE=${2:-onboard}
SESSION_NAME="track_moving"

# Validate Mode
if [[ "$MODE" != "onboard" && "$MODE" != "sitl" ]]; then
    echo "ERROR: Mode must be 'onboard' or 'sitl'"
    exit 1
fi

# Environment Setup Commands
ROS2_SETUP="export ROS_DOMAIN_ID=86 && source /opt/ros/humble/setup.bash && source $WORKSPACE_DIR/install/setup.bash"
FSM_CONFIG="$WORKSPACE_DIR/src/offboard_state_machine/config/fsm_track.yaml"

# Data recording directory
FLY_LOG_DIR="/home/radxa/RealFlight/fly_log"
mkdir -p "$FLY_LOG_DIR"

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
echo "Starting Track Test System (Moving Target)"
echo "Drone ID: $DRONE_ID | Mode: $MODE | use_sim_time: $USE_SIM_TIME"
echo "======================================"

# 1. Create Session and start State Machine (Pane 0)
echo "Launching State Machine..."
tmux new-session -d -s "$SESSION_NAME" -n "mission" \
    "$ROS2_SETUP && ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=$DRONE_ID mode:=$MODE config_file:=$FSM_CONFIG; exec bash"

# Small delay to ensure the first pane is initialized
sleep 0.5

# 2. Split window horizontally and start Target Publisher (Pane 1)
echo "Splitting window and launching Target Publisher..."
tmux split-window -h -t "$SESSION_NAME" \
    "$ROS2_SETUP && ros2 launch track_test target_publisher.launch.py \
    use_sim_time:=$USE_SIM_TIME; exec bash"

# Small delay to ensure the second pane is initialized
sleep 0.5

# 3. Split window vertically and start Track Test (Pane 2, focused)
# Split the right pane (Pane 1) vertically, creating Pane 2 below it
echo "Splitting window and launching Track Test Node (Moving Target Mode)..."
tmux split-window -v -t "$SESSION_NAME" \
    "$ROS2_SETUP && ros2 launch track_test track_test.launch.py \
    drone_id:=$DRONE_ID mode:=$MODE use_target_topic:=true; exec bash"

# Small delay to ensure the third pane is initialized
sleep 0.5

# 4. Split window and start Data Recorder (Pane 3)
# Generate timestamp for unique bag file name
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="track_test_data_${TIMESTAMP}"
BAG_PATH="$FLY_LOG_DIR/$BAG_NAME"

echo "Splitting window and launching Data Recorder..."
tmux split-window -v -t "$SESSION_NAME" \
    "$ROS2_SETUP && cd $FLY_LOG_DIR && ros2 bag record \
    /fmu/out/vehicle_odometry \
    /target/position \
    /target/velocity \
    -o $BAG_NAME; exec bash"

# 5. Optional: Select layout for better pane distribution
# Use tiled layout for equal-sized panes
tmux select-layout -t "$SESSION_NAME" tiled

echo "--------------------------------------"
echo "Startup Successful!"
echo "Pane 0 (Top-Left): State Machine"
echo "Pane 1 (Top-Right): Target Publisher"
echo "Pane 2 (Bottom-Left): Track Test Node (Moving Target, Focused)"
echo "Pane 3 (Bottom-Right): Data Recorder"
echo "Data will be saved to: $BAG_PATH"
echo "--------------------------------------"
echo "Attaching to session in 1 second..."
sleep 1

tmux attach-session -t "$SESSION_NAME"

