#!/bin/bash

# ============================================================================
# Traj-Track Multi-Drone Startup Script
# ============================================================================
# This script starts:
#   - drone0: traj_test (publishes target position/velocity)
#   - drone3: track_test (tracks target from drone0)
# ============================================================================

set -e

# Path Initialization
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Argument Parsing
MODE=${1:-onboard}
DRONE_ID_TRAJ=${2:-0}  # drone0 for traj
DRONE_ID_TRACK=${3:-3}  # drone3 for track

# Validate Mode
if [[ "$MODE" != "onboard" && "$MODE" != "sitl" ]]; then
    echo "ERROR: Mode must be 'onboard' or 'sitl'"
    exit 1
fi

# Environment Setup Commands
ROS2_SETUP="export ROS_DOMAIN_ID=86 && source /opt/ros/humble/setup.bash && source $WORKSPACE_DIR/install/setup.bash"
FSM_CONFIG_TRAJ="$WORKSPACE_DIR/src/offboard_state_machine/config/fsm_traj.yaml"
FSM_CONFIG_TRACK="$WORKSPACE_DIR/src/offboard_state_machine/config/fsm_track.yaml"

# Set use_sim_time based on mode
if [[ "$MODE" == "sitl" ]]; then
    USE_SIM_TIME="true"
else
    USE_SIM_TIME="false"
fi

echo "======================================"
echo "Starting Traj-Track Multi-Drone System"
echo "======================================"
echo "Mode: $MODE"
echo "Drone $DRONE_ID_TRAJ: traj_test (target publisher)"
echo "Drone $DRONE_ID_TRACK: track_test (target tracker)"
echo "======================================"

# Kill existing sessions if they exist
if tmux has-session -t "traj_track" 2>/dev/null; then
    echo "Existing session 'traj_track' found. Killing it..."
    tmux kill-session -t "traj_track"
fi

# ============================================================================
# Drone 0: Traj Test (Target Publisher)
# ============================================================================
echo ""
echo "--- Starting Drone $DRONE_ID_TRAJ (traj_test) ---"

# Create session with drone0 state machine
tmux new-session -d -s "traj_track" -n "drone${DRONE_ID_TRAJ}_fsm" \
    "$ROS2_SETUP && ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=$DRONE_ID_TRAJ mode:=$MODE config_file:=$FSM_CONFIG_TRAJ; exec bash"

sleep 0.5

# Split and add drone0 traj_test
tmux split-window -h -t "traj_track" \
    "$ROS2_SETUP && ros2 launch traj_test traj_test.launch.py \
    drone_id:=$DRONE_ID_TRAJ mode:=$MODE; exec bash"

sleep 0.5

# ============================================================================
# Drone 3: Track Test (Target Tracker)
# ============================================================================
echo "--- Starting Drone $DRONE_ID_TRACK (track_test) ---"

# Create new window for drone3
tmux new-window -t "traj_track" -n "drone${DRONE_ID_TRACK}_fsm" \
    "$ROS2_SETUP && ros2 launch offboard_state_machine single_drone_test.launch.py \
    drone_id:=$DRONE_ID_TRACK mode:=$MODE config_file:=$FSM_CONFIG_TRACK; exec bash"

sleep 0.5

# Split and add drone3 track_test (with use_target_topic:=true)
tmux split-window -h -t "traj_track" \
    "$ROS2_SETUP && ros2 launch track_test track_test.launch.py \
    drone_id:=$DRONE_ID_TRACK mode:=$MODE use_target_topic:=true; exec bash"

sleep 0.5

# Select first window
tmux select-window -t "traj_track:drone${DRONE_ID_TRAJ}_fsm"

echo ""
echo "--------------------------------------"
echo "Startup Successful!"
echo "--------------------------------------"
echo "Window 0 (drone${DRONE_ID_TRAJ}_fsm):"
echo "  - Pane 0: State Machine (drone $DRONE_ID_TRAJ)"
echo "  - Pane 1: Traj Test Node (drone $DRONE_ID_TRAJ) - publishes /target/position"
echo ""
echo "Window 1 (drone${DRONE_ID_TRACK}_fsm):"
echo "  - Pane 0: State Machine (drone $DRONE_ID_TRACK)"
echo "  - Pane 1: Track Test Node (drone $DRONE_ID_TRACK) - subscribes /target/position"
echo ""
echo "Use 'tmux list-windows' to see all windows"
echo "Use 'tmux select-window -t <window_name>' to switch windows"
echo "--------------------------------------"
echo "Attaching to session in 1 second..."
sleep 1

tmux attach-session -t "traj_track"

