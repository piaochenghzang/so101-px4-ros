#!/bin/bash
# Reuse the existing ROS/SUPER chain and the guarded fixed-grasp demo.
set -eo pipefail
DEMO_ARGS=("$@")
# Catkin setup scripts inherit positional arguments from the caller.
set --
source /opt/ros/noetic/setup.bash
source "$HOME/super_ws/devel/setup.bash"
set -u
PX4_DIR="${PX4_DIR:-$HOME/PX4-RoboMaster}"
export PX4_SIM_SPEED_FACTOR="${PX4_SIM_SPEED_FACTOR:-.25}"
CASE_NAME="super_obstacle_$(date +%Y%m%d_%H%M%S)"
exec python3 -u "$PX4_DIR/src/modules/arm_control/tests/hover/run_known_target_flight.py" \
  "$CASE_NAME" --sim-vision --super-obstacle --execute-flight --execute-fixed-grasp "${DEMO_ARGS[@]}"
