#!/usr/bin/env bash
# Simulation-only: ideal LiDAR target snapshot + simulated localization + native guarded grasp.
set -eo pipefail
DEMO_ARGS=("$@"); set --
source /opt/ros/noetic/setup.bash
source "$HOME/super_ws/devel/setup.bash"
set -u
PX4_DIR="${PX4_DIR:-$HOME/PX4-RoboMaster}"
export PX4_SIM_SPEED_FACTOR="${PX4_SIM_SPEED_FACTOR:-.5}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
CASE_NAME="lidar_grasp_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)"
echo "Result directory: $PX4_DIR/build/known_target_flight/$CASE_NAME"
exec python3 -u "$PX4_DIR/src/modules/arm_control/tests/hover/run_known_target_flight.py" \
  "$CASE_NAME" --sim-vision --lidar-target --execute-flight --execute-fixed-grasp "${DEMO_ARGS[@]}"
