#!/bin/bash

set -euo pipefail

source /opt/ros/noetic/setup.bash
source "$HOME/super_ws/devel/setup.bash"

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
unset ROS_IP

PX4_DIR="$HOME/PX4-RoboMaster"
GAZEBO_MODEL_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
WORLD_FILE="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/so101_test.world"

export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_DIR:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$HOME/super_ws/devel/lib:/opt/ros/noetic/lib:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="$HOME/super_ws/devel/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"

ROSCORE_PID=""
GAZEBO_PID=""

cleanup()
{
    echo
    echo "Stopping SO101 test simulation..."

    if [ -n "$GAZEBO_PID" ] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
        kill "$GAZEBO_PID" 2>/dev/null || true
        wait "$GAZEBO_PID" 2>/dev/null || true
    fi

    if [ -n "$ROSCORE_PID" ] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
        kill "$ROSCORE_PID" 2>/dev/null || true
        wait "$ROSCORE_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: world file not found:"
    echo "$WORLD_FILE"
    exit 1
fi

echo "=============================="
echo " SO101 standalone Gazebo test"
echo "=============================="

if ! rostopic list >/dev/null 2>&1; then
    echo "[1/2] Starting roscore..."

    roscore >/tmp/so101_roscore.log 2>&1 &
    ROSCORE_PID=$!

    for i in $(seq 1 20); do
        if rostopic list >/dev/null 2>&1; then
            break
        fi

        if ! kill -0 "$ROSCORE_PID" 2>/dev/null; then
            echo "ERROR: roscore exited."
            tail -n 100 /tmp/so101_roscore.log
            exit 1
        fi

        sleep 0.5
    done
else
    echo "[1/2] ROS master already running."
fi

echo "[2/2] Starting Gazebo Classic..."

roslaunch gazebo_ros empty_world.launch \
    world_name:="$WORLD_FILE" \
    paused:=false \
    use_sim_time:=true \
    gui:=true \
    verbose:=true \
    >/tmp/so101_gazebo.log 2>&1 &

GAZEBO_PID=$!

echo "Waiting for Gazebo ROS interface..."

GAZEBO_READY=false

for i in $(seq 1 60); do
    if rosservice list 2>/dev/null | grep -q "^/gazebo/get_model_state$"; then
        GAZEBO_READY=true
        break
    fi

    if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
        echo "ERROR: Gazebo exited unexpectedly."
        tail -n 150 /tmp/so101_gazebo.log
        exit 1
    fi

    sleep 1
done

if [ "$GAZEBO_READY" != true ]; then
    echo "ERROR: Gazebo did not become ready."
    tail -n 150 /tmp/so101_gazebo.log
    exit 1
fi

echo
echo "=============================="
echo " SO101 test environment ready"
echo "=============================="
echo "Joint states: /so101/joint_states"
echo "Gazebo log : /tmp/so101_gazebo.log"
echo
echo "Press Ctrl+C to stop."

wait "$GAZEBO_PID"