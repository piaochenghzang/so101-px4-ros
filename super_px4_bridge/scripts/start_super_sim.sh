#!/bin/bash

set -euo pipefail

source /opt/ros/noetic/setup.bash
source "$HOME/super_ws/devel/setup.bash"

export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
unset ROS_IP

GAZEBO_SYSTEM_PLUGIN_DIR=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins

export LD_LIBRARY_PATH="$HOME/super_ws/devel/lib:/opt/ros/noetic/lib:$GAZEBO_SYSTEM_PLUGIN_DIR:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="$HOME/super_ws/devel/lib:/opt/ros/noetic/lib:$GAZEBO_SYSTEM_PLUGIN_DIR:${GAZEBO_PLUGIN_PATH:-}"

PX4_DIR="$HOME/PX4-RoboMaster"
WALL_FILE="$HOME/super_ws/src/super_px4_bridge/models/super_wall.sdf"

ROSCORE_PID=""
PX4_PID=""
MAVROS_PID=""


# ============================================================
# Clean residual Gazebo Classic processes
# ============================================================

clean_gazebo()
{
    echo "Checking for residual Gazebo processes..."

    # Try graceful shutdown first
    pkill -TERM -x gzclient 2>/dev/null || true
    pkill -TERM -x gzserver 2>/dev/null || true

    sleep 1

    # Force kill if they are still alive
    pkill -KILL -x gzclient 2>/dev/null || true
    pkill -KILL -x gzserver 2>/dev/null || true

    sleep 1

    echo "Gazebo residual processes cleared."
}


# ============================================================
# Cleanup when Ctrl+C / exit / error
# ============================================================

cleanup()
{
    echo
    echo "================================"
    echo " Stopping SUPER simulation..."
    echo "================================"

    # Stop MAVROS
    if [ -n "$MAVROS_PID" ] && kill -0 "$MAVROS_PID" 2>/dev/null; then
        echo "Stopping MAVROS..."
        kill "$MAVROS_PID" 2>/dev/null || true
        wait "$MAVROS_PID" 2>/dev/null || true
    fi

    # Stop PX4 make process
    if [ -n "$PX4_PID" ] && kill -0 "$PX4_PID" 2>/dev/null; then
        echo "Stopping PX4 SITL..."
        kill "$PX4_PID" 2>/dev/null || true
        wait "$PX4_PID" 2>/dev/null || true
    fi

    # Gazebo may survive after the PX4 parent process exits
    clean_gazebo

    # Only stop roscore if this script started it
    if [ -n "$ROSCORE_PID" ] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
        echo "Stopping roscore..."
        kill "$ROSCORE_PID" 2>/dev/null || true
        wait "$ROSCORE_PID" 2>/dev/null || true
    fi

    echo "Simulation stopped."
}

trap cleanup EXIT INT TERM


echo "=============================="
echo " SUPER simulation startup"
echo "=============================="


# ------------------------------------------------------------
# 0. Remove residual Gazebo processes
# ------------------------------------------------------------

echo "[0/4] Cleaning old Gazebo processes..."

clean_gazebo


# ------------------------------------------------------------
# 1. Start ROS master
# ------------------------------------------------------------

if ! rostopic list >/dev/null 2>&1; then

    echo "[1/4] Starting roscore..."

    roscore >/tmp/super_roscore.log 2>&1 &
    ROSCORE_PID=$!

    for i in $(seq 1 20); do

        if rostopic list >/dev/null 2>&1; then
            break
        fi

        if ! kill -0 "$ROSCORE_PID" 2>/dev/null; then
            echo "ERROR: roscore exited."
            tail -n 100 /tmp/super_roscore.log
            exit 1
        fi

        sleep 0.5
    done

else

    echo "[1/4] ROS master already running."

fi


# ------------------------------------------------------------
# 2. Start PX4 + Gazebo Classic
# ------------------------------------------------------------

echo "[2/4] Starting PX4 SITL + yhang550..."

cd "$PX4_DIR"

NO_PXH=1 make px4_sitl gazebo-classic_yhang550 \
    >/tmp/super_px4_gazebo.log 2>&1 &

PX4_PID=$!


# ------------------------------------------------------------
# Wait for Gazebo ROS interface
# ------------------------------------------------------------

echo "Waiting for Gazebo..."

GAZEBO_READY=false

for i in $(seq 1 60); do

    if rosservice list 2>/dev/null \
        | grep -q "^/gazebo/spawn_sdf_model$"; then

        GAZEBO_READY=true
        break
    fi

    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        echo "ERROR: PX4/Gazebo exited unexpectedly."
        tail -n 150 /tmp/super_px4_gazebo.log
        exit 1
    fi

    sleep 1

done


if [ "$GAZEBO_READY" != true ]; then

    echo "ERROR: Gazebo did not become ready."

    tail -n 150 /tmp/super_px4_gazebo.log
    exit 1

fi

echo "Gazebo is ready."


# ------------------------------------------------------------
# Wait for Velodyne
# ------------------------------------------------------------

echo "Waiting for Velodyne point cloud..."

LIDAR_READY=false

for i in $(seq 1 60); do

    if rostopic type /velodyne_points 2>/dev/null \
        | grep -q "sensor_msgs/PointCloud2"; then

        LIDAR_READY=true
        break

    fi

    if ! kill -0 "$PX4_PID" 2>/dev/null; then

        echo "ERROR: PX4/Gazebo exited unexpectedly."

        tail -n 100 /tmp/super_px4_gazebo.log
        exit 1

    fi

    sleep 1

done


if [ "$LIDAR_READY" != true ]; then

    echo "ERROR: /velodyne_points was not available after 60 seconds."

    grep -niE \
        'velodyne|failed to load|not found|RayPlugin|fatal|error' \
        /tmp/super_px4_gazebo.log \
        | tail -n 100

    exit 1

fi

echo "Velodyne point cloud is ready."


# ------------------------------------------------------------
# 3. Start MAVROS
# ------------------------------------------------------------

echo "[3/4] Starting MAVROS..."

roslaunch mavros px4.launch \
    fcu_url:="udp://:14540@127.0.0.1:14580" \
    >/tmp/super_mavros.log 2>&1 &

MAVROS_PID=$!


echo "Waiting for MAVROS connection..."

while true; do

    STATE=$(
        timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null \
        | grep "connected:" || true
    )

    if echo "$STATE" | grep -q "True"; then
        break
    fi

    if ! kill -0 "$MAVROS_PID" 2>/dev/null; then

        echo "ERROR: MAVROS exited unexpectedly."

        tail -n 100 /tmp/super_mavros.log
        exit 1

    fi

    if ! kill -0 "$PX4_PID" 2>/dev/null; then

        echo "ERROR: PX4/Gazebo exited unexpectedly."

        tail -n 100 /tmp/super_px4_gazebo.log
        exit 1

    fi

    sleep 1

done

echo "MAVROS connected."


# ------------------------------------------------------------
# 4. Spawn obstacle
# ------------------------------------------------------------

echo "[4/4] Spawning SUPER wall..."

if [ ! -f "$WALL_FILE" ]; then

    echo "ERROR: wall file not found:"
    echo "$WALL_FILE"

    exit 1

fi


rosrun gazebo_ros spawn_model \
    -sdf \
    -file "$WALL_FILE" \
    -model super_wall \
    -x 4.0 \
    -y 1.0 \
    -z 1.5


echo
echo "=============================="
echo " Simulation ready"
echo "=============================="
echo "PX4 + Gazebo : running"
echo "MAVROS       : connected"
echo "Velodyne     : /velodyne_points ready"
echo "Obstacle     : spawned"
echo
echo "Logs:"
echo "  PX4/Gazebo : /tmp/super_px4_gazebo.log"
echo "  MAVROS     : /tmp/super_mavros.log"
echo
echo "Press Ctrl+C to stop."


wait "$PX4_PID"