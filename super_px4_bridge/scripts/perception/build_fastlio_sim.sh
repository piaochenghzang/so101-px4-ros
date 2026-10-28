#!/usr/bin/env bash
# Compile existing FAST-LIO for Gazebo. Official Livox messages only; no SDK/device driver.
set -eo pipefail
source /opt/ros/noetic/setup.bash
set -u
PX4_DIR="${PX4_DIR:-$HOME/PX4-RoboMaster}"
FASTLIO_DIR="${FASTLIO_DIR:-$HOME/FAST_LIO}"
export PX4_DIR FASTLIO_DIR
python3 - <<'PYTHON'
import os,shutil,subprocess
from pathlib import Path
root=Path(os.environ['PX4_DIR'])/'build';source=Path(os.environ['FASTLIO_DIR']).resolve()
if not (source/'include/ikd-Tree/ikd_Tree.cpp').is_file():raise SystemExit('Existing FAST-LIO with ikd-Tree submodule is required')
dep=root/'fastlio_dependencies/livox_ros_driver'
revision='3d240d5666129e1a3052e78ee8487a04b08fdda3'
if not dep.exists():
    dep.parent.mkdir(parents=True,exist_ok=True)
    subprocess.run(['git','clone','https://github.com/Livox-SDK/livox_ros_driver.git',str(dep)],check=True)
    subprocess.run(['git','-C',str(dep),'checkout','--detach',revision],check=True)
actual=subprocess.check_output(['git','-C',str(dep),'rev-parse','HEAD'],text=True).strip()
if actual!=revision:raise SystemExit('Livox dependency revision differs; review existing checkout')
src=root/'fastlio_ws/src';src.mkdir(parents=True,exist_ok=True)
link=src/'FAST_LIO'
if link.exists() and link.resolve()!=source:raise SystemExit('Existing FAST-LIO workspace points elsewhere')
if not link.exists():link.symlink_to(source,target_is_directory=True)
pkg=src/'livox_ros_driver';pkg.mkdir(exist_ok=True)
shutil.copytree(dep/'livox_ros_driver/msg',pkg/'msg',dirs_exist_ok=True)
shutil.copy2(dep/'LICENSE.txt',pkg/'LICENSE.txt')
(pkg/'package.xml').write_text('<package format="2"><name>livox_ros_driver</name><version>2.6.0</version><description>Official Livox messages only, simulation build dependency.</description><maintainer email="simulation@example.invalid">Simulation</maintainer><license>MIT</license><buildtool_depend>catkin</buildtool_depend><build_depend>message_generation</build_depend><depend>std_msgs</depend><exec_depend>message_runtime</exec_depend></package>')
(pkg/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.0.2)\nproject(livox_ros_driver)\nfind_package(catkin REQUIRED COMPONENTS message_generation std_msgs)\nadd_message_files(FILES CustomMsg.msg CustomPoint.msg)\ngenerate_messages(DEPENDENCIES std_msgs)\ncatkin_package(CATKIN_DEPENDS message_runtime std_msgs)\n')
print('FAST-LIO source:',source)
print('FAST-LIO revision:',subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip())
PYTHON
catkin_make -C "$PX4_DIR/build/fastlio_ws" fast_lio_generate_messages livox_ros_driver_generate_messages -j1
catkin_make -C "$PX4_DIR/build/fastlio_ws" fastlio_mapping -j1
