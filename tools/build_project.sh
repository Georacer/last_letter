#!/usr/bin/env bash
# Run this script from the source root directory

set -ex

# Set current working directory to the directory of the script
cd "$(dirname "$0")"
cd ../../..
ROS_WS_DIR=$(pwd)

# Compile the last_leter ROS2 package
echo ""
echo "*** Compiling last_letter"
cd ${ROS_WS_DIR}
source /opt/ros/foxy/setup.bash
export LC_NUMERIC="en_US.UTF-8"
export LANG="en_US.UTF-8"
export ROSCONSOLE_FORMAT='[${severity}] [${time}] [${node}]: ${message}'
rm -rf build install log

colcon build --symlink-install
