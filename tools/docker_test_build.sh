#!/usr/bin/env bash

# Run this from the ros workspace source dir
set -e

BASE_IMAGE=ros:foxy-ros-base
echo "*** Building project on docker on image ${BASE_IMAGE}"
docker run -it --volume=$(pwd):/ros_ws/src ${BASE_IMAGE} bash -c " \\
    cd /ros_ws/src/last_letter && \\
    ./tools/install_dependencies.sh && \\
    ./tools/build_project.sh && \\
    ./tools/install_project.sh && \\
    ls ~/last_letter_models
"

