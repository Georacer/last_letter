#!/usr/bin/env bash

set -ex
# Avoid getting asked for prompts
export DEBIAN_FRONTEND=noninteractive

# Set current working directory to the ros_ws directory
cd "$(dirname "$0")"/../../..

# Install whatever rosdep can cover
apt-get update
rosdep update
rosdep install -i --from-path . --rosdistro ${ROS_DISTRO} -y

# Install system dependencies not covered by rosdep
apt-get install -y libignition-common3-graphics
## Add 3rd party repositories
apt-get install software-properties-common -y
### iir1
add-apt-repository ppa:berndporr/dsp -y
## Install packages
apt-get update && apt-get install -y\
    iir1-dev \
    python3-pip

# Install pip packages
pip3 install -r src/last_letter/last_letter/external/last_letter_lib/tools/requirements.txt
pip3 install -r src/last_letter/tools/requirements.txt
