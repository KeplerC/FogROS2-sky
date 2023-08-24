#!/bin/bash
set -e

# setup ros2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
source $ROS_WS/install/setup.bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc

# work with CycloneDDS DDS implementation
ver=$(lsb_release -rs | sed 's/\.//')
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

exec "$@"