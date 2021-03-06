#!/bin/bash

#########################################################################
# STOP: If the dependency you want to add is required for the project   #
#       to build, it should be added as a rosdep (ie. a dependency      #
#       specified in one of the packages `package.xml` files).          #
#       This script should only contain other dependecies, like         #
#       external packages or utilities                                  #
#########################################################################

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "================================================================" 
echo "Installing ROS Melodic"
echo "================================================================"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update -y
sudo apt-get install ros-melodic-desktop-full -y

echo "================================================================"
echo "Installing Project Dependent ROS packages."
echo "================================================================"

# Setup the workspace
# Setup directory for pulling external pkgs
# Download packages from merged .rosinstall files
cd $CURR_DIR/..
wstool update

echo "================================================================"
echo "Installing other ROS dependencies specified by our packages"
echo "================================================================"

cd $CURR_DIR

# Init Rosdep
sudo rosdep init
# Update Rosdeps
rosdep update

# Install all required dependencies to build this repo
# (unfortunately this is not recursive, so we have to manually specify a few
# directories)
rosdep install --from-paths \
    $CURR_DIR/../src \
    $CURR_DIR/../src/external_pkgs \
    --ignore-src --rosdistro melodic --skip-keys=librealsense2 -y 

echo "================================================================"
echo "Installing other dependencies specified by our packages"
echo "================================================================"
cd $CURR_DIR
sudo ./setup_realsense_manual.sh

echo "================================================================"
echo "Installing Misc. Utilities"
echo "================================================================"

sudo apt-get install -y\
    clang-format\
    python-rosinstall

echo "================================================================"
echo "Finished Installing Utilities"
echo "================================================================"

