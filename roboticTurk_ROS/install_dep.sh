#!/bin/sh

# Install ROS kinetic for Ubuntu

# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Set up your keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && sudo apt update
# Install with apt
sudo apt-get install -y ros-kinetic-desktop-full ros-kinetic-octomap-msgs ros-kinetic-moveit

# Create directory K.O.U.R.A for robotic arm and initialize ROS workspace for FRESH workspace


#mkdir K.O.U.R.A && mkdir K.O.U.R.A/src
#source /opt/ros/kinetic/setup.bash
#catkin_make
#source /devel/setup.bash

# Download required packages for Dynamixel servos

cd src

git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs

# Using modified version of Dynamixel Workbench, not cloning it
#git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench

git clone https://github.com/wg-perception/object_recognition_msgs.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git

#git clone https://github.com/ros-planning/moveit_msgs.git
#git clone https://github.com/ros-planning/moveit.git
# Installation from git requires some dependencies as well, which requries manual compiling and goes big hassle



