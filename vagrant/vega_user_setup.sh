#!/bin/bash

export ROS_DISTRO_VER=hydro
source /opt/ros/$ROS_DISTRO_VER/setup.bash

# Create ROS workspace
mkdir -p /srv/vega/Tools/ros/$ROS_DISTRO_VER/src
cd /srv/vega/Tools/ros/$ROS_DISTRO_VER/src
catkin_init_workspace
cd /srv/vega/Tools/ros/$ROS_DISTRO_VER/
catkin_make

# Add ROS additions to bashrc
#cat /vagrant/ros_bashrc.sh >> ~/.bashrc
cp /vagrant/ros_bashrc.sh ~/.bashrc
chmod 750 ~/.bashrc

source ~/.bashrc
rosdep update

roscd
cd ../src
git clone https://github.com/felixendres/rgbdslam_v2.git

rosdep update
rosdep install rgbdslam
roscd; cd ..
catkin_make
catkin_make
