#!/bin/bash

export ROS_DISTRO=hydro
source /opt/ros/$ROS_DISTRO/setup.bash



export ROS_WORKSPACE=~/Tools/ros/$ROS_DISTRO
source $ROS_WORKSPACE/devel/setup.sh




export PRIMARY_IP=127.0.0.1
export ROS_MASTER_URI=http://192.168.50.1:11311
export ROS_IP=$PRIMARY_IP

