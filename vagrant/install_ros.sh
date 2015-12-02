#!/bin/bash

ROS_DISTRO_VER=hydro
UBUNTU_DISTRO_VER=precise

#Add apt source
#sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'

#Install release key
#wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

#Updage package listing
#sudo apt-get update

#Install ROS
#sudo apt-get install -y ros-$ROS_DISTRO_VER-desktop-full python-rosinstall 

#sudo apt-get install -y nodejs npm ros-hydro-openni2-launch ros-hydro-rosbridge-suite ros-hydro-depthcloud-encoder

#sudo adduser --system --home /srv/vega vega
#sudo rosdep init

#Switch to user vega
su - vega -s /bin/bash -c /vagrant/vega_user_setup.sh
