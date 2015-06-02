#!/bin/bash

FILEPATH=$(dirname $(readlink -f $0))
PKGDIR=${FILEPATH%/*}

source /opt/ros/indigo/setup.bash
source ~/Tools/ros/indigo/devel/setup.bash

echo $PKGDIR
roscd
cd ..

sudo service housebot stop

catkin_make housebot_display_firmware
catkin_make housebot_display_firmware_display-upload

sudo service housebot start

