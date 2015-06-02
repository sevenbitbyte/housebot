#!/bin/bash

FILEPATH=$(dirname $(readlink -f $0))
PKGDIR=${FILEPATH%/*}

#. /home/vega/.bashrc


echo $PKGDIR
roscd
cd ..

catkin_make housebot_display_firmware
catkin_make housebot_display_firmware_display-upload
