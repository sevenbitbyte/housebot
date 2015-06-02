#!/bin/bash

PATH=$(dirname $(readlink -f $0))
PKGDIR=${PATH%/*}

echo $PKGDIR
roscd
cd ..

. ~/.bashrc
catkin_make housebot_display_firmware
catkin_make housebot_display_firmware_display-upload
