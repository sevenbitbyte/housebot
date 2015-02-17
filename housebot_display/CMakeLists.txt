cmake_minimum_required(VERSION 2.8.3)
project(housebot_display)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  housebot_msgs
  rosserial_arduino
  rosserial_client
  std_msgs
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware display ALL)
rosserial_add_client_target(firmware display-upload)
