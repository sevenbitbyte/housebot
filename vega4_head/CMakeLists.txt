cmake_minimum_required(VERSION 2.8.3)
project(vega4_head)

find_package(catkin REQUIRED COMPONENTS
  vega4_msgs
  ros_teensy
  rosserial_arduino
  rosserial_client
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS vega4_msgs std_msgs sensor_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY teensy
  TOOLCHAIN_FILE ${ROS_TEENSY_TOOLCHAIN}
)

rosserial_add_client_target(teensy head_Firmware ALL)
