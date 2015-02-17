cmake_minimum_required(VERSION 2.8.3)
project(housebot_display)

find_package(catkin REQUIRED COMPONENTS
  housebot_msgs
  rosserial_arduino
  rosserial_client
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS housebot_msgs std_msgs
)

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
add_dependencies(housebot_display_ros_lib housebot_msgs_generate_messages_cpp)
