cmake_minimum_required(VERSION 2.8.3)
project(vega4_joint_state)

find_package(catkin REQUIRED rospy sensor_msgs dynamixel_msgs)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy sensor_msgs dynamixel_msgs
)

install(PROGRAMS
  scripts/joint_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
