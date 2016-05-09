cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

FILE(GLOB_RECURSE ros_src
    "${ROS_LIB_DIR}/*.cpp"
    "${ROS_LIB_DIR}/*.h")
add_library(ros_lib ${ros_src})

add_teensy_executable(head head.cpp)
target_link_libraries(head ros_lib)
