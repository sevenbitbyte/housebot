cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

FILE(GLOB_RECURSE ros_src
    "${ROS_LIB_DIR}/*.cpp"
    "${ROS_LIB_DIR}/*.h")

add_library(ros_lib ${ros_src})

import_arduino_library(Adafruit_NeoPixel)

add_teensy_executable(belt belt.cpp)
target_link_libraries(belt ros_lib)
