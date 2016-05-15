cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR} SmartMatrix/)

FILE(GLOB_RECURSE ros_src
    "${ROS_LIB_DIR}/*.cpp"
    "${ROS_LIB_DIR}/*.h")

add_library(ros_lib ${ros_src})

import_arduino_library(SmartMatrix3)
import_arduino_library(Adafruit_NeoPixel)

add_teensy_executable(head head.cpp)
target_link_libraries(head ros_lib)
