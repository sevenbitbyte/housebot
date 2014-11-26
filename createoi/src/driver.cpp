#include "createoi.h"

#include "driver.h"

#include <algorithm>

Driver::Driver(ros::NodeHandlePtr nodeHandlePtr) : _nhPtr(nodeHandlePtr) {
    std::string serialport;

    _nhPtr->param<std::string>("port", serialport, "/dev/ttyUSB0");

    if (startOI(serialport.c_str()) == 0) {
        _twistSubscriber = _nhPtr->subscribe("cmd_vel", 1, &Driver::driveCallback, this);
        _directTwistSubscriber = _nhPtr->subscribe("cmd_wheel_vel", 1, &Driver::directDriveCallback, this);
    } else {
        ROS_FATAL("Failed to open serial port [%s]", serialport.c_str());
    }
}

Driver::~Driver() {
    stopOI();
}

void Driver::driveCallback(const geometry_msgs::TwistPtr & twistPtr) {
    ROS_DEBUG("driveCallback called with [%f] [%f]", twistPtr->linear.x, twistPtr->angular.z);

    short linear = std::max(std::min((int)round(twistPtr->linear.x * 1000.0f), 500), -500);
    short angular = std::max(std::min((int)round(twistPtr->angular.z * 1000.0f), 2000), -2000);
    drive(linear, angular);
}

void Driver::directDriveCallback(const geometry_msgs::TwistPtr & twistPtr) {
    ROS_DEBUG("directDriveCallback called with [%f] [%f]", twistPtr->linear.x, twistPtr->linear.y);
    directDrive(twistPtr->linear.x, twistPtr->linear.y);
}
