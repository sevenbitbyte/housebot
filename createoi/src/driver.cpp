#include "createoi.h"

#include "driver.h"

Driver::Driver(ros::NodeHandlePtr nodeHandlePtr) : _nhPtr(nodeHandlePtr) {
    std::string serialport;

    _nhPtr->param<std::string>("port", serialport, "/dev/ttyUSB0");

    if (startOI(serialport.c_str()) == 0) {
        _twistSubscriber = _nhPtr->subscribe("drivecmd", 1, &Driver::driveCallback, this);
        _directTwistSubscriber = _nhPtr->subscribe("directdrivecmd", 1, &Driver::directDriveCallback, this);
    } else {
        ROS_FATAL("Failed to open serial port [%s]", serialport.c_str());
    }
}

Driver::~Driver() {
    stopOI();
}

void Driver::driveCallback(const geometry_msgs::TwistPtr & twistPtr) {
    ROS_DEBUG("driveCallback called with [%i] [%i]", (int)twistPtr->linear.x, (int)twistPtr->angular.x);
    drive(twistPtr->linear.x, twistPtr->angular.x);
}

void Driver::directDriveCallback(const geometry_msgs::TwistPtr & twistPtr) {
    ROS_DEBUG("directDriveCallback called with [%i] [%i]", (int)twistPtr->linear.x, (int)twistPtr->linear.y);
    directDrive(twistPtr->linear.x, twistPtr->linear.y);
}
