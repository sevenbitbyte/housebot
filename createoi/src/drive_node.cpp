#include <ros/ros.h>
#include <ros/console.h>

#include "driver.h"

ros::NodeHandlePtr _nhPtr;

int main(int argc, char** argv) {
    ros::init(argc, argv, "drive_node");
    _nhPtr = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    Driver driveNode(_nhPtr);

    ros::spin();

    return 0;
}
