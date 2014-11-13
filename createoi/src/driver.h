#ifndef DRIVER_H
#define DRIVER_H

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>

class Driver {
private:
    ros::NodeHandlePtr _nhPtr;
    ros::Subscriber _twistSubscriber;
    ros::Subscriber _directTwistSubscriber;

public:
    Driver(ros::NodeHandlePtr);
    ~Driver();

    void driveCallback(const geometry_msgs::TwistPtr&);
    void directDriveCallback(const geometry_msgs::TwistPtr&);
};

#endif
