#!/usr/bin/env python

"""
    dynamixel_joint_state_publisher.py - Version 1.0 2010-12-28

    Publish the dynamixel_controller joint states on the /joint_states topic

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import roslib;
import rospy

from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import DynamixelJointState

class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class JointStatePublisher():
    def __init__(self):
        rospy.init_node('joint_state_publisher', anonymous=True)

        rate = rospy.get_param('~rate', 50)
        r = rospy.Rate(rate)

        dynamixels = rospy.get_param('dynamixel_joints', '')

        self.joints = list()

        self.servos = list()
        self.controllers = list()
        self.joint_states = dict({})

        self.kinect_calib = - 0.11

        for joint in sorted(dynamixels):
            controller = joint.replace("_joint", "") + "_controller"
            self.joint_states[joint] = JointStateMessage(joint, 0.0, 0.0, 0.0)
            self.controllers.append(controller)

            rospy.loginfo('Subscribing to topic: %s','/arm/' + self.controllers + '/state')

        # Start controller state subscribers
        [rospy.Subscriber('/dynamixel/' + c + '/state', DynamixelJointState, self.controller_state_handler) for c in self.controllers]

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState)

        rospy.loginfo("Starting Dynamixel Joint State Publisher at " + str(rate) + "Hz")


        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()

    def controller_state_handler(self, msg):
        self.joint_states[msg.name] = JointStateMessage(msg.name, msg.current_pos, msg.velocity, msg.load)




    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        for joint in self.joint_states.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position)
            msg.velocity.append(joint.velocity)
            msg.effort.append(joint.effort)

        msg.header.stamp = rospy.Time.now()
        self.joint_states_pub.publish(msg)

if __name__ == '__main__':
    try:
        s = JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass
