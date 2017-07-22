#!/usr/bin/python

import rospy

rospy.init_node("hello_ros")

rospy.loginfo("Hello ROS!")
rospy.spin()
