#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose

wallHit = False

def callback(data):
    global wallHit
    if (data.x==0.0 or data.x>11.0 or data.y==0.0 or data.y>11.0):
        if not wallHit:
	    wallHit = True
    	    rospy.loginfo("Wall hit! (" + str(data.x) + ", " + str(data.y)+")")
    else:
        wallHit=False

def listener():
    rospy.init_node("turtle_wall_hit_detector", anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
