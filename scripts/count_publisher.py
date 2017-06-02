#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64

if __name__ == '__main__':
	rospy.init_node('count_publisher')
	rospy.loginfo('Executing count publisher')
	pub = rospy.Publisher('count', Int64, queue_size=10)
	r = rospy.Rate(1) # 10hz
	c = 0
	while not rospy.is_shutdown():
	   pub.publish(c)
	   c += 1
	   r.sleep()
