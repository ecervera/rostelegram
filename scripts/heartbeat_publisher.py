#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

if __name__ == '__main__':
	rospy.init_node('heartbeat_publisher')
	rospy.loginfo('Executing heartbeat publisher')
	pub = rospy.Publisher('heartbeat', Bool, queue_size=10)
	r = rospy.Rate(1) # 10hz
	while not rospy.is_shutdown():
	   pub.publish(True)
	   r.sleep()
