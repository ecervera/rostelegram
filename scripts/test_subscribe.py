#!/usr/bin/env python

import rospy

from json import loads, dumps

from rosapi.srv import Topics
from rosbridge_library.protocol import Protocol
from rosbridge_library.capabilities.subscribe import Subscribe

def send(outgoing):
	print(outgoing)

if __name__ == '__main__':

	rospy.init_node("test_subscribe")
	rospy.wait_for_service("/rosapi/topics")
	get_topics = rospy.ServiceProxy("/rosapi/topics", Topics)
	resp = get_topics()
	#print(resp.topics)
	#print(resp.types)
	msg_type = dict(zip(resp.topics, resp.types))

	proto = Protocol("test_subscribe")
	proto.send = send

	sub = Subscribe(proto)

	topic = "/count"
	#msg_type = "std_msgs/Int64"

	sub.subscribe(loads(dumps({"op": "subscribe", "topic": topic, "type": msg_type[topic]})))

	rospy.spin()
