#!/usr/bin/env python

import rospy

from json import loads, dumps

from rosbridge_library.protocol import Protocol
from rosbridge_library.capabilities.subscribe import Subscribe

def send(outgoing):
	print(outgoing)

if __name__ == '__main__':

	rospy.init_node("test_subscribe")

	proto = Protocol("test_subscribe")
	proto.send = send

	sub = Subscribe(proto)

	topic = "/count"
	msg_type = "std_msgs/Int64"

	sub.subscribe(loads(dumps({"op": "subscribe", "topic": topic, "type": msg_type})))

	rospy.spin()
