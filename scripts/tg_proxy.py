#!/usr/bin/env python

import rospy

from json import loads

from pytg.receiver import Receiver  # get messages
from pytg.sender import Sender  # send messages, and other querys.
from pytg.utils import coroutine

#from rosapi.srv import Topics
from rosbridge_library.protocol import Protocol
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.advertise import Advertise

def main():
	rospy.init_node("tg_proxy")
	#rospy.wait_for_service("/rosapi/topics")
	#get_topics = rospy.ServiceProxy("/rosapi/topics", Topics)
	#resp = get_topics()

	#topics = resp.topics
	#msg_type = dict(zip(topics, resp.types))

	#topics = ["/count", "/heartbeat"]
	#types = ['std_msgs/Int64', 'std_msgs/Bool']
	#msg_type = dict(zip(topics, types))
	proto = Protocol("tg_proxy")
	adv = Advertise(proto)
	#for topic in topics:
	#	adv.advertise({'topic':topic, 'type':msg_type[topic]})

	publisher = Publish(proto)

	receiver = Receiver(host="localhost", port=4458)
	sender = Sender(host="localhost", port=4458)

	receiver.start()
	#sender.send_msg("ROS_bot", u"/subscribe")
	sender.send_msg("ROS_bot", u"/topics")
	receiver.message(main_loop(sender, publisher, adv)) 

	sender.send_msg("ROS_bot", u"/unsubscribe")
	receiver.stop()
	print("I am done!")

@coroutine
def main_loop(sender, publisher, adv):
	#quit = False
	try:
		#while not quit:
		while not rospy.is_shutdown():
			msg = (yield) # it waits until it got a message, stored now in msg.
			sender.status_online()
			if msg.event != "message":
				continue  # is not a message.
			if msg.own:  # the bot has send this message.
				continue  # we don't want to process this message.
			if msg.text is None:  # we have media instead.
				continue  # and again, because we want to process only text message.
			# additional processing
			#print(msg.text)
			content = loads(msg.text)
			if "topic" in content:
				publisher.publish(content)
			else:
				for topic, msg_type in content.iteritems():
					adv.advertise({'topic':topic, 'type':msg_type})
				sender.send_msg("ROS_bot", u"/subscribe")
			#publisher.publish(loads(msg.text))

	except GeneratorExit:
		# the generator (pytg) exited (got a KeyboardIterrupt).
		pass
	except KeyboardInterrupt:
		# we got a KeyboardIterrupt(Ctrl+C)
		pass
	except rospy.ROSInterruptException:
		pass
	else:
		# the loop exited without exception, becaues _quit was set True
		pass

if __name__ == '__main__':
    main()
