#!/usr/bin/env python

import rospy, thread, yaml

from json import loads, dumps

from rosapi.srv import Topics
from rosbridge_library.protocol import Protocol
from rosbridge_library.capabilities.subscribe import Subscribe
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.call_service import CallService

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from io import BytesIO
import numpy as np
import cv2

import telegram.ext as tgext

def error(bot, update, error):
    logger.warning('Update "%s" caused error "%s"' % (update, error))

def list_topics(bot, update):
	s = ''
	for topic in topics:
		s += topic + '\n'
	#update.message.reply_text(s)
	update.message.reply_text(msg_type)

def print_info(bot, update):
	s = 'ROS Telegram Bot\n'
	s += '/help - print information\n'
	s += '/topics - list of available topics\n'
	s += '/subscribe [<topic>...]\n'
	s += '/unsubscribe [<topic>...]\n'
	update.message.reply_text(s)

def subscribe(bot, update, args):
	lock.acquire()
	msg = update.message
	msg_dict[msg.from_user.id] = msg
	if not args:
		for topic in topics:
			subscribers[topic].add(msg.from_user.id)
	else:
		for topic in args:
			try:
				subscribers[topic].add(msg.from_user.id)
			except KeyError:
				msg.reply_text('Topic %s not found' % topic)
	lock.release()

def unsubscribe(bot, update, args):
	lock.acquire()
	msg = update.message
	if not args:
		for topic in topics:
			subscribers[topic].discard(msg.from_user.id)
	else:
		for topic in args:
			try:
				subscribers[topic].discard(msg.from_user.id)
			except KeyError:
				msg.reply_text('Topic %s not found' % topic)
	lock.release()

def call(bot, update, args):
	msg = update.message
	msg_dict[msg.from_user.id] = msg
	call_srv.call_service(loads(dumps({"op": "call_service", "id": msg.from_user.id, "service": args[0], "args": map(loads, args[1:]) })))

def publish(bot, update, args):
	pub.publish(loads(dumps({"topic": args[0], "msg": {"data": args[1]}})))

class TgListener():
	def __init__(self, topic):
		self.topic = topic
		dp.add_handler(tgext.CommandHandler(topic[1:], self.handler))

	def handler(self, bot, update):
		lock.acquire()
		msg = update.message
		tlg_id = msg.from_user.id
		if tlg_id in subscribers[self.topic]:
			subscribers[self.topic].discard(tlg_id)
		else:
			subscribers[self.topic].add(tlg_id)
			msg_dict[tlg_id] = msg
		lock.release()

import base64

def send(outgoing):
	if 'topic' in outgoing:
		lock.acquire()
		topic = outgoing['topic']
		msg = outgoing['msg']
		op = outgoing['op']
		if topic=='/image':
			data = base64.b64decode(msg['data'])
			blank_image = np.fromstring(data, dtype=np.uint8)
			blank_image = np.reshape(blank_image, (msg['height'],msg['width'],3))
			retval, enc_image = cv2.imencode('.png', blank_image)
			bio = BytesIO()
			bio.name = 'image.png'
			bio.write(enc_image)
			for tlg_id in subscribers[topic]:
				bio.seek(0)
				msg_dict[tlg_id].reply_photo( bio )
		else:
			for tlg_id in subscribers[topic]:
				msg_dict[tlg_id].reply_text( {'topic':topic, 'msg':msg} )
		lock.release()
	elif 'service' in outgoing:
		values = outgoing['values']
		tlg_id = outgoing['id']
		msg_dict[tlg_id].reply_text( values )

if __name__ == '__main__':

	lock = thread.allocate_lock()
	with open('tg_bridge.yaml', 'r') as yamlfile:
	    cfg = yaml.load(yamlfile)

	rospy.init_node("tg_bridge")
	rospy.wait_for_service("/rosapi/topics")
	get_topics = rospy.ServiceProxy("/rosapi/topics", Topics)
	resp = get_topics()
	topics = resp.topics
	msg_type = dict(zip(topics, resp.types))

	proto = Protocol("tg_bridge")
	proto.send = send

	sub = Subscribe(proto)
	subscribers = {}
	msg_dict = {}
	for topic in topics:
		subscribers[topic] = set()
		sub.subscribe(loads(dumps({"op": "subscribe", "topic": topic, "type": msg_type[topic]})))

	pub = Publish(proto)
	call_srv = CallService(proto)

	updater = tgext.Updater(cfg['token'])
	dp = updater.dispatcher

	dp.add_handler(tgext.CommandHandler('start', print_info))
	dp.add_handler(tgext.CommandHandler('help', print_info))
	dp.add_handler(tgext.CommandHandler('topics', list_topics))
	dp.add_handler(tgext.CommandHandler('subscribe', subscribe, pass_args=True))
	dp.add_handler(tgext.CommandHandler('unsubscribe', unsubscribe, pass_args=True))
	dp.add_handler(tgext.CommandHandler('publish', publish, pass_args=True))
	dp.add_handler(tgext.CommandHandler('call', call, pass_args=True))

	tglist = []
	for topic in topics:
		tglist.append( TgListener(topic) )

	dp.add_error_handler(error)

	updater.start_polling()
	updater.idle()
