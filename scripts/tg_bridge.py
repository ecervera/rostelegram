#!/usr/bin/env python

import rospy, thread, yaml

from json import loads, dumps

from rosapi.srv import Topics
from rosbridge_library.protocol import Protocol
from rosbridge_library.capabilities.subscribe import Subscribe

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

def send(outgoing):
	lock.acquire()
	topic = outgoing['topic']
	msg = outgoing['msg']
	op = outgoing['op']
	for tlg_id in subscribers[topic]:
		#msg_dict[tlg_id].reply_text( str(msg) )
		msg_dict[tlg_id].reply_text( {'topic':topic, 'msg':msg} )
	lock.release()
	#print(outgoing)

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

	#rospy.spin()
	updater = tgext.Updater(cfg['token'])
	dp = updater.dispatcher

	dp.add_handler(tgext.CommandHandler('start', print_info))
	dp.add_handler(tgext.CommandHandler('help', print_info))
	dp.add_handler(tgext.CommandHandler('topics', list_topics))
	dp.add_handler(tgext.CommandHandler('subscribe', subscribe, pass_args=True))
	dp.add_handler(tgext.CommandHandler('unsubscribe', unsubscribe, pass_args=True))

	tglist = []
	for topic in topics:
		tglist.append( TgListener(topic) )

	dp.add_error_handler(error)

	updater.start_polling()
	updater.idle()
