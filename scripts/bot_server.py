#!/usr/bin/env python

import importlib, logging, sys, thread, yaml
import rospy
import telegram.ext as tgext

class Listener():
	def __init__(self, topic_name, topic_class):
		self.subscribers = set()
		self.msg = {}
		self.lock = thread.allocate_lock()
		rospy.Subscriber(topic_name, topic_class, self.callback)

	def callback(self, data):
		self.lock.acquire()
		for tlg_id in self.subscribers:
			self.msg[tlg_id].reply_text( 'data: %s\n---' % data.data)
		self.lock.release()

	def subscribe(self, msg):
		self.lock.acquire()
		tlg_id = msg.from_user.id
		self.subscribers.add(tlg_id)
		self.msg[tlg_id] = msg
		self.lock.release()

	def unsubscribe(self, msg):
		self.lock.acquire()
		tlg_id = msg.from_user.id
		self.subscribers.discard(tlg_id)
		self.msg[tlg_id] = None
		self.lock.release()

def error(bot, update, error):
    logger.warning('Update "%s" caused error "%s"' % (update, error))

def list_topics(bot, update):
	s = ''
	for topic_name in subscribed_topics:
		s += topic_name + '\n'
	update.message.reply_text(s)

def print_info(bot, update):
	s = 'ROS Telegram Bot\n'
	s += '/help - print information\n'
	s += '/topics - list of available topics\n'
	s += '/subscribe [<topic>...]\n'
	s += '/unsubscribe [<topic>...]\n'
	update.message.reply_text(s)

def subscribe(bot, update, args):
	msg = update.message
	if not args:
		for topic in subscribed_topics:
			listeners[topic].subscribe(msg)
	else:
		for topic in args:
			try:
				listeners[topic].subscribe(msg)
			except KeyError:
				msg.reply_text( 'Topic not found')

def unsubscribe(bot, update, args):
	msg = update.message
	if not args:
		for topic in subscribed_topics:
			listeners[topic].unsubscribe(msg)
	else:
		for topic in args:
			try:
				listeners[topic].unsubscribe(msg)
			except KeyError:
				msg.reply_text( 'Topic not found')

class TgListener():
	def __init__(self, topic_name):
		self.topic_name = topic_name
		dp.add_handler(tgext.CommandHandler(topic_name[1:], self.handler))

	def handler(self, bot, update):
		msg = update.message
		tlg_id = msg.from_user.id
		if tlg_id in listeners[self.topic_name].subscribers:
			listeners[self.topic_name].unsubscribe(msg)
		else:
			listeners[self.topic_name].subscribe(msg)

if __name__ == '__main__':

	if len(sys.argv) < 2:
		print('usage: bot_server.py config_file')
		sys.exit(0)
	else:
		config_file = sys.argv[1]

	logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
			    level=logging.INFO)

	logger = logging.getLogger(__name__)

	#with open("config.yaml", 'r') as ymlfile:
	with open(config_file, 'r') as ymlfile:
	    cfg = yaml.load(ymlfile)

	rospy.init_node('bot_server', anonymous=True)

	topic_list = cfg['topics']
	published_topics = dict(rospy.get_published_topics())
	subscribed_topics = []
	classes = {}
	listeners = {}
	for topic_name in topic_list:
		try:
			topic_type = published_topics[topic_name]
			try:
				topic_class = classes[topic_name]
			except KeyError:
				tokens = topic_type.split('/')
				package = tokens[0] + '.msg'
				type_name = tokens[1]
				module = importlib.import_module(package)
				topic_class = getattr(module, type_name)
				classes[topic_name] = topic_class
			listeners[topic_name] = Listener(topic_name, topic_class)
			subscribed_topics.append(topic_name)
			rospy.loginfo('Subscribed to %s' % topic_name)
		except KeyError:
			rospy.loginfo('ROS Topic %s not available' % topic_name)

	updater = tgext.Updater(cfg['token'])
	dp = updater.dispatcher

	dp.add_handler(tgext.CommandHandler('start', print_info))
	dp.add_handler(tgext.CommandHandler('help', print_info))
	dp.add_handler(tgext.CommandHandler('topics', list_topics))
	dp.add_handler(tgext.CommandHandler('subscribe', subscribe, pass_args=True))
	dp.add_handler(tgext.CommandHandler('unsubscribe', unsubscribe, pass_args=True))

	tglist = []
	for topic_name in subscribed_topics:
		tglist.append( TgListener(topic_name) )

	dp.add_error_handler(error)

	updater.start_polling()
	updater.idle()
