#! /usr/bin/python
import rospy
import rosbag
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty
from wam_msgs.msg import HandCommand

import grasp_capture

import threading
import time
import socket

class HandPlayback:
	def __init__(self):
		# ROS Interface
		#self.first_pos_pub = rospy.Publisher('/bhand/hand_cmd_blocking', JointState, queue_size=1)
		self.hand_cmd_pub = rospy.Publisher('/bhand/hand_cmd', HandCommand, queue_size=1)
		self.playback_start_sub = rospy.Subscriber('start_hand_playback', String, self.start_hand_playback)
		self.playback_stop_sub = rospy.Subscriber('stop_hand_playback', Empty, self.start_hand_playback)

		self.stop_playback = False

	# Preconditions: The hand has been set in its initial position
	def start_hand_playback(self, msg):
		rospy.loginfo("Hand playback initiated for: " + msg.data)
		hand_bag_path = msg.data
		hand_bag = rosbag.Bag(hand_bag_path, "r")
		self.get_start_time(hand_bag)
		local_start_time = rospy.Time.now()
		rospy.loginfo("local_start_time: " + str(local_start_time) + "self.bag_start_time: " + str(self.bag_start_time))
		for topic, msg, t in hand_bag.read_messages(topics=['/bhand/hand_cmd']):
			if self.stop_playback:
				break
			else:
				# Wait until the message was sent at record time
				msg_delay = t - self.bag_start_time
				rospy.loginfo("Message delay: " + str(msg_delay))
				while True:
					cur_delay = rospy.Time.now() - local_start_time
					if msg_delay < cur_delay:
						break
					time.sleep(0.1)
				
				msg.header.stamp = rospy.Time.now()
				self.hand_cmd_pub.publish(msg)

	def get_start_time(self, hand_bag):
		for topic, msg, t in hand_bag.read_messages(topics=['/bhand/hand_cmd']):
			self.bag_start_time = msg.header.stamp
			break
			

	def stop_hand_playback(self,msg):
		self.stop_playback = True


if __name__ =="__main__":
	rospy.init_node("bhand_playback")
	rospy.loginfo("Hand playback node online.")
	hp = HandPlayback()
	rospy.spin()
