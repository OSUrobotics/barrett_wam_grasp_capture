import rospy
import rosbag

from sensor_msgs.msg import JointState

record_playback = None
stall_lock = None
stop_action = False
data_target = ""

def init_data_mgr():
	global stall_lock
	data_mgr_thread = threading.Thread(group=None, target=grasp_data_mgmt.data_mgr)
	stall_lock = Lock()
	stall_lock.acquire()

	data_mgr_thread.start()
	return data_mgr_thread

def begin_capture(file_name):
	global record_playback, stall_lock, stop_action, data_target
	stop_action = False
	record_playback = "r"
	data_target = file_name

	stall_lock.release()
	
def end_capture():
	global stop_action
	stop_action = True

def begin_playback(file_name):
	global record_playback, stall_lock, stop_action, data_target
	data_target = file_name
	record_playback = "p"
	stop_action = False

	stall_lock.release()

def end_playback():
	global stop_action
	stop_action = True

def record_bag():
	global data_target, stop_action
	
	record_freq = rospy.Duration(0.1)
	while not rospy.is_shutdown() and not stop_action:
		print "Recording to bag", data_target
		record_freq.sleep()

def playback_bag():
	global data_target, stop_action

	# Initialize ros service
	
	# Playback
	dummy = rospy.Duration(.5)
	while not rospy.is_shutdown() and not stop_action:
		print "Sending joints"
		dummy.sleep()


def data_mgr_main():
	global stall_lock, record_playback

	while not rospy.is_shutdown():
		stall_lock.acquire()
		if record_playback == "r":
			record_bag()
		elif record_playback == "p":
			play_bag()

		stall_lock.release()

	return
