#! /usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int32

if __name__ == "__main__":
	print "Node online."
	rospy.init_node("rbagtest")
	
	bag_name = "test_bag.bag"
	print "Creating bag file", bag_name
	b1 = rosbag.Bag(bag_name, "w")
	i = Int32()
	i.data = 42
	b1.write('dummytopic', i)
	b1.close()

	rospy.spin()
