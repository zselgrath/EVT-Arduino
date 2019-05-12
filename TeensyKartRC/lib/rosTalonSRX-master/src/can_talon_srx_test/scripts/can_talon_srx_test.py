#!/usr/bin/env python
import rospy
from can_talon_srx_msgs.msg import Set
from sensor_msgs.msg import Joy

def joyCallback(joy):
	print(joy.axes[1])
	msg = Set()
	msg.set = joy.axes[1]
	pub.publish(msg)


def can_talon_srx_test():
	rospy.init_node('can_talon_srx_test')
	rospy.Subscriber("joy", Joy, joyCallback, queue_size=10)
	global pub
	pub = rospy.Publisher('set', Set, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
	try:
		can_talon_srx_test()
	except rospy.ROSInterruptException:
		pass
