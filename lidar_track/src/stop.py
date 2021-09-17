#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def sim():
	rospy.init_node('stop')
	cmv_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		cmv = Twist()		# which waypoint follower to use
		cmv_pub.publish(cmv)
		rate.sleep()

if __name__ == '__main__':
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
