#!/usr/bin/env python

import rospy
import tf.transformations
from math import pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3


def translate_callback(Odo):
	pub = rospy.Publisher('sim/euler', Vector3)
	(yaw, pitch, roll) = tf.transformations.euler_from_quaternion([Odo.pose.pose.orientation.x, Odo.pose.pose.orientation.y, Odo.pose.pose.orientation.z, Odo.pose.pose.orientation.w],'rzyx')
	euler = Vector3()
	euler.x = roll*180/pi
	euler.y = pitch*180/pi
	euler.z = yaw*180/pi
	pub.publish(euler)

if __name__ == '__main__':
	try:
		rospy.init_node('quat2euler')
		rospy.Subscriber('sim/states', Odometry, translate_callback)
		while not rospy.is_shutdown():
			rospy.spin()
			
	except rospy.ROSInterruptException:
		pass
