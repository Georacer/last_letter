#!/usr/bin/env python

import rospy
from math import pow
from math import pi
from uav_model.srv import *


def calc_drag(req):
	alpha = req.alpha
	#Import required parameters
	c_drag_p = rospy.get_param('airframe/c_drag_p')
	c_lift_0 = rospy.get_param('airframe/c_lift_0')
	c_lift_a = rospy.get_param('airframe/c_lift_a')
	oswald = rospy.get_param('world/oswald')
	b = rospy.get_param('airframe/b')
	S = rospy.get_param('airframe/s')
	AR = pow(b,2)/S

	response = c_drag_p + pow(c_lift_0+c_lift_a*alpha,2)/(pi*oswald*AR)	
	return response

if __name__=='__main__':
	rospy.init_node('c_drag_a_server')
	s = rospy.Service('c_drag_a', c_drag_a, calc_drag)
	rospy.loginfo('c_drag_a service running')
	rospy.spin()
