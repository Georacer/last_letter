#!/usr/bin/env python

import rospy
from math import pow
from math import exp
from math import sin
from math import cos
from math import copysign
from last_letter.srv import *


def calc_lift(req):
	alpha = req.alpha
	#Import required parameters
	M = rospy.get_param('airframe/mcoeff')
	alpha0 = rospy.get_param('airframe/alpha_stall')
	c_lift_0 = rospy.get_param('airframe/c_lift_0')
	c_lift_a = rospy.get_param('airframe/c_lift_a')

	
	sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)))
	linear = (1-sigmoid)*(c_lift_0 + c_lift_a*alpha) #Lift at small AoA
	flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)) #Lift beyond stall
	response = linear+flatPlate
	return response

if __name__=='__main__':
	rospy.init_node('c_lift_a_server')
	s = rospy.Service('c_lift_a', c_lift_a, calc_lift)
	rospy.loginfo('c_lift_a service running')
	rospy.spin()
