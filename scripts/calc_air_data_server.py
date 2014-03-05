#!/usr/bin/env python

#Service Provider for the air data

import rospy
from math import sqrt
from math import atan2
from math import asin
from math import pow
from uav_model.srv import *


def calc_air_data_callback(req):
	u = req.speeds.x
	v = req.speeds.y
	w = req.speeds.z
	
	airspeed = sqrt(pow(u,2)+pow(v,2)+pow(w,2))
	alpha = atan2(w,u)
	if u==0:
		if v==0:
			beta=0
		else:
			beta=asin(copysign(1,v))
	else:
		beta = atan2(v,u)
	return (airspeed, alpha, beta)

if __name__=='__main__':
	rospy.init_node('calc_air_data_server')
	s = rospy.Service('calc_air_data', calc_air_data, calc_air_data_callback)
	rospy.loginfo('calc_air_data service running')
	rospy.spin()
