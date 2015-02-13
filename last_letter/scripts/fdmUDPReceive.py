#!/usr/bin/env python
# Receive controlPWM data as a UDP stream, packed as an FG_FDM object
# Based on runsim.py by Ardupilot Tools git

import roslib
import sys
import rospy
from geometry_msgs.msg import Vector3, Vector3Stamped
import numpy as np
import tf.transformations
from mathutils import saturation, quat2Reb, Vector2Array
from last_letter.msg import SimStates, SimPWM, Environment

from pymavlink import fgFDM
import socket, struct, errno

def interpret_address(addrstr):
    '''interpret a IP:port string'''
    a = addrstr.split(':')
    a[1] = int(a[1])
    return tuple(a)

def process_ctrl_input(buf):
	global fdm_ctrls

	fdm_ctrls.parse(buf)
	ctrls = SimPWM()
	ctrls.value[0] = fdm_ctrls.get('left_aileron')
	ctrls.value[1] = fdm_ctrls.get('elevator')
	ctrls.value[2] = fdm_ctrls.get('rpm')
	ctrls.value[3] = fdm_ctrls.get('rudder')
	# print ctrls
	pub.publish(ctrls)


jsb_out_address = "127.0.0.1:5505" # FDM control data coming into the ROS Simulator from Ardupilot
jsb_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
jsb_out.bind(interpret_address(jsb_out_address))
# jsb_out.setblocking(0)

fdm_ctrls = fgFDM.fgFDM()

###############
## Main Program
###############

if __name__ == '__main__':
	try:
		rospy.init_node('fdmUDPReceive')
		# pub = rospy.Publisher('/fw1/ctrlPWM',SimPWM)
		pub = rospy.Publisher('/fw1/rawPWM',SimPWM)

		timer = rospy.Rate(1000)

		while not rospy.is_shutdown():

			buf = jsb_out.recv(fdm_ctrls.packet_size())
			if buf:
				process_ctrl_input(buf)

			timer.sleep()

		print "this node is pretty much dead"

	except rospy.ROSInterruptException:
		print "something bad happened"
		pass