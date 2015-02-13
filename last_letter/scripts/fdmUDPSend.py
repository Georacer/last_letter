#!/usr/bin/env python
# Send simState data as a UDP stream, packed as an FG_FDM object
# Based on runsim.py by Ardupilot Tools git

import roslib
import sys
import rospy
from geometry_msgs.msg import Vector3, Vector3Stamped
import numpy as np
import tf.transformations
from mathutils import quat2Reb, Vector2Array
from last_letter.msg import SimStates, SimPWM, Environment

from pymavlink import fgFDM
import socket, struct, errno

def interpret_address(addrstr):
    '''interpret a IP:port string'''
    a = addrstr.split(':')
    a[1] = int(a[1])
    return tuple(a)

def state_callback(state):

	global fdm, stateStorage

	stateStorage = state

	fdm.set('latitude', state.geoid.latitude, units='degrees')
	fdm.set('longitude', state.geoid.longitude, units='degrees')
	fdm.set('altitude', state.geoid.altitude, units='meters')
	# rospy.logerr('Lat/Lon/Alt: %g/%g/%g',state.geoid.latitude, state.geoid.longitude, state.geoid.altitude)

	fdm.set('v_north', state.geoid.velocity.x, units='mps')
	fdm.set('v_east', state.geoid.velocity.y, units='mps')
	fdm.set('v_down', -state.geoid.velocity.z, units='mps') # Downwards velocity
	# rospy.logerr('vn/ve/vd: %g/%g/%g',state.geoid.velocity.x, state.geoid.velocity.y, -state.geoid.velocity.z)

	# Calculate accelerometer felt accelerations
	# Reb = quat2Reb(state.pose.orientation)
	# accx = (state.acceleration.linear.x - 9.80665*Reb[2][0])
	# accy = (state.acceleration.linear.y - 9.80665*Reb[2][1])
	# accz = (state.acceleration.linear.z - 9.80665*Reb[2][2])
	# fdm.set('A_X_pilot', accx, units='mpss')
	# fdm.set('A_Y_pilot', accy, units='mpss')
	# fdm.set('A_Z_pilot', accz, units='mpss')
	# rospy.logerr('acc_x/y/z: %g/%g/%g',accx, accy, accz)

	(yaw, pitch, roll) = tf.transformations.euler_from_quaternion([state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w],'rzyx')
	fdm.set('phi', roll, units='radians')
	fdm.set('theta', pitch, units='radians')
	fdm.set('psi', yaw, units='radians')
	# rospy.logerr('r/p/y: %4.1f/%4.1f/%4.1f',180/np.pi*roll, 180/np.pi*pitch, 180/np.pi*yaw)

	(p, q, r) = (state.velocity.angular.x, state.velocity.angular.y, state.velocity.angular.z)
	phiDot = p + np.tan(pitch)*np.sin(roll)*q + np.tan(pitch)*np.cos(roll)*r
	thetaDot = np.cos(roll)*q - np.sin(roll)*r
	psiDot = np.sin(roll)/np.cos(pitch)*q + np.cos(roll)/np.cos(pitch)*r

	fdm.set('phidot', phiDot, units='rps')
	fdm.set('thetadot', thetaDot, units='rps')
	fdm.set('psidot', psiDot, units='rps')
	# rospy.logerr('dot_phi/theta/psi: %g/%g/%g',phiDot, thetaDot, psiDot)

	vcas = np.sqrt(state.velocity.linear.x*state.velocity.linear.x + state.velocity.linear.y*state.velocity.linear.y + state.velocity.linear.z*state.velocity.linear.z)
	fdm.set('vcas', vcas, units='mps') # Needs to take wind into account!
	# rospy.logerr('vcas: %g',vcas)

	fdm.set('rpm', state.rotorspeed[0]*np.pi*60)
	fdm.set('agl', state.geoid.altitude, units='meters')

def accel_callback(accel):

	global fdm, stateStorage
	Reb = quat2Reb(stateStorage.pose.orientation)
	accx = (accel.x - 9.80665*Reb[2][0])
	accy = (accel.y - 9.80665*Reb[2][1])
	accz = (accel.z - 9.80665*Reb[2][2])
	fdm.set('A_X_pilot', accx, units='mpss')
	fdm.set('A_Y_pilot', accy, units='mpss')
	fdm.set('A_Z_pilot', accz, units='mpss')
	rospy.logerr('acc_x/y/z: %g/%g/%g',accx, accy, accz)


########
## Setup
########

jsb_in_address = "127.0.0.1:5504" # FDM data coming out of the ROS Simulator
jsb_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
jsb_in.connect(interpret_address(jsb_in_address)) # Should I use connect?

fdm = fgFDM.fgFDM() # Create fdm objects

###############
## Main Program
###############

if __name__ == '__main__':
	try:
		rospy.init_node('fdmUDPSend')
		rospy.Subscriber('/fw1/states', SimStates, state_callback, queue_size=1)
		rospy.Subscriber('/fw1/linearAcc', Vector3, accel_callback, queue_size=1)
		timer = rospy.Rate(1000)
		stateStorage = SimStates()

		while not rospy.is_shutdown():
			try:
				jsb_in.send(fdm.pack())
			except socket.error as e:
				if e.errno not in [ errno.ECONNREFUSED ]:
					print "error on jsb_in sending"
					raise
			timer.sleep()

		print "this node is pretty much dead"

	except rospy.ROSInterruptException:
		print "something bad happened"
		pass