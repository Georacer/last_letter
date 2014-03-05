#!/usr/bin/env python

import rospy
import numpy as NP
import tf.transformations
import rosbag
from math import cos, sin, tan
from uav_model.msg import inputs as inputType
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Vector3, Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from uav_model.srv import *


class model:
	def __init__(self):
		self.kinematics = Odometry()
		self.kinematics.header.frame_id = 'bodyFrame' #body axes position frame name
		self.tprev = rospy.Time.now()
		self.kinematics.header.stamp = self.tprev
		self.kinematics.pose.pose.position = Point(0.0, 0.0, -300.0) #initial position
		self.kinematics.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 0.0) #initial orientation 
		self.kinematics.twist.twist.linear = Vector3(20.0, 0.0, 0.0) #initial velocity
		self.kinematics.twist.twist.angular = Vector3(0.0, 0.0, 0.0) #initial angular rotation
		self.input = [0, 0, 0, 0]

		self.dynamics = WrenchStamped()
		self.dynamics.header.frame_id = 'bodyFrame' #body axes force frame name
		
		self.state_pub = rospy.Publisher('sim/states', Odometry)
		self.wrench_pub = rospy.Publisher('sim/wrenchStamped', WrenchStamped)
		rospy.Subscriber("sim/input", inputType, self.getInput,1)


	#simulation step, calls the differential equation solver, is called at simulation rate, supports variable simulation step time
	def step(self):
		#rospy.loginfo('entered step')#
		self.dt = rospy.Time.now() - self.tprev
		self.dt = self.dt.to_sec()
		self.tprev = rospy.Time.now()		
		self.kinematics.header.stamp = self.tprev
		
		#Calculate body forces/torques
		self.dynamics.header.stamp = self.tprev
		#rospy.loginfo('calling getForce')#
		self.dynamics.wrench.force = getForce(self.kinematics, self.input)
		#rospy.loginfo('calling getTorque')#
		self.dynamics.wrench.torque = getTorque(self.kinematics, self.input)
		
		#Apply force/torques on the model
		#rospy.loginfo('calling diffEq')#
		self.diffEq()
	
	#Record messages to a bag
	def record(self):
		bag.write('record.bag',self.kinematics)
		bag.write('recrod.bag',self.dynamics)
		#rospy.loginfo('wrote to bag') #

	#access and store control inputs in the model object
	def getInput(self,inputs):
		self.input = inputs

	#Differential equation step solver
	def diffEq(self):
		q1=self.kinematics.pose.pose.orientation.x
		q2=self.kinematics.pose.pose.orientation.y
		q3=self.kinematics.pose.pose.orientation.z
		q4=self.kinematics.pose.pose.orientation.w

		#Create angles, linear speeds, angular speeds
		(phi, theta, psi) = tf.transformations.euler_from_quaternion([q1, q2, q3, q4])
		(u, v, w) = (self.kinematics.twist.twist.linear.x, self.kinematics.twist.twist.linear.y, self.kinematics.twist.twist.linear.z)
		(p, q, r) = (self.kinematics.twist.twist.angular.x, self.kinematics.twist.twist.angular.y, self.kinematics.twist.twist.angular.z)
		
		#Create position derivatives
		R = tf.transformations.euler_matrix(phi, theta, psi, 'rzyx')
		(pndot, pedot, pddot, discard) = R*NP.matrix([[u], [v], [w], [0]])

		#Create speed derivatives
		m = rospy.get_param('airframe/m')
		(fx, fy, fz) = (self.dynamics.wrench.force.x, self.dynamics.wrench.force.y, self.dynamics.wrench.force.z)
		#rospy.loginfo(m)#
		#rospy.loginfo('point 1')#
		#rospy.loginfo(u)#
		#rospy.loginfo(v)#
		#rospy.loginfo(w)#
		#rospy.loginfo(p)#
		#rospy.loginfo(q)#
		#rospy.loginfo(r)#
		#rospy.loginfo(r*v-q*w)#
		#rospy.loginfo(q*w-r*u)#
		#rospy.loginfo(q*u-p*v)#

		mat2 = 1.0/m*NP.matrix([[fx], [fy], [fz]]) #Linear Acceleration
		mat1 = NP.matrix([[r*v-q*w],[p*w-r*u],[q*u-p*v]]) #Corriolis Acceleration
		
		#rospy.loginfo('point 2')#
		#rospy.loginfo(mat1)#
		#rospy.loginfo('point 3')#
		#rospy.loginfo(mat2)#
		(udot, vdot, wdot) = mat1 + mat2
		udot = udot.item(0)
		vdot = vdot.item(0)
		wdot = wdot.item(0)

		#Create angular derivatives
		(phidot, thetadot, psidot) = NP.matrix([[1, sin(phi)*tan(theta), cos(phi)*tan(theta)],
		[0, cos(phi), -sin(phi)],
		[0, sin(phi)/cos(theta), cos(phi)/cos(theta)]])*NP.matrix([[p], [q], [r]])

		#Create angular rate derivatives
		j_x = rospy.get_param('airframe/j_x')
		j_y = rospy.get_param('airframe/j_y')
		j_z = rospy.get_param('airframe/j_z')
		j_xz = rospy.get_param('airframe/j_xz')
		G = j_x*j_z-pow(j_xz,2)
		G1 = j_xz*(j_x-j_y+j_z)/G
		G2 = (j_z*(j_z-j_y)+pow(j_xz,2))/G
		G3 = j_z/G
		G4 = j_xz/G
		G5 = (j_z-j_x)/j_y
		G6 = j_xz/j_y

		G7 = ((j_x-j_y)*j_x+pow(j_xz,2))/G
		G8 = j_x/G
		(l, m, n) = (self.dynamics.wrench.torque.x, self.dynamics.wrench.torque.y, self.dynamics.wrench.torque.z)
		(pdot, qdot, rdot) = NP.matrix([[G1*p*q-G2*q*r],[G5*p*r-G6*(pow(p,2)-pow(r,2))],[G7*p*q-G1*q*r]]) + NP.matrix([[G3*l+G4*n],[1/j_y*m],[G4*l+G8*n]])
		pdot = pdot.item(0)
		qdot = qdot.item(0)
		rdot = rdot.item(0)

		#Integrate quantities using forward Euler
		self.kinematics.pose.pose.position.x = self.kinematics.pose.pose.position.x + pndot*self.dt
		self.kinematics.pose.pose.position.y = self.kinematics.pose.pose.position.y + pedot*self.dt
		self.kinematics.pose.pose.position.z = self.kinematics.pose.pose.position.z + pddot*self.dt

		self.kinematics.twist.twist.linear.x = self.kinematics.twist.twist.linear.x + udot*self.dt
		self.kinematics.twist.twist.linear.y = self.kinematics.twist.twist.linear.y + vdot*self.dt
		self.kinematics.twist.twist.linear.z = self.kinematics.twist.twist.linear.z + wdot*self.dt
		
		phi = phi + phidot*self.dt
		theta = theta + thetadot*self.dt
		psi = psi + psidot*self.dt
		(q1, q2, q3, q4) = tf.transformations.quaternion_from_euler(psi, theta, phi, 'rzyx')
		self.kinematics.pose.pose.orientation.x = q1
		self.kinematics.pose.pose.orientation.y = q2
		self.kinematics.pose.pose.orientation.z = q3
		self.kinematics.pose.pose.orientation.w = q4
		
		self.kinematics.twist.twist.angular.x  = self.kinematics.twist.twist.angular.x + pdot*self.dt
		self.kinematics.twist.twist.angular.y  = self.kinematics.twist.twist.angular.y + qdot*self.dt
		self.kinematics.twist.twist.angular.z  = self.kinematics.twist.twist.angular.z + rdot*self.dt

		#Publish states and wrench
		self.state_pub.publish(self.kinematics)
		self.wrench_pub.publish(self.dynamics)

def getForce(states, inputs):
	#rospy.loginfo('Entered getForce')#
	rospy.wait_for_service('calc_force')
	try:
		#rospy.loginfo('Creating serivce proxy')#
		calcForce = rospy.ServiceProxy('calc_force', calc_force)
		#rospy.loginfo('Calling the proxy')#
		resp1 = calcForce(states,inputs)
		return Vector3(resp1.force.x, resp1.force.y, resp1.force.z)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def getTorque(states, inputs):
	rospy.wait_for_service('calc_torque')
	try:
		calcTorque = rospy.ServiceProxy('calc_torque', calc_torque)
		resp1 = calcTorque(states, inputs)
		return Vector3(resp1.torque.x, resp1.torque.y, resp1.torque.z)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e



if __name__ == '__main__':
	simRate = rospy.get_param('simRate')
	#bag = rosbag.Bag('record.bag','w')
	try:
		rospy.init_node('simNode')
		#rospy.loginfo('node up')
		spinner = rospy.Rate(simRate)
		#rospy.loginfo('spinner up')
		modelObj = model()
		#rospy.loginfo('model up')
		rospy.sleep(0.2); #wait for nodes to get initialized
		while not rospy.is_shutdown():
			#rospy.loginfo('before step')#
			modelObj.step()
			#rospy.loginfo('after step')#
			spinner.sleep()
			#modelObj.record()			
	except rospy.ROSInterruptException:
		#bag.close()
		pass	



	
