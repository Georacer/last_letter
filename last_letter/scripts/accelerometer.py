#!/usr/bin/env python

import roslib
import sys
import rospy
from geometry_msgs.msg import Vector3, Vector3Stamped
import numpy as np
from uav_utils import ThermalRise
from mathutils import saturation, quat2Reb, Vector2Array
from math import floor, exp
from last_letter.msg import SimStates, SimSensor3, Environment

#######################  Accelerometer Class  #####################################################

class Accelerometer():
	def __init__(self,name):
		
		rospy.loginfo('Create ' + name +' Sensor')
		
		self.dt = 1.0/rospy.get_param(name+'/rate', 100.0)
		self.rate =  rospy.Rate(rospy.get_param(name+'/rate', 100.0))
		rospy.loginfo("\ta.rate: %.1f Hz",1.0/self.dt)

		self.cp =  np.array(rospy.get_param(name+'/CP', [0.0, 0.0, 0.0])).reshape(3,1)
		rospy.loginfo("\tb.position: [%.2f,%.2f,%.2f]", self.cp[0],self.cp[1],self.cp[2])

		self.resolution = rospy.get_param(name+'/resolution', 0.0001)
		rospy.loginfo("\tc.resolution: %.1e g",self.resolution)

		self.maxvalue = rospy.get_param(name+'/max', 16.0)
		rospy.loginfo("\td.maxvalue: %.2f g",self.maxvalue)

		self.std_coeff = rospy.get_param(name+'/std_coeff', [0.0,0.0,0.0])
		rospy.loginfo("\te.noise coeff:")
		rospy.loginfo("\t\t1.main std: %.3f",self.std_coeff[0])
		rospy.loginfo("\t\t2.bias coeff: %.3f",self.std_coeff[1])
		rospy.loginfo("\t\t3.random walk coeff: %.3f",self.std_coeff[2])
		
		#Noise Characteristics
		self.bs = np.random.normal(0,rospy.get_param(name+'/scaleBias', 0.03)/3,3)
		self.ks = np.random.normal(0,rospy.get_param(name+'/scaleThermal', 0.002)/3,3)
		self.bb = np.random.normal(0,rospy.get_param(name+'/offsetBias', 0.785)/3,3)
		self.kb = np.random.normal(0,rospy.get_param(name+'/offsetThermal', 0.0084)/3,3)
		self.p0 = rospy.get_param(name+'/RWPSD', 0.00392)	
		self.bias=Vector3()
		self.randomwalk=Vector3()
		self.scale = Vector3()
		
		self.temp=25.0
		self.grav=9.807
		
		#Thermal Chracteristics
		self.thermalMass = ThermalRise(name)
		rospy.loginfo("\tf.Loading Thermal Chracteristics")
		self.airspeed = 0.0
		
		self.measurement = SimSensor3()

		self.pub = rospy.Publisher(name, SimSensor3)
		self.states_sub = rospy.Subscriber("states",SimStates,self.StatesCallback)
		self.env_sub = rospy.Subscriber("environment",Environment,self.EnvironmentCallback)

		self.real=SimStates()
		self.realnew=False

	def iterate(self,states):
		Reb = quat2Reb(states.pose.orientation)
		
		rw_noise = np.random.normal(0,self.p0,3)
		self.randomwalk.x +=self.dt*rw_noise[0]
		self.randomwalk.y +=self.dt*rw_noise[1]
		self.randomwalk.z +=self.dt*rw_noise[2]
		
		self.bias.x = self.bb[0] + self.kb[0] * (self.temp - 298.15)
		self.bias.y = self.bb[1] + self.kb[1] * (self.temp - 298.15)
		self.bias.z = self.bb[1] + self.kb[1] * (self.temp - 298.15)
		
		self.scale.x = self.bs[0] + self.ks[0] * (self.temp - 298.15)
		self.scale.y = self.bs[1] + self.ks[1] * (self.temp - 298.15)
		self.scale.z = self.bs[2] + self.ks[2] * (self.temp - 298.15)

		avel = Vector2Array(states.velocity.angular)
		aveldot = Vector2Array(states.acceleration.angular)
		missal = np.cross(avel,np.cross(avel,self.cp.T)) + np.cross(aveldot,self.cp.T)
		
		self.measurement.axis.x = (1+self.scale.x)*(states.acceleration.linear.x/self.grav + missal[0][0] + self.grav*Reb[2][0]) + self.bias.x + self.randomwalk.x
		self.measurement.axis.y = (1+self.scale.y)*(states.acceleration.linear.y/self.grav + missal[0][1] + self.grav*Reb[2][1]) + self.bias.y + self.randomwalk.y
		self.measurement.axis.z = (1+self.scale.z)*(states.acceleration.linear.z/self.grav + missal[0][2] + self.grav*Reb[2][2]) + self.bias.z + self.randomwalk.z

		self.measurement.axis.x = saturation(floor(self.measurement.axis.x/self.resolution)*self.resolution,-self.maxvalue*self.grav, self.maxvalue*self.grav)
		self.measurement.axis.y = saturation(floor(self.measurement.axis.y/self.resolution)*self.resolution,-self.maxvalue*self.grav, self.maxvalue*self.grav)
		self.measurement.axis.z = saturation(floor(self.measurement.axis.z/self.resolution)*self.resolution,-self.maxvalue*self.grav, self.maxvalue*self.grav)

		self.tempOffset = self.thermalMass.step(self.airspeed,self.dt)
		self.measurement.temperature = self.temp + self.tempOffset


	def StatesCallback(self,data):
		self.real=data
		self.realnew=True

	def EnvironmentCallback(self,data):
		self.grav=data.gravity
		self.temp=data.temperature
		if self.realnew:
			self.airspeed = np.sqrt(pow(self.real.velocity.linear.x - data.wind.x,2) + pow(self.real.velocity.linear.y - data.wind.y,2) + pow(self.real.velocity.linear.z - data.wind.z,2))
			

###################################################################################################	
#######################  Main Program  ############################################################
###################################################################################################	
if __name__ == '__main__':
	rospy.init_node('acc_model')

	fullname = rospy.get_name().split('/')
	accel = Accelerometer(fullname[-1])

	while not rospy.is_shutdown():
		if accel.realnew:
			accel.realnew=False
			accel.iterate(accel.real)
			accel.measurement.header.stamp=rospy.Time.now()
		accel.pub.publish(accel.measurement)
		accel.rate.sleep()
