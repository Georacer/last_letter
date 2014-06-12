#!/usr/bin/env python

import roslib
import sys
import rospy
import numpy as np
from utils import saturation, quat2Reb, ThermalRise
from math import floor
from last_letter.msg import SimStates, SimSensor, Environment

#######################  Altimeter Class  #########################################################

class Altimeter():
	def __init__(self,name):

		rospy.loginfo('Create ' + name +' Sensor')
		
		self.dt = 1.0/rospy.get_param(name+'/rate', 100.0)
		self.rate =  rospy.Rate(rospy.get_param(name+'/rate', 100.0))
		rospy.loginfo("\ta.rate: %.1f Hz",1.0/self.dt)

		self.cp =  np.array(rospy.get_param(name+'/CP', [0.0, 0.0, 0.0])).reshape(3,1)
		rospy.loginfo("\tb.position: [%.2f,%.2f,%.2f]", self.cp[0],self.cp[1],self.cp[2])

		self.resolution = rospy.get_param(name+'/resolution', 0.001)
		rospy.loginfo("\tc.resolution: %.1e meters",self.resolution)

		self.maxvalue = rospy.get_param(name+'/max', 1500.0)
		rospy.loginfo("\td.maxvalue: %.4f meters",self.maxvalue)

		self.minvalue = rospy.get_param(name+'/min', 0.0)
		rospy.loginfo("\te.minvalue: %.4f meters",self.minvalue)

		self.std_coeff = rospy.get_param(name+'/std_coeff', 0.1)
		rospy.loginfo("\tf.noise coeff: %.3f",self.std_coeff)

		self.pub = rospy.Publisher(name, SimSensor)
		self.states_sub = rospy.Subscriber('states',SimStates,self.StatesCallback)
		self.env_sub = rospy.Subscriber("environment",Environment,self.EnvironmentCallback)

		self.measurement = SimSensor()
		self.realnew=False
		
		#Thermal Chracteristics
		self.thermalMass = ThermalRise(name)
		rospy.loginfo("\tg.Loading Thermal Chracteristics")
		
		self.airspeed = 0.0
		self.temp=25.0

	def iterate(self,states):
		Reb = quat2Reb(states.pose.orientation)
		
		if abs(Reb[2][2])<=1e-4:
			self.measurement.value=self.maxvalue
		else:
			self.measurement.value=saturation((-states.pose.position.z-np.dot(Reb[2][0:3],self.cp)/Reb[2][2]),self.minvalue,self.maxvalue)
		
		rf_noise = np.random.normal(0,self.std_coeff,1)
		if self.measurement.value<=self.minvalue:
			rf_noise = np.random.normal(0,self.minvalue/3.0,1)
		if self.measurement.value>=self.maxvalue-1:
			rf_noise = np.random.normal(0,self.maxvalue/3.0,1)
			

		self.measurement.value +=rf_noise[0]
		self.measurement.value = max(self.minvalue,floor(self.measurement.value/self.resolution)*self.resolution)
		
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
	rospy.init_node('bar_model')

	fullname = rospy.get_name().split('/')
	alt = Altimeter(fullname[-1])

	while not rospy.is_shutdown():
		if alt.realnew:
			alt.realnew=False
			alt.iterate(alt.real)
			alt.measurement.header.stamp=rospy.Time.now()
		alt.pub.publish(alt.measurement)
		alt.rate.sleep()
