#!/usr/bin/env python

import roslib
import sys
import rospy
import geomag

from geometry_msgs.msg import Vector3, Vector3Stamped
import numpy as np
from utils import saturation, quat2Reb, Vector2Array, quat2euler, getMag, ThermalRise
from math import floor, exp, atan2, pi
from last_letter.msg import SimStates, SimSensor3, Environment

_grav=9.81

#######################  Magnetometer Class  #####################################################

class Magnetometer():
	def __init__(self,name):
		
		rospy.loginfo('Create ' + name +' Sensor')
		
		self.dt = 1.0/rospy.get_param(name+'/rate', 100.0)
		self.rate =  rospy.Rate(rospy.get_param(name+'/rate', 100.0))
		rospy.loginfo("\ta.rate: %.1f Hz",1.0/self.dt)

		self.cp =  np.array(rospy.get_param(name+'/CP', [0.0, 0.0, 0.0])).reshape(3,1)
		rospy.loginfo("\tb.position: [%.2f,%.2f,%.2f]", self.cp[0],self.cp[1],self.cp[2])

		self.precision = rospy.get_param(name+'/precision', 0.0001)
		rospy.loginfo("\tc.precision: %.1e g",self.precision)

		self.std_coeff = rospy.get_param(name+'/std_coeff', 0.1)
		rospy.loginfo("\td.noise coeff: %.3f",self.std_coeff)

		#Noise Characteristics
		self.bs = np.random.normal(0,rospy.get_param(name+'/scaleBias', 0.03)/3,3)
		self.ks = np.random.normal(0,rospy.get_param(name+'/scaleThermal', 0.002)/3,3)
		self.bias=Vector3()
		self.randomwalk=Vector3()
		self.scale = Vector3()
		
		self.measurement = SimSensor3()
		self.temp=25.0

		self.pub = rospy.Publisher(name, SimSensor3)
		self.states_sub = rospy.Subscriber("states",SimStates,self.StatesCallback)
		self.atm_sub = rospy.Subscriber("environment",Environment,self.EnvironmentCallback)

		self.real=SimStates()
		self.realnew=False
		
		#Thermal Chracteristics
		self.thermalMass = ThermalRise(name)
		rospy.loginfo("\te.Loading Thermal Chracteristics")
		self.airspeed = 0.0

	def iterate(self,states):

		Reb = quat2Reb(states.pose.orientation)
		Mb,dec=getMag(states.geoid.latitude,states.geoid.longitude,states.geoid.altitude,Reb)

		nm_noise = np.random.normal(0,self.std_coeff,3)
		
		self.scale.x = self.bs[0] + self.ks[0] * (self.temp - 298.15)
		self.scale.y = self.bs[1] + self.ks[1] * (self.temp - 298.15)
		self.scale.z = self.bs[2] + self.ks[2] * (self.temp - 298.15)

		self.measurement.axis.x=(1+self.scale.x)*Mb.x+nm_noise[0]
		self.measurement.axis.y=(1+self.scale.y)*Mb.y+nm_noise[1]
		self.measurement.axis.z=(1+self.scale.z)*Mb.z+nm_noise[2]

		self.measurement.axis.x = floor(self.measurement.axis.x/self.precision)*self.precision
		self.measurement.axis.y = floor(self.measurement.axis.y/self.precision)*self.precision
		self.measurement.axis.z = floor(self.measurement.axis.z/self.precision)*self.precision

		self.measurement.temperature = self.temp
		
		self.tempOffset = self.thermalMass.step(self.airspeed,self.dt)
		self.measurement.temperature = self.temp + self.tempOffset


	def StatesCallback(self,data):
		self.real=data
		self.realnew=True

	def EnvironmentCallback(self,data):
		self.temp=data.temperature
		if self.realnew:
			self.airspeed = np.sqrt(pow(self.real.velocity.linear.x - data.wind.x,2) + pow(self.real.velocity.linear.y - data.wind.y,2) + pow(self.real.velocity.linear.z - data.wind.z,2))
			

###################################################################################################	
#######################  Main Program  ############################################################
###################################################################################################	
if __name__ == '__main__':
	rospy.init_node('acc_model')

	fullname = rospy.get_name().split('/')
	mag = Magnetometer(fullname[-1])

	while not rospy.is_shutdown():
		if mag.realnew:
			mag.realnew=False
			mag.iterate(mag.real)
			mag.measurement.header.stamp=rospy.Time.now()
		mag.pub.publish(mag.measurement)
		mag.rate.sleep()
