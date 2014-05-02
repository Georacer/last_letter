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

		iT=1.0/120.0
		self.ad = np.exp(iT*self.dt)
		self.bd = -iT*np.exp(-iT*self.dt)+iT
		self.bias=Vector3()
		self.randomwalk=Vector3()
		self.noise_main=Vector3()
		self.Vib=Vector3()
		
		self.measurement = SimSensor3()
		self.temp=25.0

		self.pub = rospy.Publisher(name, SimSensor3)
		self.states_sub = rospy.Subscriber("states",SimStates,self.StatesCallback)
		self.atm_sub = rospy.Subscriber("environment",Environment,self.EnvironmentCallback)

		self.real=SimStates()
		self.realnew=False
		
		#Thermal Chracteristics
		self.thermalMass = ThermalRise(name)
		self.airspeed = 0.0

	def iterate(self,states):

		Reb = quat2Reb(states.pose.orientation)
		Mb,dec=getMag(states.geoid.latitude,states.geoid.longitude,states.geoid.altitude,Reb)

		nm_noise = np.random.normal(0,self.std_coeff,3)

		self.measurement.axis.x=Mb.x+nm_noise[0]
		self.measurement.axis.y=Mb.y+nm_noise[1]
		self.measurement.axis.z=Mb.z+nm_noise[2]

		self.measurement.axis.x = floor(self.measurement.axis.x/self.precision)*self.precision
		self.measurement.axis.y = floor(self.measurement.axis.y/self.precision)*self.precision
		self.measurement.axis.z = floor(self.measurement.axis.z/self.precision)*self.precision

		self.measurement.temperature = self.temp
		
		self.tempOffset = self.thermalMass.step(self.airspeed)
		self.measurement.temperature = self.temp + self.tempOffset


		#euler=quat2euler(states.pose.orientation)
		#phi = euler.x		
		#theta = euler.y
		#psi = euler.z

		#Hx = self.measurement.axis.x*np.cos(theta)+self.measurement.axis.y*np.sin(phi)*np.sin(theta)+self.measurement.axis.z*np.cos(phi)*np.sin(theta)
		#Hy = self.measurement.axis.y*np.cos(phi)-self.measurement.axis.z*np.sin(phi)

		#print -psi*180.0/pi+atan2(-Hy,Hx)*180.0/pi+dec


	def StatesCallback(self,data):
		self.real=data
		self.realnew=True

	def EnvironmentCallback(self,data):
		self.temp=data.temperature
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
