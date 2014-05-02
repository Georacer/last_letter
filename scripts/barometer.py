#!/usr/bin/env python

import roslib
import sys
import rospy
import numpy as np
from utils import saturation, quat2Reb, ThermalRise
from math import floor, exp
from last_letter.msg import SimStates, SimBarometer, Environment

_grav=9.81

#######################  Barometer Class  #########################################################

class Barometer():
	def __init__(self,name):
		
		rospy.loginfo('Create ' + name +' Sensor')
		
		self.dt = 1.0/rospy.get_param(name+'/rate', 100.0)
		self.rate =  rospy.Rate(rospy.get_param(name+'/rate', 100.0))
		rospy.loginfo("\ta.rate: %.1f Hz",1.0/self.dt)

		self.cp =  np.array(rospy.get_param(name+'/CP', [0.0, 0.0, 0.0])).reshape(3,1)
		rospy.loginfo("\tb.position: [%.2f,%.2f,%.2f]", self.cp[0],self.cp[1],self.cp[2])

		self.resolution = rospy.get_param(name+'/resolution', 0.001)
		rospy.loginfo("\tc.resolution: %.1e mbar",self.resolution)

		self.maxvalue = rospy.get_param(name+'/max', 1500.0)
		rospy.loginfo("\td.maxvalue: %.4f mbar",self.maxvalue)

		self.minvalue = rospy.get_param(name+'/min', 0.0)
		rospy.loginfo("\te.minvalue: %.4f mbar",self.minvalue)

		self.std_coeff = rospy.get_param(name+'/std_coeff', 0.1)
		rospy.loginfo("\tf.noise coeff: %.3f",self.std_coeff)

		self.bar_true=1013.25
		self.temp=25.0

		M=0.0289644;
		ro=8.31447;
		pp = np.random.normal(0,50,1)
		self.P0=1000+pp[0]

		pp = np.random.normal(0,10,1)
		T0=298.15+pp[0]


		self.k1=M/(ro*T0);

		self.pub = rospy.Publisher(name, SimBarometer)
		self.states_sub = rospy.Subscriber("states",SimStates,self.StatesCallback)
		self.atm_sub = rospy.Subscriber("environment",Environment,self.EnvironmentCallback)

		self.measurement = SimBarometer()
		self.realnew=False
		
		#Thermal Chracteristics
		self.thermalMass = ThermalRise(name)
		self.airspeed = 0.0
		self.temp=25.0

	def iterate(self,states):
		Reb = quat2Reb(states.pose.orientation)
		bar_noise = np.random.normal(0,self.std_coeff,1)
		barpos = states.pose.position.z+np.dot(Reb[2][0:3],self.cp)
		#self.measurement.pressure=((self.P0)*exp(-self.k1*_grav*(barpos))+bar_noise[0])+0.001*states.rotorspeed[0]/(barpos*barpos);
		self.measurement.pressure = self.bar_true

		self.measurement.pressure = saturation(floor(self.measurement.pressure/self.resolution)*self.resolution,self.minvalue, self.maxvalue)

		self.tempOffset = self.thermalMass.step(self.airspeed)
		self.measurement.temperature = self.temp + self.tempOffset


	def StatesCallback(self,data):
		self.real=data
		self.realnew=True

	def EnvironmentCallback(self,data):
		self.bar_true=data.pressure
		self.grav=data.gravity
		self.temp=data.temperature
		self.airspeed = np.sqrt(pow(self.real.velocity.linear.x - data.wind.x,2) + pow(self.real.velocity.linear.y - data.wind.y,2) + pow(self.real.velocity.linear.z - data.wind.z,2))
			

###################################################################################################	
#######################  Main Program  ############################################################
###################################################################################################	
if __name__ == '__main__':
	rospy.init_node('bar_model')

	fullname = rospy.get_name().split('/')
	bar = Barometer(fullname[-1])

	while not rospy.is_shutdown():
		if bar.realnew:
			bar.realnew=False
			bar.iterate(bar.real)
			bar.measurement.header.stamp=rospy.Time.now()
		bar.pub.publish(bar.measurement)
		bar.rate.sleep()
