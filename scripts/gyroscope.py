#!/usr/bin/env python

import roslib
import sys
import rospy
from geometry_msgs.msg import Vector3, Vector3Stamped
import numpy as np
from utils import saturation, quat2Reb, ThermalRise
from math import floor, exp
from last_letter.msg import SimStates, SimSensor3, Environment

#######################  Gyroscope Class  #########################################################

class Gyroscope():
	def __init__(self,name):

		rospy.loginfo('Create ' + name +' Sensor')

		self.dt = 1.0/rospy.get_param(name+'/rate', 100.0)
		self.rate =  rospy.Rate(rospy.get_param(name+'/rate', 100.0))
		rospy.loginfo("\ta.rate: %.1f Hz",1.0/self.dt)

		self.cp =  np.array(rospy.get_param(name+'/CP', [0.0, 0.0, 0.0])).reshape(3,1)
		rospy.loginfo("\tb.position: [%.2f,%.2f,%.2f]", self.cp[0],self.cp[1],self.cp[2])

		self.resolution = rospy.get_param(name+'/resolution', 0.0001)
		rospy.loginfo("\tc.resolution: %.1e rad/s",self.resolution)

		self.maxvalue = rospy.get_param(name+'/max', 9.0)
		rospy.loginfo("\td.maxvalue: %.2f rad/s",self.maxvalue)

		self.std_coeff = rospy.get_param(name+'/std_coeff', [0.0,0.0,0.0])
		rospy.loginfo("\te.noise coeff:")
		rospy.loginfo("\t\t1.main std: %.3f",self.std_coeff[0])
		rospy.loginfo("\t\t2.bias coeff: %.3f",self.std_coeff[1])
		rospy.loginfo("\t\t3.random walk coeff: %.3f",self.std_coeff[2])

		iT=1.0/120.0
		self.ad = np.exp(iT*self.dt)
		self.bd = -iT*np.exp(-iT*self.dt)+iT
		self.bias=Vector3()
		self.randomwalk=Vector3()
		self.noise_main=Vector3()
		self.Vib=Vector3()

		self.temp=25.0
		
		self.measurement = SimSensor3()

		self.pub = rospy.Publisher(name, SimSensor3)
		self.states_sub = rospy.Subscriber("states",SimStates,self.StatesCallback)
		self.atm_sub = rospy.Subscriber("environment",Environment,self.EnvironmentCallback)

		self.real=SimStates()
		self.realnew=False
		
		#Thermal Chracteristics
		self.thermalMass = ThermalRise(name)
		self.airspeed = 0.0
		self.temp=25.0


	def iterate(self,states):
		Reb = quat2Reb(states.pose.orientation)
		nm_noise = np.random.normal(0,self.std_coeff[0],3)
		bias_noise = np.random.normal(0,self.std_coeff[1],3)
		rw_noise = np.random.normal(0,self.std_coeff[2],3)

		self.bias.x +=-self.ad*self.bias.x+self.bd*bias_noise[0]
		self.bias.y +=-self.ad*self.bias.y+self.bd*bias_noise[1]
		self.bias.z +=-self.ad*self.bias.z+self.bd*bias_noise[2]

		self.noise_main.x = nm_noise[0]
		self.noise_main.y = nm_noise[1]
		self.noise_main.z = nm_noise[2]
		
		self.randomwalk.x +=self.dt*0.1*rw_noise[0]
		self.randomwalk.y +=self.dt*0.1*rw_noise[1]
		self.randomwalk.z +=self.dt*0.1*rw_noise[2]

		self.measurement.axis.x = states.velocity.angular.x + self.bias.x + self.noise_main.x + self.randomwalk.x
		self.measurement.axis.y = states.velocity.angular.y + self.bias.y + self.noise_main.y + self.randomwalk.y
		self.measurement.axis.z = states.velocity.angular.z + self.bias.z + self.noise_main.z + self.randomwalk.z

		self.measurement.axis.x = saturation(floor(self.measurement.axis.x/self.resolution)*self.resolution,-self.maxvalue, self.maxvalue)
		self.measurement.axis.y = saturation(floor(self.measurement.axis.y/self.resolution)*self.resolution,-self.maxvalue, self.maxvalue)
		self.measurement.axis.z = saturation(floor(self.measurement.axis.z/self.resolution)*self.resolution,-self.maxvalue, self.maxvalue)
		
		self.tempOffset = self.thermalMass.step(self.airspeed)
		self.measurement.temperature = self.temp + self.tempOffset


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
	rospy.init_node('gyro_model')

	fullname = rospy.get_name().split('/')
	gyro = Gyroscope(fullname[-1])

	while not rospy.is_shutdown():
		if gyro.realnew:
			gyro.realnew=False
			gyro.iterate(gyro.real)
			gyro.measurement.header.stamp=rospy.Time.now()
		gyro.pub.publish(gyro.measurement)
		gyro.rate.sleep()
