#!/usr/bin/env python

import roslib
import sys
import rospy
import ephem
from urllib2 import urlopen
import datetime


from geometry_msgs.msg import Vector3, Vector3Stamped
import numpy as np
from numpy.linalg import inv, lstsq
from utils import saturation, quat2Reb, Vector2Array, quat2euler, ECEF2lla, lla2ECEF, vectornorm
from math import floor, exp, atan2, pi, sqrt, sin, cos
from last_letter.msg import SimStates, SimGPS, SimSats

lspeed=299792458.0
_grav=9.81

def block_diag(*arrs):
    """Create a new diagonal matrix from the provided arrays.

    Parameters
    ----------
    a, b, c, ... : ndarray
        Input arrays.

    Returns
    -------
    D : ndarray
        Array with a, b, c, ... on the diagonal.

    """
    arrs = [np.asarray(a) for a in arrs]
    shapes = np.array([a.shape for a in arrs])
    out = np.zeros(np.sum(shapes, axis=0))

    r, c = 0, 0
    for i, (rr, cc) in enumerate(shapes):
        out[r:r + rr, c:c + cc] = arrs[i]
        r += rr
        c += cc
    return out
		

####################### subfunctions #############################################################
def chunks(l, n):
	""" Yield successive n-sized chunks from l"""
	for i in xrange(0, len(l), n):
		yield l[i:i+n]

#######################  Magnetometer Class  #####################################################

class GPSsensor():
	def __init__(self,name):
		
		rospy.loginfo('Create ' + name +' Sensor')
		
		self.maxdt = 1.0/rospy.get_param(name+'/maxrate', 100.0)
		self.maxrate =  rospy.Rate(rospy.get_param(name+'/maxrate', 100.0))
		rospy.loginfo("\ta.maxrate: %.1f Hz",1.0/self.maxdt)

		self.dt = 1.0/rospy.get_param(name+'/rate', 100.0)
		self.rate =  rospy.Rate(rospy.get_param(name+'/rate', 100.0))
		rospy.loginfo("\tb.rate: %.1f Hz",1.0/self.dt)


		self.cp =  np.array(rospy.get_param(name+'/CP', [0.0, 0.0, 0.0])).reshape(3,1)
		rospy.loginfo("\tc.position: [%.2f,%.2f,%.2f]", self.cp[0],self.cp[1],self.cp[2])

		self.precision =  np.array(rospy.get_param(name+'/precision', [3e-9, 1e-2, 1e-1, 1e-1, 1e-1])).reshape(5,1)# np.array([3e-9, 1e-2, 1e-1, 1e-1, 1e-1]).reshape(5,1)

		self.cbias =  rospy.get_param(name+'/clock_bias', 1.2)
		rospy.loginfo("\td.clock bias: %.2f sec",self.cbias)

		self.std_coeff = rospy.get_param(name+'/std_coeff', 10.0)
		rospy.loginfo("\te.noise coeff: %.1f meters ",self.std_coeff)
		
		self.measurement = SimGPS()

		self.sat_glonass, self.sat_sbas, self.sat_gpsop =  self.get_satellites()

		self.pub = rospy.Publisher(name, SimGPS)
		self.states_sub = rospy.Subscriber("states",SimStates,self.StatesCallback)

		self.real=SimStates()
		self.realnew=False
		self.place = ephem.Observer()
		self.sats=SimSats()
		
		self.firstime=True
		self.position=Vector3(6.378e6,0,0)
		self.ctime = 0.0

		self.maxcount = int(self.dt/self.maxdt)
		self.av=[]
		self.xyzold=[0, 0, 0]

		self.Kstates = np.zeros((8,))
		self.KP = 10.0*np.eye(8)

		Sf = 36.0
		Sg = 0.1
		sigma=5.0
		T3 = pow(self.dt,3)/3.0
		T2 = pow(self.dt,2)/2.0
		self.KR = 36.0;

		self.speedold=[0,0,0]
		self.speed3=Vector3()

		Qxyz = sigma*sigma*np.array([T3, T2, T2, self.dt]).reshape(2,2)
		
		Qb = np.array([Sf*self.dt+Sg*T3, Sg*T2, Sg*T2, Sg*self.dt]).reshape(2,2)
		self.KQ =  block_diag(Qxyz,Qxyz,Qxyz,Qb)

		Jacobx = np.array([1.0,self.dt,0.0,1.0]).reshape(2,2)
		self.Jx =  block_diag(Jacobx,Jacobx,Jacobx,Jacobx)
		
	def get_satellites(self):
    		GLONASS='http://celestrak.com/NORAD/elements/glo-ops.txt'
    		SBAS = 'http://celestrak.com/NORAD/elements/sbas.txt'
		GPSO = 'http://www.celestrak.com/NORAD/elements/gps-ops.txt'
    
		rospy.loginfo("[GPS] Get Glonass Sat. data") 
    		r = urlopen(GLONASS).read()
    		data = r.split('\r\n')

    		sat_glonass = []
    		for tle in chunks(data, 3):
			if len(tle)==3:
				sat_glonass.append(ephem.readtle(str(tle[0]), str(tle[1]), str(tle[2])))

		rospy.loginfo("[GPS] Get SBAS Sat. data") 
   		r = urlopen(SBAS).read()
    		data = r.split('\r\n')

    		sat_sbas=[]
    		for tle in chunks(data, 3):
			if len(tle)==3:
				a=str(tle[0])
				if a.find('EGNOS')>0:
		 			sat_sbas.append(ephem.readtle(str(tle[0]), str(tle[1]), str(tle[2])))
		rospy.loginfo("[GPS] Get GPS Operational Sat. data")
		r = urlopen(GPSO).read()
		data = r.split('\r\n')

		sat_gosop=[]
    		for tle in chunks(data, 3):
			if len(tle)==3:
		 		sat_gosop.append(ephem.readtle(str(tle[0]), str(tle[1]), str(tle[2])))
		
    		return sat_glonass,sat_sbas,sat_gosop

	def getVisible(self,pos,sat):
		for iss in sat:
			iss.compute(self.place)
			bad=abs(np.random.normal(0,3000,1))
			if float(repr(iss.alt)) > 15.0*pi/180.0 and float(repr(iss.alt))<pi/2.0 and not bad>=9900:
				possat=lla2ECEF(iss.sublat,iss.sublong,iss.elevation)
				self.sats.sat[self.sats.visible].position.x = possat.x
				self.sats.sat[self.sats.visible].position.y = possat.y
				self.sats.sat[self.sats.visible].position.z = possat.z
				self.sats.sat[self.sats.visible].range=vectornorm(possat,pos)
				self.sats.visible=self.sats.visible+1		
		self.measurement.satellites=self.sats.visible

	def getdop(self,pos):
		dop=[]
		for i in range(0,self.sats.visible):
			dop.append((pos.x-self.sats.sat[i].position.x)/self.sats.sat[i].range)
			dop.append((pos.y-self.sats.sat[i].position.y)/self.sats.sat[i].range)
			dop.append((pos.z-self.sats.sat[i].position.z)/self.sats.sat[i].range)
			dop.append(1.0)

		dop=-np.array(dop).reshape(self.sats.visible,4)
		G= inv(np.dot(dop.T,dop))
		self.measurement.dop.g = sqrt(np.trace(G))
		self.measurement.dop.p = sqrt(np.trace(G[0:3,0:3]))
		self.measurement.dop.h = sqrt(np.trace(G[0:2,0:2]))
   		self.measurement.dop.v = sqrt(G[2,2])
    		self.measurement.dop.t = sqrt(G[3,3])

	def setNoise(self):
		for i in range(0,self.sats.visible):
			self.sats.sat[i].range = self.sats.sat[i].range + self.cbias*lspeed + np.random.normal(0,self.std_coeff,1)

	def getPosition(self,k):
		cosine_vector=Vector3()
		G = np.zeros((self.sats.visible,4))
		dr = np.zeros((self.sats.visible,))
		Rc = 0.01#*np.eye(self.sats.visible)
		if self.sats.visible>3.0:
			if self.firstime:
				for i in range(0,10):
					for x in range(0,self.sats.visible):
						cosine_vector.x=self.sats.sat[x].position.x-self.position.x
						cosine_vector.y=self.sats.sat[x].position.y-self.position.y
						cosine_vector.z=self.sats.sat[x].position.z-self.position.z
						r_ref = sqrt(cosine_vector.x*cosine_vector.x+cosine_vector.y*cosine_vector.y+cosine_vector.z*cosine_vector.z)
						G[x,0] = -cosine_vector.x/r_ref
						G[x,1] = -cosine_vector.y/r_ref
						G[x,2] = -cosine_vector.z/r_ref
						G[x,3] = 1.0
						dr[x] = self.sats.sat[x].range - r_ref -self.ctime
				
				 	#N=np.dot(G.T,Rc)
				 	#c=np.dot(N,dr)
				 	#N=np.dot(N,G)
						
				 	#dx = lstsq(N,c)[0]
					dx = lstsq(G,dr)[0]

					self.position.x = self.position.x + dx[0]
					self.position.y = self.position.y + dx[1]
					self.position.z = self.position.z + dx[2]
					self.ctime = self.ctime + dx[3]

				self.Kstates[0]=self.position.x
				self.Kstates[2]=self.position.y
				self.Kstates[4]=self.position.z
				self.Kstates[6]=self.ctime

				self.firstime=False

			else:
				self.getPositionKalman()
				self.measurement.fix=1
				if (k>0):
					self.measurement.fix=2
				
				#for x in range(0,self.sats.visible):
				#		cosine_vector.x=self.sats.sat[x].position.x-self.position.x
				#		cosine_vector.y=self.sats.sat[x].position.y-self.position.y
				#		cosine_vector.z=self.sats.sat[x].position.z-self.position.z
				#		r_ref = sqrt(cosine_vector.x*cosine_vector.x+cosine_vector.y*cosine_vector.y+cosine_vector.z*cosine_vector.z)

				#		G[x,0] = -cosine_vector.x/r_ref
				#		G[x,1] = -cosine_vector.y/r_ref
				#		G[x,2] = -cosine_vector.z/r_ref
				#		G[x,3] = 1.0

				#		dr[x] = self.sats.sat[x].range - r_ref -self.ctime#*lspeed
						
				#N=np.dot(G.T,Rc)
				#c=np.dot(N,dr)
				#N=np.dot(N,G)
						
				#dx = lstsq(N,c)[0]
				#dx = lstsq(G,dr)[0]

				#self.position.x = self.position.x + dx[0]
				#self.position.y = self.position.y + dx[1]
				#self.position.z = self.position.z + dx[2]
				#self.ctime = self.ctime + dx[3]#/lspeed

				#print "pos"
				#print self.position.x-pos.x
				#print self.position.y-pos.y
				#print self.position.z-pos.z
				#print self.ctime/lspeed
								
	def calculateMeas(self):
		
		sf= sin(self.measurement.latitude)
		cf = cos(self.measurement.latitude)
		sl= sin(self.measurement.longitude)
		cl = cos(self.measurement.longitude)

		RNe=np.array([-sf*cl, -sf*sl, cf,
     		     	      -sl,    cl,     0.0,    
                              -cf*cl, -cf*sl, -sf  ]).reshape(3,3)

		self.measurement.latitude, self.measurement.longitude, self.measurement.altitude = ECEF2lla(Vector3(self.Kstates[0],self.Kstates[2],self.Kstates[4]))
		
		speedE = np.array([self.Kstates[1],self.Kstates[3],self.Kstates[5]]).reshape(3,)
		#print RNe
		speed = np.dot(RNe,speedE)

		a2=-0.1
		b1=0.7
		b2=0.2
		self.speed3.x=-a2*self.speed3.x+b1*speed[0]+b2*self.speedold[0]
		self.speed3.y=-a2*self.speed3.y+b1*speed[1]+b2*self.speedold[1]
		self.speed3.z=-a2*self.speed3.z+b1*speed[2]+b2*self.speedold[2]

		self.speedold=speed

		self.measurement.speed = sqrt(speed[0]*speed[0]+speed[1]*speed[1])*1.94384449#sqrt(self.speed3.x*self.speed3.x+self.speed3.y*self.speed3.y)*1.94384449
		self.measurement.vspeed = speed[2]#self.speed3.z
		self.measurement.course = atan2(speed[1],speed[0])*180.0/pi#atan2(self.speed3.y,self.speed3.x)*180.0/pi


		self.measurement.header.stamp=rospy.Time.now()
		
		self.measurement.latitude = floor(self.measurement.latitude/self.precision[0])*self.precision[0]
		self.measurement.longitude = floor(self.measurement.longitude/self.precision[0])*self.precision[0] 
		self.measurement.altitude = floor(self.measurement.altitude/self.precision[1])*self.precision[1]

		self.measurement.speed = floor(self.measurement.speed/self.precision[2])*self.precision[2]/1.94384449
		self.measurement.vspeed = floor(self.measurement.vspeed/self.precision[2])*self.precision[2]
		self.measurement.course = floor(self.measurement.course/self.precision[3])*self.precision[3]

		self.measurement.dop.g = floor(self.measurement.dop.g/self.precision[4])*self.precision[4]
		self.measurement.dop.p = floor(self.measurement.dop.p/self.precision[4])*self.precision[4]
		self.measurement.dop.h = floor(self.measurement.dop.h/self.precision[4])*self.precision[4]
		self.measurement.dop.v = floor(self.measurement.dop.v/self.precision[4])*self.precision[4]
		self.measurement.dop.t = floor(self.measurement.dop.t/self.precision[4])*self.precision[4]
		
		
		self.pub.publish(self.measurement)
		
		#self.av.append(self.position)
		#if (len(self.av)==self.maxcount):
		#	x=0.0
		#	y=0.0
		#	z=0.0
		#	self.count=0
		#	for avi in self.av:
		#		x=x+avi.x
		#		y=y+avi.y
		#		z=z+avi.z

		#	x=x/float(self.maxcount)
		#	y=y/float(self.maxcount)
		#	z=z/float(self.maxcount)
			
		#	self.measurement.latitude, self.measurement.longitude, self.measurement.altitude = ECEF2lla(Vector3(x,y,z))
			
			#print (x-self.xyzold[0])/self.dt
			#print (y-self.xyzold[1])/self.dt
			#print (z-self.xyzold[2])/self.dt
			#self.xyzold[0]=x
			#self.xyzold[1]=y
			#self.xyzold[2]=z
		#	self.pub.publish(self.measurement)
			
		#	self.av=[]

	def iterate(self,states):
		self.measurement.fix=0
		
		self.place.lat = states.geoid.latitude
		self.place.long = states.geoid.longitude
		self.place.elevation = states.geoid.altitude
		self.place.date = datetime.datetime.now()

		pos = lla2ECEF(self.place.lat,self.place.long,self.place.elevation)

		self.sats.visible=0

		self.getVisible(pos,self.sat_sbas)
		k=self.sats.visible
		self.getVisible(pos,self.sat_gpsop)

		self.setNoise()

		self.getPosition(k)

		self.getdop(self.position)

		self.calculateMeas()
		

	def StatesCallback(self,data):
		self.real=data
		self.realnew=True
		

	def getPositionKalman(self):
		#Model Propagation
		self.Kstates[0] =  self.Kstates[0] + self.dt*self.Kstates[1]
		self.Kstates[2] =  self.Kstates[2] + self.dt*self.Kstates[3]
		self.Kstates[4] =  self.Kstates[4] + self.dt*self.Kstates[5]
		self.Kstates[6] =  self.Kstates[6] + self.dt*self.Kstates[7]

		#Sensor Model
		Jz = np.zeros((self.sats.visible,8))
		ze = np.zeros((self.sats.visible,))
		Zm = np.zeros((self.sats.visible,))
		for i in range(0,self.sats.visible):
			dx=self.Kstates[0]-self.sats.sat[i].position.x
			dy=self.Kstates[2]-self.sats.sat[i].position.y
			dz=self.Kstates[4]-self.sats.sat[i].position.z
			ze[i]=sqrt(dx*dx+dy*dy+dz*dz)
			Jz[i,0]=dx/ze[i]
			Jz[i,2]=dy/ze[i]
			Jz[i,4]=dz/ze[i]
			Jz[i,6]=1.0
			Zm[i] = self.sats.sat[i].range
			ze[i]= ze[i] + self.Kstates[6]

		R=self.KR*np.eye(self.sats.visible)

		#Kalman P,K
		self.KP = np.dot(self.Jx,np.dot(self.KP,self.Jx.T)) + self.KQ
		
		K1 = np.dot(self.KP,Jz.T)
		K2 = np.dot(Jz,np.dot(self.KP,Jz.T))+R

		K = np.dot(K1,inv(K2))

		#New P, states
		self.Kstates = self.Kstates + 0.8*np.dot(K,(Zm-ze))
		
		self.KP = np.dot((np.eye(8)-np.dot(K,Jz)),self.KP)
		
###################################################################################################	
#######################  Main Program  ############################################################
###################################################################################################	
if __name__ == '__main__':
	rospy.init_node('acc_model')

	fullname = rospy.get_name().split('/')
	gps = GPSsensor(fullname[-1])

	maxcount = gps.dt/gps.maxdt

	while not rospy.is_shutdown():
		if gps.realnew:
			gps.realnew=False
			gps.iterate(gps.real)
			gps.measurement.header.stamp=rospy.Time.now()

		#if (count==maxcount):
		#	count=0
		#	gps.pub.publish(gps.measurement)
		gps.maxrate.sleep()
