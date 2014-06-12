#!/usr/bin/env python

import roslib
import sys
import rospy
import numpy as np
from math import floor, exp, sqrt, atan2, sin, pi, asin, cos
import geomag
from geometry_msgs.msg import Point, Vector3, Quaternion

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

def sign(x): 
    return 1 if x >= 0 else -1

def Vector2Array(vector):
	return np.array([vector.x,vector.y,vector.z]).reshape(3,)

def saturation(val,minval,maxval):
	return max(minval,min(val,maxval))

def quat_normalize(q):
	q1=Quaternion()  	
  	qnorm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
  	q1.w = q.w/qnorm
 	q1.x = q.x/qnorm
  	q1.y = q.y/qnorm
  	q1.z = q.z/qnorm

	return q1

def vectornorm(a,b):
	cx = a.x - b.x
	cy = a.y - b.y
	cz = a.z - b.z
	return sqrt(cx*cx+cy*cy+cz*cz)

def quat_inverse (q):
	q_inv=Quaternion() 
	norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
  	q_inv.w = +q.w / norm
  	q_inv.x = -q.x / norm
  	q_inv.y = -q.y / norm
  	q_inv.z = -q.z / norm

	return q_inv

def quat_product (qa,qb):
	qc=Quaternion() 
  	qc.w = qa.w*qb.w - (qa.x*qb.x + qa.y*qb.y + qa.z*qb.z)
  	qc.x = (qa.w*qb.x + qa.x*qb.w + qa.y*qb.z - qa.z*qb.y)
  	qc.y = (qa.w*qb.y - qa.x*qb.z + qa.y*qb.w + qa.z*qb.x)
  	qc.z = (qa.w*qb.z + qa.x*qb.y - qa.y*qb.x + qa.z*qb.w)

	return qc

def quat_divide(qa,qb):
	return quat_product(qa,quat_inverse(qb))

def quat2Reb(quat):
	rotmtx=np.zeros((3,3))
  	rotmtx[0][0] = quat.w*quat.w + quat.x*quat.x - quat.y*quat.y - quat.z*quat.z
  	rotmtx[0][1] = 2.0*(quat.x*quat.y - quat.w*quat.z)
  	rotmtx[0][2] = 2.0*(quat.w*quat.y + quat.x*quat.z)
  	rotmtx[1][0] = 2.0*(quat.x*quat.y + quat.w*quat.z)
  	rotmtx[1][1] = quat.w*quat.w - quat.x*quat.x + quat.y*quat.y - quat.z*quat.z
  	rotmtx[1][2] = 2.0*(quat.y*quat.z - quat.w*quat.x)
  	rotmtx[2][0] = 2.0*(quat.x*quat.z - quat.w*quat.y)
  	rotmtx[2][1] = 2.0*(quat.w*quat.x + quat.y*quat.z)
  	rotmtx[2][2] = quat.w*quat.w - quat.x*quat.x - quat.y*quat.y + quat.z*quat.z
	
	return rotmtx

def euler2Reb(euler):
	rotmtx=np.zeros((3,3))
  	cpsi   = np.cos(euler.z)
	spsi   = np.sin(euler.z)
  	ctheta = np.cos(euler.y)
	stheta = np.sin(euler.y)
  	cphi   = np.cos(euler.x)
	sphi   = np.sin(euler.x)
  	""" Calculate rotation matrix"""
  	rotmtx[0][0] = cpsi * ctheta
  	rotmtx[0][1] = sphi * cpsi * stheta - cphi * spsi
  	rotmtx[0][2] = cphi * cpsi * stheta + sphi * spsi
  	rotmtx[1][0] = spsi * ctheta
  	rotmtx[1][1] = sphi * spsi * stheta + cphi * cpsi
  	rotmtx[1][2] = cphi * spsi * stheta - sphi * cpsi
  	rotmtx[2][0] = -stheta
  	rotmtx[2][1] = sphi * ctheta
  	rotmtx[2][2] = cphi * ctheta

	return rotmtx

def quat2euler2(q):
	euler=Vector3()
	q01 = q.w*q.x	
	q02 = q.w*q.y	
	q03 = q.w*q.z
	q11 = q.x*q.x	
	q12 = q.x*q.y
	q13 = q.x*q.z
	q22 = q.y*q.y
	q23 = q.y*q.z
	q33 = q.z*q.z
  	euler.z = atan2 (2.0 * (q03 + q12), 1.0 - 2.0 * (q22 - q33))
  	if euler.z<0.0:
   		euler.z+=2.0*pi
  
	euler.y = asin (2.0 * (q02 - q13))
 	euler.x = atan2 (2.0 * (q01 + q23), 1.0 - 2.0 * (q11 + q22))

	return euler

def euler2quat(euler):
	q = Quaternion()
	cpsi = cos (0.5 * euler.z)
	spsi = sin (0.5 * euler.z)
  	ctheta = cos (0.5 * euler.y)
	stheta = sin (0.5 * euler.y)
  	cphi = cos (0.5 * euler.x)
	sphi = sin (0.5 * euler.x)
  	q.w = cphi*ctheta*cpsi + sphi*stheta*spsi
  	q.x = sphi*ctheta*cpsi - cphi*stheta*spsi
  	q.y = cphi*stheta*cpsi + sphi*ctheta*spsi
  	q.z = cphi*ctheta*spsi - sphi*stheta*cpsi

	return q

def quat2euler(q):
	euler=Vector3()
	tol = 0.499;
	q0 = q.w
	q1 = q.x
	q2 = q.y
	q3 = q.z;
	q00 = q0*q0
	q11 = q1*q1
	q22 = q2*q2
	q33 = q3*q3
	q01 = q0*q1
	q23 = q2*q3
	q12 = q1*q2
	q03 = q0*q3
	test = q1*q3 - q0*q2
  	if (test < -tol):
		euler.x = 0.0
      		euler.y = 0.5 * pi
      		euler.z = 2.0 * atan2 (q0, q3)

 	elif (test > +tol):
     		euler.x = 0.0
      		euler.y = -0.5 * pi
      		euler.z = 2.0 * atan2 (q0, q3)
		return euler
  	else:
      		euler.x = atan2 (2.0*(q01 + q23), q00 - q11 - q22 + q33)
      		euler.y = asin (-2.0*test)
      		euler.z = atan2 (2.0*(q12 + q03), q00 + q11 - q22 - q33)
      		return euler

def getMag(lat,lon,alt,Reb):	
	Mb=Vector3()	
	alt_feet=alt*3.2808399
	mag= geomag.magnetic_data(lat,lon,alt_feet)

	Me=np.array([mag.bx, mag.by, mag.bz])
	M=np.dot(Reb.T,Me)
	Mb.x=M[0]
	Mb.y=M[1]
	Mb.z=M[2]

	return Mb,mag.dec
	
def lla2ECEF(lat,lon,alt,R_earth,e_earth):
	#intermediate variables
	pos = Vector3() 
	clat = np.cos(lat)
	slat = np.sin(lat)

	e2 = e_earth*e_earth
	#temp = 1.0-e2
	N = R_earth/sqrt(1.0-e2*slat*slat)
	A = (N+alt)*clat

	#get pos
	pos.x = A*np.cos(lon)
	pos.y = A*np.sin(lon)
	pos.z = ((1.0-e2)* N + alt)*slat

	return pos

def ECEF2lla(pos,R_earth,e_earth):
	#intermediate variables
	R_earth2=R_earth*R_earth
	e2 = e_earth*e_earth

	b   = sqrt(R_earth2*(1.0-e2));
	ep2 = (R_earth2-b*b)/(b*b);
	p   = sqrt(pos.x*pos.x+pos.y*pos.y)
	th  = atan2(R_earth*pos.z,b*p)
	
	sth = np.sin(th)
	cth= np.cos(th)

	#get lon,lat
	lon = atan2(pos.y,pos.x)
	lat = atan2((pos.z+ep2*b*sth*sth*sth),(p-e2*R_earth*cth*cth*cth))

	#get alt
	slat = np.sin(lat)
	N = R_earth/sqrt(1.0-e2*slat*slat)

	alt = pos.z/slat-(1.0-e2)*N

	return lat,lon,alt

def WGS84_NM(lat,R_earth,e_earth):

	sfi =sin(lat)
	e2 = e_earth*e_earth

	temp = 1.0-e2*sfi*sfi

	NE = R_earth / sqrt(temp)
	ME =  R_earth*(1.0-e2) / pow(temp,3.0/2.0)

	return NE,ME

#######################  Madgwick AHRS Class  #####################################################

class Madgwick():
	def __init__(self,orientation,Kp):
		self.orientation=orientation
		self.dt=1.0
		self.MwtwoKp=2.0*Kp

	def AHRSupdate(self,gx,gy,gz,ax,ay,az,mx,my,mz):
		#Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
		if((mx == 0.0) and (my == 0.0) and (mz == 0.0)):
			self.AHRSupdateIMU(gx, gy, gz, ax, ay, az)
			return

		q0 = self.orientation.w
		q1 = self.orientation.x
		q2 = self.orientation.y
		q3 = self.orientation.z

		#Rate of change of quaternion from gyroscope
		qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
		qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
		qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
		qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

		#Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(not ((ax == 0.0) and (ay == 0.0) and (az == 0.0))):

			#Normalise accelerometer measurement
			recipNorm = sqrt(ax * ax + ay * ay + az * az)
			ax /= recipNorm
			ay /= recipNorm
			az /= recipNorm

			# Normalise magnetometer measurement
			recipNorm = sqrt(mx * mx + my * my + mz * mz)
			mx /= recipNorm
			my /= recipNorm
			mz /= recipNorm

			# Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0 * q0 * mx
			_2q0my = 2.0 * q0 * my
			_2q0mz = 2.0 * q0 * mz
			_2q1mx = 2.0 * q1 * mx
			_2q0 = 2.0 * q0
			_2q1 = 2.0 * q1
			_2q2 = 2.0 * q2
			_2q3 = 2.0 * q3
			_2q0q2 = 2.0 * q0 * q2
			_2q2q3 = 2.0 * q2 * q3
			q0q0 = q0 * q0
			q0q1 = q0 * q1
			q0q2 = q0 * q2
			q0q3 = q0 * q3
			q1q1 = q1 * q1
			q1q2 = q1 * q2
			q1q3 = q1 * q3
			q2q2 = q2 * q2
			q2q3 = q2 * q3
			q3q3 = q3 * q3

			# Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3
			_2bx = sqrt(hx * hx + hy * hy)
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3
			_4bx = 2.0 * _2bx
			_4bz = 2.0 * _2bz

			# Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			recipNorm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)#invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) # normalise step magnitude
			s0 /= recipNorm
			s1 /= recipNorm
			s2 /= recipNorm
			s3 /= recipNorm

			# Apply feedback step
			qDot1 -= self.MwtwoKp * s0
			qDot2 -= self.MwtwoKp * s1
			qDot3 -= self.MwtwoKp * s2
			qDot4 -= self.MwtwoKp * s3

		# Integrate rate of change of quaternion to yield quaternion
		self.orientation.w += qDot1 * self.dt
		self.orientation.x += qDot2 * self.dt
		self.orientation.y += qDot3 * self.dt
		self.orientation.z += qDot4 * self.dt

		# Normalise quaternion
		self.orientation=quat_normalize(self.orientation)

	def AHRSupdateIMU(self,gx, gy, gz, ax, ay, az):
		q0 = self.orientation.w
		q1 = self.orientation.x
		q2 = self.orientation.y
		q3 = self.orientation.z

		# Rate of change of quaternion from gyroscope
		qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
		qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
		qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
		qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)


		# Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(not ((ax == 0.0) and (ay == 0.0) and (az == 0.0))):

			# Normalise accelerometer measurement
			recipNorm = sqrt(ax * ax + ay * ay + az * az)
			ax /= recipNorm
			ay /= recipNorm
			az /= recipNorm

			# Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2.0 * q0
			_2q1 = 2.0 * q1
			_2q2 = 2.0 * q2
			_2q3 = 2.0 * q3
			_4q0 = 4.0 * q0
			_4q1 = 4.0 * q1
			_4q2 = 4.0 * q2
			_8q1 = 8.0 * q1
			_8q2 = 8.0 * q2
			q0q0 = q0 * q0
			q1q1 = q1 * q1
			q2q2 = q2 * q2
			q3q3 = q3 * q3

			# Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
			s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
			s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
			recipNorm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) # normalise step magnitude
			s0 /= recipNorm
			s1 /= recipNorm
			s2 /= recipNorm
			s3 /= recipNorm

			# Apply feedback step
			qDot1 -= self.MwtwoKp * s0
			qDot2 -= self.MwtwoKp * s1
			qDot3 -= self.MwtwoKp * s2
			qDot4 -= self.MwtwoKp * s3
	
		# Integrate rate of change of quaternion to yield quaternion
		self.orientation.w += qDot1 * self.dt
		self.orientation.x += qDot2 * self.dt
		self.orientation.y += qDot3 * self.dt
		self.orientation.z += qDot4 * self.dt

############################# END of class ########################################################

#######################  Class: model the rise in temperature for electronic components  ##########

#Model for rise in temperature for electronic components
class ThermalRise():
	def __init__ (self,name):
		#self.Pth = rospy.get_param(name+'/ThermalPower', 10.0)
		self.kAir = rospy.get_param(name+'/kAir', 0.06)
		self.kCov = rospy.get_param(name+'/kCov', 0.5)
		self.DeltaT = rospy.get_param(name+'/DeltaT', 10.0)
		self.Tr = rospy.get_param(name+'/Tr', 300.0)
		self.tempOffset = 0.0
		if self.kAir==0.06 and self.kCov==0.5 and self.DeltaT==10.0 and self.Tr == 300.0:
			rospy.loginfo("\t-Thermal Defaults")

	def step(self,airspeed,dt):
		self.Rth = self.DeltaT*self.kCov/(1.0 + self.kAir*airspeed)
		#self.Cth = self.Rth/self.DeltaT
		self.tempOffset = (1.0 - 1/self.Tr*dt)*self.tempOffset + self.Rth/self.Tr*dt
		return self.tempOffset 

############################# END of class ########################################################
