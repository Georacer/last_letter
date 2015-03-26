#!/usr/bin/env python

import roslib
import sys
import rospy
import numpy as np
from math import floor, exp, sqrt, atan2, sin, pi, asin, cos
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
