#!/usr/bin/env python3
"""General mathematics library

This library is meant to host mathematic utilities of general application.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos", "Peter Seres"]
__credits__ = []
__date__ = "Fri 22 Oct 2021"
__copyright__ = "Copyright 2021, George Zogopoulos"

from dataclasses import dataclass
from typing import Optional

import numpy as np
from numba import jit

# from pytransform3d import rotations as pt3d_rot
from scipy.optimize import root

# from last_letter_lib import cpp_last_letter_lib
from ..cpp_last_letter_lib.cpp_math_utils import EulerAngles
from ..cpp_last_letter_lib.cpp_math_utils import Inertial
from ..cpp_last_letter_lib.cpp_math_utils import UnitQuaternion
from ..cpp_last_letter_lib.cpp_math_utils import Vector3
from ..cpp_last_letter_lib.cpp_uav_utils import Pose as cpp_Pose
from ..cpp_last_letter_lib.cpp_uav_utils import Wrench


@jit(nopython=True)
def hat(v: np.ndarray = np.zeros(3)):
    """Skew-symmetric matrix in SO(3) from 3D vector."""
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


@jit(nopython=True)
def vex(m: np.ndarray = np.eye(3)):
    """3D vector from skew-symmetric matrix in SO(3)."""

    return np.array([m[2, 1], m[0, 2], m[1, 0]])


@jit(nopython=True)
def symproj(H: np.ndarray = np.eye(3)):
    """Symmetric projection of a square matrix."""

    return (H + H.T) / 2


@jit(nopython=True)
def asymproj(H: np.ndarray = np.eye(3)):
    """Anti-symmetric projection of a square matrix."""

    return (H - H.T) / 2


def build_vector3_from_array(arr):
    arr = np.array(arr)  # Make sure arr is a numpy object
    # Convert a numpy array to Vector3
    if len(arr.shape) == 1:  # arr is a 1-dimensional array
        x = arr[0]
        y = arr[1]
        z = arr[2]
    else:  # Arr is a 2-dimensional vertical vector array
        x = arr[0, 0]
        y = arr[1, 0]
        z = arr[2, 0]
    return Vector3(x, y, z)


# @dataclass
# class Inertial:
#     mass: float  # The object mass, in kg
#     tensor: Optional[np.array]  # The object inertia tensor, a numpy 9x9 array

#     @classmethod
#     def simple(cls, mass, i_xx, i_yy, i_zz):
#         return cls(mass, np.array([[i_xx, 0, 0], [0, i_yy, 0], [0, 0, i_zz]]))


# class EulerAngles:
#     def __init__(self, roll=None, pitch=None, yaw=None, in_degrees=False):
#         if roll is None:
#             roll = 0.0
#         if pitch is None:
#             pitch = 0.0
#         if yaw is None:
#             yaw = 0.0
#         if in_degrees:
#             roll = np.deg2rad(roll)
#             pitch = np.deg2rad(pitch)
#             yaw = np.deg2rad(yaw)

#         self.__roll = roll
#         self.__pitch = pitch
#         self.__yaw = yaw

#     def __repr__(self):
#         return "EulerAngles[deg](roll = {:}, pitch = {:}, yaw = {:})".format(
#             np.rad2deg(self.__roll),
#             np.rad2deg(self.__pitch),
#             np.rad2deg(self.__yaw),
#         )

#     def __eq__(self, other):
#         is_euler = isinstance(other, EulerAngles)
#         is_numpy = type(other) is np.array or type(other) is np.ndarray
#         is_list = type(other) is list
#         if is_numpy or is_list:
#             return self == EulerAngles.from_array(other)
#         elif is_euler:
#             return (
#                 self.roll == other.roll
#                 and self.pitch == other.pitch
#                 and self.yaw == other.yaw
#             )
#         else:
#             raise TypeError(
#                 f"Unsupported comparison between {self.__class__.__name__} and {other.__class__.__name__}"
#             )

#     # Class constructors
#     @staticmethod
#     def from_array(v, in_degrees=False):
#         """Read EulerAngles object from a numpy vector."""

#         return EulerAngles(v[0], v[1], v[2], in_degrees=in_degrees)

#     @staticmethod
#     def from_rotmat(R):
#         """Generate EulerAngles object from RotationMatrix R_bi"""

#         euler_zyx = pt3d_rot.intrinsic_euler_zyx_from_active_matrix(R)

#         return EulerAngles(euler_zyx[2], euler_zyx[1], euler_zyx[0])

#     @classmethod
#     def from_quaternion(cls, quaternion):

#         """Generate euler angles froquaternionm unit quaternion."""

#         if type(quaternion) is UnitQuaternion:
#             q = quaternion.to_array()
#         elif type(quaternion) is np.ndarray:
#             q = quaternion
#         else:
#             raise TypeError("Input has to be 4D vector or quaternion.")

#         R = pt3d_rot.matrix_from_quaternion(q)
#         return cls.from_rotmat(R)

#     # Class representations
#     def to_array(self, in_degrees=False):
#         """Represent class as a 3-by-1 vector."""

#         output = np.array([self.__roll, self.__pitch, self.__yaw])

#         if in_degrees:
#             output = np.rad2deg(output)

#         return output

#     def to_quaternion(self):
#         return UnitQuaternion.from_euler(self)

#     # Accessors
#     @property
#     def roll(self):
#         return self.__roll

#     @property
#     def pitch(self):
#         return self.__pitch

#     @property
#     def yaw(self):
#         return self.__yaw

#     # Transformation functions
#     def R_roll(self):
#         """Generate (inertial-to-body) rotation matrix due to roll."""

#         sPH = np.sin(self.__roll)
#         cPH = np.cos(self.__roll)

#         return np.array([1, 0, 0, 0, cPH, sPH, 0, -sPH, cPH]).reshape(3, 3)

#     def R_pitch(self):
#         """Generate (inertial-to-body) rotation matrix due to pitch."""

#         sTH = np.sin(self.__pitch)
#         cTH = np.cos(self.__pitch)
#         return np.array([cTH, 0, -sTH, 0, 1, 0, sTH, 0, cTH]).reshape(3, 3)

#     def R_yaw(self):
#         """Generate (inertial-to-body) rotation matrix due to yaw."""

#         sPS = np.sin(self.__yaw)
#         cPS = np.cos(self.__yaw)

#         return np.array([cPS, sPS, 0, -sPS, cPS, 0, 0, 0, 1]).reshape(3, 3)

#     def R_bi(self):
#         """Generate 3-by-3 rotation matrix from body to inertial frame (NED)"""
#         return pt3d_rot.active_matrix_from_intrinsic_euler_zyx(
#             reversed(self.to_array())
#         )

#     def R_ib(self):
#         """Generate 3-by-3 rotation matrix from inertial (NED) to body frame."""
#         return self.R_bi().T

#     def T_eb(self):
#         """Generate 3-by-3 transformation matrix from Euler-rates to body-rates."""

#         sPH = np.sin(self.__roll)
#         cPH = np.cos(self.__roll)
#         sTH = np.sin(self.__pitch)
#         cTH = np.cos(self.__pitch)

#         return np.array([1, 0, -sTH, 0, cPH, sPH * cTH, 0, -sPH, cPH * cTH]).reshape(
#             3, 3
#         )

#     def T_be(self):
#         """Generate 3-by-3 transformation matrix from body-rates to Euler-rates."""

#         sPH = np.sin(self.__roll)
#         cPH = np.cos(self.__roll)
#         cTH = np.cos(self.__pitch)
#         tTH = np.tan(self.__pitch)
#         secTH = 1 / cTH
#         if not np.isfinite(secTH):
#             raise ValueError("Euler angles in gimbal lock, can't calculate T_be.")

#         return np.array(
#             [1, sPH * tTH, cPH * tTH, 0, cPH, -sPH, 0, sPH * secTH, cPH * secTH]
#         ).reshape(3, 3)


# class UnitQuaternion:
#     def __init__(self, w: float = 1.0, xyz: np.ndarray = np.zeros(3)):
#         """Quaternion constructor
#         >> Quaternion()
#         Quaternion(1.0 + 0.0*i + 0.0*j + 0.0*k)

#         >> Quaternion(4, xyz = [1,2,3])
#         Quaternion(4.0 + 1.0*i + 2.0*j + 3.0*k)

#         >> Quaternion(4, yxz = np.array([1,2,3]))
#         Quaternion(4.0 + 1.0*i + 2.0*j + 3.0*k)
#         """

#         self._q = np.array([w, xyz[0], xyz[1], xyz[2]])
#         self._normalize()

#     @property
#     def __w(self):
#         return self._q[0]

#     @property
#     def __x(self):
#         return self._q[1]

#     @property
#     def __y(self):
#         return self._q[2]

#     @property
#     def __z(self):
#         return self._q[3]

#     @property
#     def norm(self):
#         return self._static_norm(self._q)

#     @staticmethod
#     @jit(nopython=True)
#     def _static_norm(q):
#         return np.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)

#     @property
#     def is_unit(self, tolerance=1e-17):
#         return abs(1.0 - self.norm) < tolerance

#     @property
#     def real(self):
#         """Real / Scalar part of quaternion."""
#         return self._q[0]

#     @property
#     def imag(self):
#         """Imaginary / Vector part of a quaternion"""
#         return self._static_imag(self._q)

#     @staticmethod
#     @jit(nopython=True)
#     def _static_imag(q: np.array):
#         return np.array([q[1], q[2], q[3]])

#     def _normalize(self):
#         if not self.is_unit:
#             if self.norm > 0:
#                 self._q = self._q / self.norm

#     def conjugate(self):
#         return UnitQuaternion(self.real, -self.imag)

#     def inverse(self):
#         """inverse = conjugate / norm for general quaternions. Unit quats are already normalized."""
#         return UnitQuaternion(self.real, -self.imag)

#     def flipped(self):
#         """The quaternion on the opposite side of the 4D sphere."""
#         return UnitQuaternion(-self.real, self.imag)

#     def to_array(self):
#         """Returns the quaternion elements as a 4D numpy array."""

#         return self._q

#     def to_prodmat(self):
#         """
#         Return quaternion product matrix (Kronecker matrix):

#                np.ndarray([ [self.__w, -self.__x, -self.__y, -self.__z],
#                             [self.__x,  self.__w, -self.__z,  self.__y],
#                             [self.__y,  self.__z,  self.__w, -self.__x],
#                             [self.__z, -self.__y,  self.__x,  self.__w]])
#         """
#         return self._static_to_prodmat(self._q)

#     @staticmethod
#     @jit(nopython=True)
#     def _static_to_prodmat(q):
#         return np.array(
#             [
#                 [q[0], -q[1], -q[2], -q[3]],
#                 [q[1], q[0], -q[3], q[2]],
#                 [q[2], q[3], q[0], -q[1]],
#                 [q[3], -q[2], q[1], q[0]],
#             ]
#         )

#     def _quatprod(self, q):
#         """Quaternion Multiplication."""

#         return UnitQuaternion.from_wxyz(self.to_prodmat() @ q.to_array())

#     def _quatrot(self, v: np.ndarray = np.ones(3)):
#         """
#         Rotates a 3D vector

#         q.conjugated().quatrot(v) -> transforms 'v' from inertial to body frame
#         q.quatrot(v) -> transforms 'v' from body to inertial frame
#         """

#         q_inv = self.inverse().to_array()
#         V = UnitQuaternion(0, v).to_prodmat() * np.linalg.norm(v)
#         Q = self.to_prodmat()
#         v_rot = np.matmul(Q, np.matmul(V, q_inv))  # v = Q * v * conj(q)

#         return v_rot[1:]

#     def unitX(self):
#         """Return X axis of respective rotation matrix (body X)."""

#         return np.array(
#             [
#                 self.__w**2 + self.__x**2 - self.__y**2 - self.__z**2,
#                 2 * (self.__x * self.__y + self.__w * self.__z),
#                 2 * (self.__x * self.__z - self.__w * self.__y),
#             ]
#         )

#     def unitY(self):
#         """Return Y axis of respective rotation matrix (body Y)."""

#         return np.array(
#             [
#                 2 * (self.__x * self.__y - self.__w * self.__z),
#                 self.__w**2 + self.__y**2 - self.__x**2 - self.__z**2,
#                 2 * (self.__y * self.__z + self.__w * self.__x),
#             ]
#         )

#     def unitZ(self):
#         """Return Z axis of respective rotation matrix (body Z)."""

#         return np.array(
#             [
#                 2 * (self.__x * self.__z + self.__w * self.__y),
#                 2 * (self.__y * self.__z - self.__w * self.__x),
#                 self.__w**2 + self.__z**2 - self.__x**2 - self.__y**2,
#             ]
#         )

#     def R_ib(self):
#         """
#         Generate body-to-inertial rotation matrix from quaternion.
#         Diebel, J. (2006). Representing attitude: Euler angles, unit quaternions, and rotation vectors (Vol. 58). https://doi.org/10.1093/jxb/erm298
#         """
#         q0 = self._q[0]
#         q1 = self._q[1]
#         q2 = self._q[2]
#         q3 = self._q[3]
#         return np.array(
#             [
#                 [
#                     q0**2 + q1**2 - q2**2 - q3**2,
#                     2 * q1 * q2 + 2 * q0 * q3,
#                     2 * q1 * q3 - 2 * q0 * q2,
#                 ],
#                 [
#                     2 * q1 * q2 - 2 * q0 * q3,
#                     q0**2 - q1**2 + q2**2 - q3**2,
#                     2 * q2 * q3 + 2 * q0 * q1,
#                 ],
#                 [
#                     2 * q1 * q3 + 2 * q0 * q2,
#                     2 * q2 * q3 - 2 * q0 * q1,
#                     q0**2 - q1**2 - q2**2 + q3**2,
#                 ],
#             ]
#         )

#     def R_bi(self):
#         return self.R_ib().T

#     def to_euler(self):

#         """Return EulerAngles() representation of rotation."""

#         return EulerAngles.from_quaternion(self)

#     def q_dot(self, omega: np.array):
#         """Returns a 4D numpy array representing the rate of the change of the quaternion."""
#         if isinstance(omega, Vector3):
#             omega = omega.to_array().reshape((3,))

#         omega = np.append(0, omega)  # Add a zero as the scalar part

#         return 0.5 * self.to_prodmat() @ omega  # q_dot = 0.5 * Q * [0, omega]^T

#     ###########################################################################
#     # Overrides and dunder methods

#     def __repr__(self):
#         return f"UnitQuaternion({self.__w}, [{self.__x}, {self.__y}, {self.__z}])"

#     def __eq__(self, other):
#         return np.all([a == b for a, b in zip(self, other)])

#     def __mul__(self, other):
#         is_unitquat = type(other) is UnitQuaternion
#         is_numpy = type(other) is np.array or type(other) is np.ndarray
#         is_vector3 = type(other) is Vector3
#         is_list = type(other) is list
#         is_scalar = type(other) in [float, int, np.float64]
#         if is_unitquat:  # Multiply with quaternion
#             return self._quatprod(q=other)
#         elif is_numpy and len(other) == 4:
#             return self._quatprod(q=UnitQuaternion(other[0], other[1:4]))
#         elif is_numpy or is_list:  # Multiply with vector
#             if len(other) > 3:
#                 raise ValueError(
#                     "UnitQuaternion multiplication must be done on 3D Vector or another UnitQuaternion."
#                 )
#             else:
#                 # Making sure shape of vector is (3,)
#                 if is_numpy:
#                     factor = other.reshape((3,))
#                 else:
#                     factor = other
#                 answer = self._quatrot(v=np.array(factor))
#                 if is_numpy:
#                     answer = answer.reshape(other.shape)
#                 return answer
#         elif is_vector3:
#             return build_vector3_from_array(self * other.to_array())
#         elif is_scalar:  # Multiply with scalar
#             raise UserWarning(
#                 "Multiplying a UnitQuaternion with a scalar does not change the quaternion."
#             )
#             q_scaled = self._q * other
#             return UnitQuaternion.from_wxyz(q_scaled)
#         else:
#             raise TypeError(
#                 f"Unsupported multiplication between {self.__class__.__name__} and {other.__class__.__name__}."
#             )

#     def __rmul__(self, other):
#         is_scalar = type(other) in [float, int, np.float64]
#         if is_scalar:  # Multiply with quaternion
#             return self * other
#         else:
#             raise TypeError(
#                 f"Unsupported multiplication between {self.__class__.__name__} and {other.__class__.__name__}."
#             )

#     def __add__(self, other):
#         if type(other) is UnitQuaternion:
#             return UnitQuaternion.from_wxyz(self._q + other._q)
#         else:
#             raise TypeError(
#                 f"Unsupported addition between {self.__class__.__name__} and {other.__class__.__name__}."
#             )

#     def __sub__(self, other):
#         if type(other) is UnitQuaternion:
#             return UnitQuaternion.from_wxyz(self._q - other._q)
#         else:
#             raise TypeError(
#                 f"Unsupported subtraction between {self.__class__.__name__} and {other.__class__.__name__}."
#             )

#     def __getitem__(self, key):
#         return self._q[key]

#     def __iter__(self):
#         return iter(self.to_array())

#     ###########################################################################
#     # Constructors

#     @staticmethod
#     def from_wxyz(*wxyz):
#         if len(wxyz) == 1:
#             wxyz = wxyz[0]
#         else:
#             raise ValueError("Argument must have length 1.")
#         return UnitQuaternion(w=wxyz[0], xyz=wxyz[1:4])

#     # Generate quaternion corresponding to a certain roll angle (yaw = pitch = 0)
#     @staticmethod
#     def _from_roll(roll: float, in_degrees=False):
#         """
#         INPUTS:
#             roll: roll angle
#         """
#         if in_degrees:
#             roll = np.deg2rad(roll)
#         return UnitQuaternion(
#             np.cos(roll / 2.0), np.array([np.sin(roll / 2.0), 0.0, 0.0])
#         )

#     # Generate quaternion corresponding to a certain pitch angle (roll = yaw = 0)
#     @staticmethod
#     def _from_pitch(pitch: float, in_degrees=False):
#         """
#         INPUTS:
#             pitch: pitch angle
#         """
#         if in_degrees:
#             pitch = np.deg2rad(pitch)
#         return UnitQuaternion(
#             np.cos(pitch / 2.0), np.array([0.0, np.sin(pitch / 2.0), 0.0])
#         )

#     # Generate quaternion corresponding to a certain yaw angle (roll = pitch = 0)
#     @staticmethod
#     def _from_yaw(yaw: float, in_degrees=False):
#         """
#         INPUTS:
#             yaw: yaw angle
#         """
#         if in_degrees:
#             yaw = np.deg2rad(yaw)
#         return UnitQuaternion(
#             np.cos(yaw / 2.0), np.array([0.0, 0.0, np.sin(yaw / 2.0)])
#         )

#     # Generate quaternion from EulerAngles
#     @staticmethod
#     def from_euler(arg):  # EulerAngle.to_vector()
#         if type(arg) is EulerAngles:
#             eul: EulerAngles = arg
#         elif type(arg) is np.array or type(arg) is list or type(arg) is tuple:
#             eul = EulerAngles.from_array(v=np.array(arg))
#         else:
#             raise ValueError(
#                 " Input must be of type EulerAngles or a 3D array or list."
#             )

#         return (
#             UnitQuaternion._from_yaw(eul.yaw)
#             * UnitQuaternion._from_pitch(eul.pitch)
#             * UnitQuaternion._from_roll(eul.roll)
#         )

#     @staticmethod
#     def from_rotmat(R: np.ndarray):
#         """
#         Generate Quaternion from a (4,4) numpy array-like rotation matrix.

#         The matrix should rotate from body-frame to inertial (NED) coordinates.
#         """

#         q_interm = pt3d_rot.quaternion_from_matrix(R)

#         return UnitQuaternion.from_wxyz(q_interm)

#     @staticmethod
#     def from_two_vectors(src: np.ndarray, dst: np.ndarray, eps: float = 1e-5):

#         dt = np.dot(src, dst)
#         cr = np.cross(src, dst)

#         src_norm = np.linalg.norm(src)
#         dst_norm = np.linalg.norm(dst)

#         # 180 degree rotation edge case:
#         if np.linalg.norm(cr) < eps and dt < 0:
#             cr = np.abs(src)
#             if cr[0] < cr[1]:
#                 if cr[0] < cr[2]:
#                     cr = np.array([1, 0, 0])
#                 else:
#                     cr = np.array([0, 0, 1])
#             else:
#                 if cr[1] < cr[2]:
#                     cr = np.array([0, 1, 0])
#                 else:
#                     cr = np.array([0, 1, 1])
#             qr = 0.0
#             cr = np.cross(src, cr)
#         else:
#             qr = dt + np.sqrt(src_norm * src_norm * dst_norm * dst_norm)

#         qi = cr[0]
#         qj = cr[1]
#         qk = cr[2]

#         return UnitQuaternion.from_wxyz([qr, qi, qj, qk])

#     @staticmethod
#     def from_axisangle_px4(aa: np.ndarray):
#         """In PX4 they make quaternions from a rotation vector, and rotate using its magnitude."""

#         angle = np.linalg.norm(aa)

#         if angle < 1e-8:
#             return UnitQuaternion()

#         q_w = np.cos(angle / 2)
#         q_ijk = aa / np.linalg.norm(aa) * np.sin(angle / 2)

#         return UnitQuaternion(w=q_w, xyz=q_ijk)


class Pose:
    orientation: UnitQuaternion
    cpp_pose_: cpp_Pose

    def __init__(self, position=np.array([0, 0, 0]), orientation=UnitQuaternion()):
        self.cpp_pose_ = cpp_Pose()
        self.cpp_pose_.position = position
        self.cpp_pose_.orientation = orientation

    @property
    def position(self) -> np.array:
        return self.cpp_pose_.position

    @position.setter
    def position(self, v: np.array):
        self.cpp_pose_.position = v

    @property
    def orientation(self) -> UnitQuaternion:
        return self.cpp_pose_.orientation

    @orientation.setter
    def orientation(self, q: UnitQuaternion):
        self.cpp_pose_.orientation = q.to_array()

    def __matmul__(self, other):
        """
        Transform a wrench by this pose.
        """
        if not isinstance(other, Wrench):
            raise TypeError(
                f"@ operation between {self.__class__.__name__} and"
                " {other.__class__.__name__} is not supported."
            )

        return self.cpp_pose_ @ other

    @property
    def T(self):
        """
        Define the inverse pose.
        """
        p_new = self.cpp_pose_.T
        return Pose(p_new.position, p_new.orientation)


def calc_poly_zero_crossing(poly_coeffs, xtol=0.01):
    def cost_function(x):
        return np.abs(np.polyval(poly_coeffs, x))

    x0 = 0
    options = {"xtol": xtol}

    optim_res = root(cost_function, x0, method="hybr", options=options)
    if not optim_res.success:
        raise RuntimeError("Could not find a root")

    return optim_res.x


def rps2radps(rps):
    return rps * (2 * np.pi)


def radps2rps(omega):
    return omega / (2 * np.pi)


def rpm2radps(rpm):
    return rps2radps(rpm / 60)


def radps2rpm(omega):
    return 60 * radps2rps(omega)


def sigmoid(x, a, width):
    """
    Evaluate a sigmoid function at point x.

    Its inflection point is at inflection point is at "a" and width defines its
    steepness.
    """
    sigmoid = (1 + np.exp(-width * (x - a)) + np.exp(width * (x + a))) / (
        (1 + np.exp(-width * (x - a))) * (1 + np.exp(width * (x + a)))
    )
    return sigmoid
