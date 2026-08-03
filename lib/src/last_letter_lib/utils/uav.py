#!/usr/bin/env python3
"""General uav library

This library is meant to host utilities related to UAV models.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Tue 18 Jan 2022"
__copyright__ = "Copyright 2022, George Zogopoulos"

import copy
from dataclasses import dataclass
from typing import List
from typing import Union

import numpy as np

from last_letter_lib.environment import EnvironmentData
from last_letter_lib.utils.math import Pose
from last_letter_lib.utils.math import UnitQuaternion
from last_letter_lib.utils.math import Vector3

from ..cpp_last_letter_lib.cpp_uav_utils import Airdata
from ..cpp_last_letter_lib.cpp_uav_utils import Inputs
from ..cpp_last_letter_lib.cpp_uav_utils import UavState


# class UavState:
#     position: Vector3 = None  # NED position
#     attitude: UnitQuaternion = None  # Attitude quaternion (from body to inertial)
#     velocity_linear: Vector3 = None  # Inertial velocities, body-frame
#     velocity_angular: Vector3 = None  # Body-frame angular velocity
#     thrusters_velocity: List[
#         float
#     ] = None  # Thruster rotational velocity (state), if applicable

#     def __init__(
#         self,
#         position=Vector3(),
#         attitude=UnitQuaternion(),
#         velocity_linear=Vector3(),
#         velocity_angular=Vector3(),
#         thrusters_velocity=[],
#     ):
#         self.position = position
#         self.attitude = attitude
#         self.velocity_linear = velocity_linear
#         self.velocity_angular = velocity_angular
#         self.thrusters_velocity = thrusters_velocity

#     def get_euler(self):
#         return self.attitude.to_euler()

#     def __repr__(self):
#         s = """
#         {}:
#         position={}
#         attitude={}
#         velocity_linear={}
#         velocity_angular={}
#         thrusters_velocity={}
#         """.format(
#             self.__class__.__name__,
#             repr(self.position),
#             repr(self.attitude.to_euler()),
#             repr(self.velocity_linear),
#             repr(self.velocity_angular),
#             self.thrusters_velocity,
#         )
#         return s

#     def __str__(self):
#         s = """
#         {}:
#         position={}
#         attitude={}
#         velocity_linear={}
#         OR magnitude/tilt/pan={}
#         velocity_angular={}
#         thrusters_velocity={}
#         """.format(
#             self.__class__.__name__,
#             str(self.position),
#             str(self.attitude.to_euler()),
#             str(self.velocity_linear),
#             str(Airdata.from_u(self.velocity_linear)),
#             str(self.velocity_angular),
#             self.thrusters_velocity,
#         )
#         return s

#     def to_array(self, shape=None):
#         # Convert object to a 13+N x 1 numpy array
#         output = np.zeros(
#             [
#                 13 + len(self.thrusters_velocity),
#             ]
#         )
#         if shape is None:
#             shape = output.shape

#         output[0:3] = self.position.to_array()
#         output[3:7] = self.attitude.to_array()
#         output[7:10] = self.velocity_linear.to_array()
#         output[10:13] = self.velocity_angular.to_array()
#         if len(self.thrusters_velocity) > 0:
#             output[13:] = np.array(self.thrusters_velocity)

#         return output.reshape(shape)

#     @classmethod
#     def from_array(cls, arr):
#         """
#         Crate a state object from a numpy array.

#         Assumes that the first 13 elements are the rigid body states and the
#         rest are motor velocities.
#         """
#         new_state = cls(
#             Vector3(arr[0], arr[1], arr[2]),
#             UnitQuaternion(arr[3], arr[4], arr[5], arr[6]),
#             Vector3(arr[7], arr[8], arr[9]),
#             Vector3(arr[10], arr[11], arr[12]),
#         )
#         if len(arr) > 13:
#             new_state.thrusters_velocity = arr[13:]

#         return new_state

#     @classmethod
#     def from_uavstate(cls, state):
#         return copy.deepcopy(state)

#     def strip_thrusters(self):
#         """
#         Return a copy of this UavState but without the thrusters.
#         """
#         return UavState(
#             self.position,
#             self.attitude,
#             self.velocity_linear,
#             self.velocity_angular,
#         )


# @dataclass
# class Airdata:
#     airspeed: float = 0  # Relative wind velocity
#     alpha: float = 0  # Angle of Attack
#     beta: float = 0  # Angle of sideslip

#     @classmethod
#     def from_u(
#         cls, v_b: Union[np.array, Vector3], v_w: Union[np.array, Vector3] = Vector3()
#     ):
#         """
#         Calculate the relative air data from inertial and wind speeds

#         INPUTS:
#             v_b: Inertial velocity, body-frame
#             v_w: Wind (air-mass) velocity, body-frame

#         OUTPUTS:
#             airspeed: The norm of the relative wind
#             alpha: The angle of attack
#             beta: The angle of sideslip
#         """

#         is_v_w_vector3 = type(v_w) in [Vector3]
#         is_v_b_vector3 = type(v_b) in [Vector3]
#         if is_v_w_vector3:
#             v_w = v_w.to_array()

#         if is_v_b_vector3:
#             v_b = v_b.to_array()

#         v_r = v_b - v_w
#         u = v_r[0]
#         v = v_r[1]
#         w = v_r[2]

#         airspeed = np.linalg.norm(v_r)
#         alpha = np.arctan2(w, np.abs(u))
#         if u == 0:
#             if v == 0:
#                 beta = 0
#             else:
#                 beta = np.arcsin(v / np.abs(v))
#         else:
#             beta = np.arctan2(v, u)

#         return cls(airspeed, alpha, beta)

#     @classmethod
#     def from_state_environment(cls, state: UavState, environment: EnvironmentData):
#         """
#         Calculate airdata given the UAV state and the environment data.
#         """
#         u_b = state.velocity_linear
#         R_ib = state.attitude.R_ib()
#         u_w = R_ib @ environment.wind.to_array()
#         return cls.from_u(u_b, u_w)

#     @property
#     def S_bw(self):
#         # Create rotation matrix S, which transforms from body frame to wind frame
#         # Taken from Aircraft Control and Simulation, Stevens Lewis, p.63
#         sa = np.sin(self.alpha)
#         ca = np.cos(self.alpha)
#         sb = np.sin(self.beta)
#         cb = np.cos(self.beta)

#         S = np.array([[ca * cb, sb, sa * cb], [-ca * sb, cb, -sa * sb], [-sa, 0, ca]])
#         return S

#     @property
#     def S_wb(self):
#         # Create rotation matrix S^T, which transforms from wind frame to body frame
#         return self.S_bw.T

#     def to_u(self) -> Vector3:
#         # Convert airdata into body-frame velocities
#         Va = self.airspeed
#         alpha = self.alpha
#         beta = self.beta

#         u = Va * np.cos(alpha) * np.cos(beta)
#         v = Va * np.sin(beta)
#         w = Va * np.sin(alpha) * np.cos(beta)

#         return Vector3(u, v, w)

#     def to_array(self) -> np.array:
#         # Convert to numpy array
#         return np.array([self.airspeed, self.alpha, self.beta])

#     def __str__(self):
#         return f"Airdata(Va={self.airspeed}, AoA[deg]={np.rad2deg(self.alpha)}, AoS[deg]={np.rad2deg(self.beta)})"


def calc_airdata_at_link(uav_airdata: Airdata, vel_ang_b: Vector3, pose_link: Pose):
    """
    Transform the airdata triplet from the body frame to the link frame
    INPUTS:
        uav_airdata: The airdata at the body frame
        vel_ang_b: The body frame angular velocity
        pose_link: The pose of the link
    """
    body_wind = uav_airdata.to_u()
    link_wind = pose_link.orientation.conjugate() * body_wind + np.cross(
        vel_ang_b.to_array(), pose_link.position
    )
    # Clean up inconsistent .to_array() calls here.
    res = Airdata()
    res.init_from_velocity(link_wind)
    return res


def calc_psi_dot_from_trajectory(trajectory: np.array):
    """
    INPUT:
        list with [airspeed, gamma, R]
    """
    return (
        trajectory[0] * np.cos(trajectory[1]) / trajectory[2]
    )  # dot_psi = V * cos(gamma) / R


def calc_radius_from_psi_dot(V, gamma, psi_dot):
    if psi_dot == 0:
        return np.inf
    else:
        return V * np.cos(gamma) / psi_dot
