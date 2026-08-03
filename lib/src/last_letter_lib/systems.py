#!/usr/bin/env python3
"""Module defining static and dynamic sytems.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos", "Peter Seres"]
__credits__ = []
__date__ = "Thu 20 Jan 2022"
__copyright__ = "Copyright 2022, George Zogopoulos"

import os
from abc import ABC
from abc import abstractmethod
from dataclasses import dataclass
from dataclasses import field
from typing import Optional
from typing import Tuple

import numpy as np
from pydantic import BaseModel
from pydantic import FilePath
from pydantic import field_validator

import last_letter_lib.utils.math as llmath
from last_letter_lib.utils.math import UnitQuaternion
from last_letter_lib.utils.math import Vector3
from last_letter_lib.utils.math import build_vector3_from_array

from .cpp_last_letter_lib.cpp_systems import Component


class PoseParameters(BaseModel):
    """
    A pose, dictating a rotation and translation *from* a parent frame *to* a child frame.
    """

    position: Tuple[float, float, float] = (0, 0, 0)
    """ Vector pointing from parent reference point to child reference point """
    orientation: Tuple[float, float, float] = (0, 0, 0)
    """Roll, Pitch, Yaw angles rotating the parent frame into the child frame, in that order, radians"""

    def flatten(self) -> dict:
        """
        Convert the Pose to a 1-level deep dictionary.

        Typically useful for generating Pandas DataFrames.
        """

        return {
            "pos_x": self.position[0],
            "pos_y": self.position[1],
            "pos_z": self.position[2],
            "yaw": self.orientation[0],
            "pitch": self.orientation[1],
            "roll": self.orientation[2],
        }


class InertialParameters(BaseModel):
    """
    The inertial characteristics of a component, comprised of mass and matrix of
    inertia.

    If an inertia matrix is not specified, then it will not be taken into account
    for overall inertia calculations.
    """

    mass: float
    """The mass of the component"""
    inertia: Optional[
        Tuple[float, float, float, float, float, float, float, float, float]
    ] = None
    """ Matrix of inertia, specified row-wise."""

    def flatten(self) -> dict:
        """
        Convert the Inertial data to a 1-level deep dictionary.

        Typically useful for generating Pandas DataFrames.
        """
        data = {
            "mass": self.mass,
        }

        if self.inertia:
            data.update(
                {
                    "ixx": self.inertia[0],
                    "iyy": self.inertia[4],
                    "izz": self.inertia[8],
                    "ixz": self.inertia[2],
                }
            )

        return data


class MeshParameters(BaseModel):
    """
    The appearance of a :class:`Component` in the form of a triangular mesh.
    """

    filepath: str
    """
    The filepath where the mesh file can be found. Can be absolute or relative
    to the *last_letter_lib* root directory.
    """
    pose: PoseParameters  # Pose from the component frame to the mesh frame
    """
    The pose of the mesh in relation to the component frame. This is useful in
    case the mesh is defined with a different origin.
    """
    scale: Optional[Tuple[float, float, float]] = tuple([1, 1, 1])
    """
    The scale factor to apply to the mesh on each axis, to convert its coordinates
    into SI units.
    """

    @field_validator("filepath")
    def file_must_exist(cls, filepath):
        filepath = os.path.expanduser(filepath)
        if os.path.isabs(filepath):
            if os.path.exists(filepath):
                return filepath
            else:
                raise ValueError(f"Mesh file {filepath} does not exist.")
        else:
            raise NotImplementedError("Relative paths are not supported yet.")
            # fullpath = os.path.join(LLL_PATH, filepath)
            # if os.path.exists(fullpath):
            #     return filepath
            # else:
            #     raise ValueError(f"Mesh file {filepath} does not exist.")


class ComponentParameters(BaseModel, extra="forbid"):
    """
    A component description.
    """

    name: str
    """An identifier name for this component."""
    pose: PoseParameters
    """The pose of this component relative to the aircraft frame."""
    inertial: Optional[InertialParameters] = None
    """The inertial characteristics of this component."""
    mesh: Optional[MeshParameters] = None
    """The appearance of this component."""


# class Component:
#     """
#     A component description.
#     """

#     def __init__(self, parameters: ComponentParameters):
#         self.name = parameters.name
#         self.pose = llmath.Pose(
#             Vector3(*parameters.pose.position),
#             UnitQuaternion.from_euler(parameters.pose.orientation),
#         )
#         if parameters.inertial:
#             self.inertial = llmath.Inertial(
#                 parameters.inertial.mass, np.array(parameters.inertial.inertia)
#             )
#         else:
#             self.inertial = None


class DynamicSystem(ABC):
    """Dynamic System class to simulate nonlinear systems. Uses rk4 to increment the system.
    Child classes must implement _dynamics function for the nonlinear system: x_dot = f(x, u, t)
    """

    def __init__(self, x_init: np.array, u_init: np.array):
        # States:
        self.x = np.array(x_init)
        self.x_init = self.x

        # State perturbations:
        self.x_dot = np.zeros([len(self.x)])

        # Input storage
        self.u = np.array(u_init)

        # Time management
        self.__t = 0.0

    @property
    def t(self):
        return self.__t

    @t.setter
    def t(self, t):
        self.__t = t

    @property
    def y(self):
        """
        Get the system output.
        """
        return self._outputs(self.x, self.u, self.t)

    @abstractmethod
    def _dynamics(
        self, x: np.ndarray, u: np.ndarray = None, t: float = None
    ) -> np.ndarray:
        """
        x_dot = f(x, u, t)
        Make sure all states and inputs are in array format. Even it it's only 1 input.
        """

        raise NotImplementedError

    @abstractmethod
    def _outputs(
        self, x: np.ndarray, u: np.ndarray = None, t: float = None
    ) -> np.ndarray:
        """
        y = h(x, u, t)
        """

        raise NotImplementedError

    def rk4(self, u_i, dt):
        assert dt > 0
        if dt < 1e-5:
            print(f"Warning: Too small timestep: {dt}")

        # Make sure input u_i is a numpy array:
        if type(u_i) is float:
            u_i = np.array([u_i])
        elif type(u_i) is list:
            u_i = np.array(u_i)

        self.u = u_i

        x_i = self.x
        t_i = self.t

        d1 = dt * self._dynamics(x_i, u_i, t_i)
        d2 = dt * self._dynamics(x_i + d1 / 2, u_i, t_i + dt / 2)
        d3 = dt * self._dynamics(x_i + d2 / 2, u_i, t_i + dt / 2)
        d4 = dt * self._dynamics(x_i + d3, u_i, t_i + dt)

        increment = (d1 + 2 * d2 + 2 * d3 + d4) / 6

        self.x = x_i + increment
        self.x_dot = increment / dt
        self.t = self.t + dt

        self.post_propagation()

        return self.x, self.t

    def post_propagation(self):
        """
        Placeholder function to run post propagation
        """
        pass

    def reset(self, x_init=None):
        if x_init is not None:
            self.x_init = x_init
        self.x = self.x_init
        self.t = 0


class RigidBody6DOF(DynamicSystem):
    """
    Drone with 6 degrees of freedom: Body coordinate system: X forward Y right Z down

        States x :
            0-1-2: position inertial (N E D)
            3-4-5-6: unit quaternion (w x y z)
            7-8-9: velocity body (F R D)
            10-11-12: angular rates (p q r)

        Inputs u : (wrench)
            0-1-2: body forces (X, Y, Z)
            4-5-6: body moments (L, M, N)

        Parameters:
            body mass
            inertia matrix

    """

    position: Vector3 = None
    orientation: UnitQuaternion = None
    velocity_linear: Vector3 = None
    velocity_angular: Vector3 = None

    def __init__(self, x_init=None, u_init=None, mass=10.0, inertia_matrix=None):
        # vehicle mass                                      [kg]
        self.mass = mass

        if inertia_matrix is None:
            self.I_v = np.eye(3)
        else:
            # vehicle inertia matrix                        [kgm2]
            self.I_v = inertia_matrix

        # Calculate inverse once only.
        self.I_v_inverse = np.linalg.inv(self.I_v)

        # Set initial conditions
        if x_init is None:
            x_init = np.zeros(shape=[13])
            x_init[3:7] = UnitQuaternion().to_array()

        if u_init is None:
            u_init = np.zeros(6)

        # Instantiate DynamicSystem
        super().__init__(x_init=x_init, u_init=u_init)

        self.position = Vector3()
        self.orientation = UnitQuaternion()
        self.velocity_linear = Vector3()
        self.velocity_angular = Vector3()
        self._update_attributes()

    @property
    def state(self) -> np.array:
        return self.x

    @state.setter
    def state(self, state: np.array):
        self.x = state
        self._update_attributes()

    @property
    def acceleration_linear(self) -> Vector3:
        """Returns the acceleration in body frame."""

        return build_vector3_from_array(self.x_dot[7:10])

    @property
    def acceleration_angular(self) -> Vector3:
        """Returns the angular acceleration of the body FRD frame."""

        return build_vector3_from_array(self.x_dot[10:13])

    def post_propagation(self):
        self._update_attributes()

    def _update_attributes(self):
        """
        Update the ease-of-use attributes.
        """
        self.position.x = self.x[0]
        self.position.y = self.x[1]
        self.position.z = self.x[2]
        self.orientation.w = self.x[3]
        self.orientation.x = self.x[4]
        self.orientation.y = self.x[5]
        self.orientation.z = self.x[6]
        self.orientation.normalize()
        self.velocity_linear.x = self.x[7]
        self.velocity_linear.y = self.x[8]
        self.velocity_linear.z = self.x[9]
        self.velocity_angular.x = self.x[10]
        self.velocity_angular.y = self.x[11]
        self.velocity_angular.z = self.x[12]

    def __repr__(self):
        return f" Rigid Body object with state: {self.x}"

    def _dynamics(self, x: np.ndarray, u: np.ndarray, t: float) -> np.ndarray:
        # Initialize state perturbation:
        x_dot = np.zeros(shape=[13])

        # Get orientation
        q = self.orientation

        # Inputs:
        self.forces = u[0:3]  # Forces
        self.moments = u[3:6]  # Moments

        # Linear kinematics:
        p_dot = q.R_bi() @ self.velocity_linear.to_array()

        # Linear dynamics:
        omega = self.velocity_angular.to_array()
        v_dot = (
            -np.cross(omega, self.velocity_linear.to_array()) + self.forces / self.mass
        )

        # Rotational kinematics:
        q_dot = q.q_dot(omega)

        # Rotational dynamics:
        omega_dot = self.I_v_inverse @ (
            self.moments - np.cross(omega, self.I_v @ omega)
        )

        # Assign to state perturbation vector:
        x_dot[0:3] = p_dot
        x_dot[3:7] = q_dot
        x_dot[7:10] = v_dot
        x_dot[10:13] = omega_dot

        self._update_attributes()

        return x_dot

    def _outputs(self, x: np.ndarray, u: np.ndarray, t: float) -> np.ndarray:
        return x


# class DynamicSystemCollection(ABC):
#     pass


# class Aircraft(DynamicSystemCollection):
#     pass


# class SimulationEndCondition:
#     pass


# class Simulation:
#     pass
