#!/usr/bin/env python3
"""Module defining vehicles and simulation.

usage:
    Import the module for use in other scripts.
"""

__authors__ = ["George Zogopoulos", "Peter Seres"]
__credits__ = []
__date__ = "Fri 21 Jan 2022"
__copyright__ = "Copyright 2022, George Zogopoulos"


from dataclasses import dataclass
from typing import List
from typing import Optional
from typing import Union

import numpy as np
import yaml
from pydantic import BaseModel

import last_letter_lib.aerodynamics as aero
import last_letter_lib.propulsion as prop
import last_letter_lib.systems as llsys
from last_letter_lib.environment import EnvironmentModel
from last_letter_lib.gravity import GravityClassic
from last_letter_lib.gravity import GravitySimple
from last_letter_lib.systems import RigidBody6DOF
from last_letter_lib.utils.math import EulerAngles
from last_letter_lib.utils.math import Inertial
from last_letter_lib.utils.math import Pose
from last_letter_lib.utils.math import UnitQuaternion
from last_letter_lib.utils.math import Vector3
from last_letter_lib.utils.math import Wrench
from last_letter_lib.utils.math import build_vector3_from_array
from last_letter_lib.utils.programming import _traverse_collect_pydantic
from last_letter_lib.utils.uav import Airdata
from last_letter_lib.utils.uav import UavState

from .cpp_last_letter_lib.cpp_simulation import Aircraft

# class AircraftParameters(BaseModel):
#     """
#     The overall aircraft specification.
#
#     This class is the top-level accessor to the complete specification of the
#     aircraft.
#     """
#
#     name: str
#     """An identifier name for this aircraft."""
#     aerodynamics: List[Union[aero.AerodynamicParameters, str]]
#     """A list containing Aerodynamic components or names of existing Aerodynamic definitions."""
#     thrusters: List[
#         Union[
#             prop.ThrusterElectricParameters,
#             prop.ThrusterSpeedControlledParameters,
#             prop.ThrusterBeardParameters,
#             prop.ThrusterSimpleParameters,
#         ]
#     ]
#     """A list containing Thruster components or names of existing Thruster definitions."""
#     batteries: Optional[List[prop.BatteryParameters]] = list()
#     """A list containing Battery components."""
#     other_components: Optional[List[llsys.ComponentParameters]] = list()
#     """A list containing miscellaneous components. Mainly used for specifying additional masses on the airframe."""
#
#     ## These are implemented by the Aircraft class.
#
#     @property
#     def components(self) -> List[llsys.ComponentParameters]:
#         """
#         Return a list with all the Aircraft Components.
#         """
#         components = self.aerodynamics + self.thrusters
#         if self.batteries:
#             components += self.batteries
#         if self.other_components:
#             components += self.other_components
#         return components
#
#     def get_num_thrusters(self, thruster_use: Optional[prop.ThrusterUseEnum]) -> float:
#         """
#         Get the number of thrusters of a particular purpose.
#
#         :param thruster_use: The type of thruster the query refers to. If None, then all the thrusters will be counted.
#         """
#         if thruster_use is None:
#             return len(self.thrusters)
#         else:
#             return len([t for t in self.thrusters if t.usage == thruster_use])
#
#
# def build_aircraft_components(params: AircraftParameters):
#     component_list = []
#     for c in params.components:
#         if isinstance(c, aero.AerodynamicParameters):
#             component_list.append(aero.Aerodynamic(c))
#         elif isinstance(c, prop.ThrusterParameters):
#             component_list.append(prop.build_thruster(c))
#         elif isinstance(c, prop.BatteryParameters):
#             component_list.append(prop.Battery(c))
#         elif isinstance(c, llsys.ComponentParameters):
#             component_list.append(llsys.Component(c.name))
#             yaml_desc = yaml.load(c.json(), Loader=yaml.SafeLoader)
#             component_list[-1].initialize(yaml.dump(yaml_desc))
#         else:
#             raise TypeError(f"Unsupported component type {c.__class__.__name__}.")
#     return component_list


# class Aircraft:
#     """
#     Aircraft suitable for complete simulation.
#     """
#
#     params: AircraftParameters
#     rigid_body: RigidBody6DOF
#     components: list
#     environment: EnvironmentModel
#     gravity: Union[GravitySimple, GravityClassic]
#     wrench_aero: List[Wrench]
#     wrench_propulsion: List[Wrench]
#
#     _com: Pose  # Center of mass
#
#     def __init__(self, params: AircraftParameters):
#         # Store parameters
#         self.params = params
#         # Initialize all sub-components
#         self.components = build_aircraft_components(params)
#         # Calculate the Center of Mass
#         self._com_pose = self.calc_com()
#         # Collect all components and sum up inertia
#         inertia = self.calc_components_inertia(params.components)
#         # Initialize internal, lumped rigid body dynamics
#         self.rigid_body = RigidBody6DOF(mass=self.calc_mass(), inertia_matrix=inertia)
#         # Instantiate environment model
#         self.environment = EnvironmentModel()
#         yaml_desc = yaml.load(params.json(), Loader=yaml.SafeLoader)
#         self.environment.initialize(yaml.dump(yaml_desc))
#         # Instantiate gravity model
#         self.gravity = GravitySimple()
#
#     def calc_mass(self) -> float:
#         """
#         Return the total mass of the aircraft as a sum of its component mases.
#         """
#         components = _traverse_collect_pydantic(self.params, llsys.ComponentParameters)
#         mass = 0
#         for c in components:
#             if c.inertial:
#                 mass += c.inertial.mass
#
#         return mass
#
#     def calc_com(self) -> Pose:  # Tuple[float, float, float]:
#         """
#         Calculate the center of mass of the aircraft, in respect to the aircraft frame.
#         """
#         com = np.zeros(
#             3,
#         )
#         total_mass = self.calc_mass()
#
#         components = _traverse_collect_pydantic(self.params, llsys.ComponentParameters)
#         for c in components:
#             if c.inertial:
#                 com += np.array(
#                     [
#                         coord * (c.inertial.mass / total_mass)
#                         for coord in c.pose.position
#                     ]
#                 )
#
#         return Pose(com, UnitQuaternion())
#
#     def calc_components_inertia(self, components: list) -> np.array:
#         """
#         Return the inertia tesnor of the aircraft as a combination of its components
#         """
#         total_inertia = np.zeros((3, 3))
#         for c in components:
#             if c.inertial:
#                 # Read the component position in the body frame and correct with the CoG position.
#                 # This is because we want the inertia wrt CoG, not the aircraft frame.
#                 pos = c.pose.position - self.com
#                 # Read the component orientation in the body frame
#                 R_cb = EulerAngles(*c.pose.orientation).R_bi()
#
#                 # Calculate inertia due to translation
#                 inertia_t = np.zeros((3, 3))
#                 # Collect the coordinate combinations
#                 inertia_t[0, 0] = pos[1] ** 2 + pos[2] ** 2
#                 inertia_t[0, 1] = inertia_t[1, 0] = -(pos[0] * pos[1])
#                 inertia_t[0, 2] = inertia_t[2, 0] = -(pos[0] * pos[2])
#                 inertia_t[1, 1] = pos[0] ** 2 + pos[2] ** 2
#                 inertia_t[1, 2] = inertia_t[2, 1] = -(pos[1] * pos[2])
#                 inertia_t[2, 2] = pos[0] ** 2 + pos[1] ** 2
#                 # Multiply with the point mass
#                 inertia_t *= c.inertial.mass
#
#                 # Calculate inertia due to rotation
#                 if c.inertial.inertia:
#                     inertia_0 = np.array(c.inertial.inertia).reshape((3, 3))
#                     inertia_r = R_cb @ inertia_0 @ R_cb.T
#                 else:
#                     inertia_r = np.zeros((3, 3))
#
#                 total_inertia += inertia_t + inertia_r
#
#         return total_inertia
#
#     def _get_components_aerodynamic(self, components: list) -> list:
#         aero_list = []
#         for c in self.components:
#             if isinstance(c, aero.Aerodynamic):
#                 aero_list.append(c)
#         return aero_list
#
#     def _get_components_thrusters(self, components: list) -> list:
#         thruster_list = []
#         for c in self.components:
#             if isinstance(c, prop.Thruster):
#                 thruster_list.append(c)
#         return thruster_list
#
#     @property
#     def thrusters(self):
#         return self._get_components_thrusters(self.components)
#
#     @property
#     def airfoils(self):
#         return self._get_components_aerodynamic(self.components)
#
#     @property
#     def mass(self):
#         return self.rigid_body.mass
#         # TODO: Verify this mass is equal to the sum of masses of all other components
#
#     @property
#     def com(self):
#         """
#         Obtain the center of mass in respect to the aircraft frame
#         """
#         return self._com_pose.position
#
#     @property
#     def state(self):
#         return UavState(
#             position=self.rigid_body.position.to_array(),
#             orientation=self.rigid_body.orientation,
#             velocity_linear=self.rigid_body.velocity_linear.to_array(),
#             velocity_angular=self.rigid_body.velocity_angular.to_array(),
#             thrusters_velocity=[t.velocity for t in self.thrusters],
#         )
#
#     def reset(self, state: UavState):
#         """
#         Initialize the aircraft with the provided UavState.
#         """
#         self.rigid_body.state = state.strip_thrusters().to_array()
#         if len(state.thrusters_velocity) != len(self.thrusters):
#             raise ValueError("Incompatible thruster state provided.")
#         for idx, t in enumerate(self.thrusters):
#             t.velocity = state.thrusters_velocity[idx]
#
#     @property
#     def acceleration_linear(self) -> Vector3:
#         """Returns the linear acceleration in body FRD frame."""
#         return self.rigid_body.acceleration_linear
#
#     @property
#     def acceleration_angular(self) -> Vector3:
#         """Returns the angular acceleration of the body FRD frame."""
#         return self.rigid_body.acceleration_angular
#
#     @property
#     def airdata_derivatives(self) -> Vector3:
#         """
#         Returns the derivatives of the airdata triplet.
#
#         According to Stevens&Lewis p.82
#         """
#         airdata = Airdata.from_state_environment(self.state, self.environment.data)
#         Va = airdata.airspeed
#         v = self.state.velocity_linear
#         acc = self.acceleration_linear
#         if Va != 0 and v.norm != 0:
#             Va_dot = (v.x * acc.x + v.y * acc.y + v.z * acc.z) / Va
#             beta_dot = (acc.y * Va - v.y * Va_dot) / (Va * Va * np.cos(airdata.beta))
#             alpha_dot = (v.x * acc.z - v.z * acc.x) / (v.x * v.x + v.z * v.z)
#         else:
#             Va_dot = acc.x + acc.y + acc.z
#             beta_dot = 0  # Don't increase beta for now
#             alpha_dot = 0  # Don't increase alpha for now
#
#         return Vector3(Va_dot, alpha_dot, beta_dot)
#
#     def step(self, inputs, dt):
#         """
#         INPUTS:
#             inputs: A uav.Inputs object. Thruster list order is the same as in its description
#         """
#         # Update environment
#         self.environment.calc_environment(self.state)
#         state = self.state  # Capture the state before stepping components
#
#         # Step the thruster models
#         for idx, thruster in enumerate(self.thrusters):
#             # Pass the correct model input, according to its type
#             if isinstance(thruster, prop.ThrusterSimple):
#                 thruster.rk4([inputs.delta_t[idx]], dt)
#             elif isinstance(thruster, prop.ThrusterBeard):
#                 airdata = Airdata.from_state_environment(
#                     self.state, self.environment.data
#                 )
#                 thruster.rk4(
#                     [inputs.delta_t[idx], airdata.airspeed, self.environment.data.rho],
#                     dt,
#                 )
#             elif isinstance(thruster, prop.ThrusterElectric):
#                 raise NotADirectoryError("Electric thrusters not supported yet.")
#             elif isinstance(thruster, prop.ThrusterSpeedControlled):
#                 airdata = Airdata.from_state_environment(
#                     self.state, self.environment.data
#                 )
#                 propeller_speed = inputs.delta_t[idx] * thruster.velocity_max
#                 thruster.rk4(
#                     [propeller_speed, airdata.airspeed, self.environment.data.rho],
#                     dt,
#                 )
#             else:
#                 raise TypeError(f"Unknown thruster type {thruster.__class__}.")
#
#         total_wrench = Wrench(np.zeros((3, 1)), np.zeros((3, 1)))
#
#         # Collect thruster wrenches and convert to aircraft frame
#         prop_wrenches = [t.pose @ t.wrench for t in self.thrusters]
#         total_wrench_aircraft = sum(
#             prop_wrenches, Wrench(np.zeros((3, 1)), np.zeros((3, 1)))
#         )
#         # Then convert from aircraft frame to body frame
#         self.wrench_propulsion = self._com_pose.T @ total_wrench_aircraft
#         total_wrench += self.wrench_propulsion
#
#         # Collect aerodynamic wrenches and convert to aircraft frame
#         aero_wrenches = [
#             a.get_wrench(state, self.environment.data, inputs) for a in self.airfoils
#         ]
#         total_wrench_aircraft = sum(
#             aero_wrenches, Wrench(np.zeros((3, 1)), np.zeros((3, 1)))
#         )
#         # Then convert from aircraft frame to body frame
#         self.wrench_aero = self._com_pose.T @ total_wrench_aircraft
#         total_wrench += self.wrench_aero
#
#         # Calculate gravity forces
#         inertial = Inertial()
#         inertial.mass = self.rigid_body.mass
#         self.gravity.calc_gravity(self.state, inertial)
#         # self.gravity
#         # gravity_force_i = (
#         #     self.rigid_body.mass * self.gravity.g(self.state.position).to_array()
#         # )
#         # gravity_force_b = self.rigid_body.orientation.R_ib() @ gravity_force_i
#         total_wrench += Wrench(
#             self.gravity.get_force(self.state, inertial),
#             self.gravity.get_torque(self.state, inertial),
#         )
#
#         # Step 6DOF model
#         self.rigid_body.rk4(total_wrench.to_array(), dt)


# class DynamicSystemCollection(ABC):
#     pass


# class SimulationEndCondition:
#     pass


# class Simulation:
#     pass
