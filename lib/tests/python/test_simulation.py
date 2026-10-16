#!/usr/bin/env python3
"""Tests for the simulation library.

usage:
    Have pytest run the tests in this file.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Fri 21 Jan 2022"
__copyright__ = "Copyright 2022, George Zogopoulos"

from pathlib import Path

import numpy as np
import pytest

from last_letter import aerodynamics as aero
from last_letter import propulsion
from last_letter import simulation as sim
from last_letter import systems
from last_letter.utils.math import EulerAngles
from last_letter.utils.math import Vector3
from last_letter.utils.math import build_vector3_from_array
from last_letter.utils.uav import Inputs
from last_letter.utils.uav import UavState

DELTAT = 0.01

###############################################################################
# Declare fixtures


@pytest.fixture
def build_desc_aircraft():
    return f"""
    world:
        deltaT: {DELTAT}
        integratorType: 0
    dynamics:
        aero:
            nWings: 1
            airfoil1:
                name: main_wing
                aerodynamicsType: 3
                chanAileron: 0
                chanElevator: 1
                chanRudder: 3
                c_L_0: 0.4
                c_L_alpha: 6.5
                c_D_alpha: 0.14
                c_m_0: 0.01
                c_m_alpha: -1.3
                c_m_qn: -30
                inertial:
                    mass: 1.8
                    tensor:
                        j_xx: 0.8244
                        j_yy: 1.135
                        j_zz: 1.759
                        j_xz: 0.1204
        prop:
            nMotors: 1
            motor1:
                name: pusher
                motorType: 6
                chanMotor: 2
                thrustMax: 10
                torqueMax: 0.01
                pose:
                    position:
                        x: 0.03
                inertial:
                    mass: 0.2
        ground:
            groundReactionType: 0
            name: no_contact
    env:
        windRef: 0
    kinematics:
    init:
        position: [0, 0, 0]
    """
    return sim.AircraftParameters(
        name="test_aircraft",
        aerodynamics=[
            aero.AerodynamicParameters(
                name="test_airfoil",
                pose=systems.PoseParameters(position=(0, 0, 0), orientation=(0, 0, 0)),
                inertial=systems.InertialParameters(
                    mass=2, inertia=[0.8244, 0, -0.12, 0, 1.135, 0, -0.12, 0, 1.759]
                ),
                s=0.45,
                b=1.88,
                c=0.24,
                c_L_0=0.4,
                c_L_alpha=[6.5, 0],
                c_D_0=0.09,
                c_D_alpha=[0.14, 0],
                c_Y_beta=[0.98, 0],
                c_l_0=0,
                c_l_pn=[-1.0, 0],
                c_l_beta=[-0.12, 0],
                c_l_deltaa=[0.25, 0],
                c_m_0=0.01,
                c_m_alpha=[-1.3, 0],
                c_m_qn=[-30, 0],
                c_m_deltae=[1.0, 0],
                c_n_0=0,
                c_n_beta=[0.25, 0],
                c_n_rn=[-1.0, 0],
                c_n_deltar=[0.1, 0],
                delta_a_max=0.3491,
                delta_e_max=0.3491,
                delta_r_max=0.3491,
            ),
        ],
        thrusters=[
            propulsion.ThrusterSimpleParameters(
                name="thruster_simple",
                pose=systems.PoseParameters(
                    position=(-0.5, 0, 0), orientation=(0, 0, 0)
                ),
                inertial=systems.InertialParameters(mass=0.1),
                rotation_dir=propulsion.DirectionEnum.CW,
                thruster_type=propulsion.ThrusterTypeEnum.SIMPLE,
                usage=propulsion.ThrusterUseEnum.AIRPLANE,
                thrust_min=0,
                thrust_max=10,
                torque_min=0,
                torque_max=0.01,
            )
        ],
        other_components=[
            systems.ComponentParameters(
                name="ballast",
                pose=systems.PoseParameters(
                    position=(0.5, 0, 0), orientation=(0, 0, 0)
                ),
                inertial=systems.InertialParameters(mass=0.1),
            )
        ],
    )


# @pytest.fixture
# def build_desc_aircraft_2():
#     return sim.AircraftParameters(
#         name="test_aircraft_2",
#         aerodynamics=[
#             aero.AerodynamicParameters(
#                 name="test_airfoil", pose=systems.PoseParameters()
#             )
#         ],
#         thrusters=[
#             propulsion.ThrusterSimpleParameters(
#                 name="thruster_simple",
#                 pose=systems.PoseParameters(),
#                 usage=propulsion.ThrusterUseEnum.MULTICOPTER,
#             ),
#             propulsion.ThrusterBeardParameters(
#                 name="thruster_beard",
#                 pose=systems.PoseParameters(),
#                 usage=propulsion.ThrusterUseEnum.AIRPLANE,
#             ),
#         ],
#         other_components=[
#             systems.ComponentParameters(
#                 name="ballast",
#                 pose=systems.PoseParameters(
#                     position=(0.5, 0, 0), orientation=(0, 0, 0)
#                 ),
#                 inertial=systems.InertialParameters(mass=0.1),
#             )
#         ],
#     )


###############################################################################
# Declare the tests themselves ################################################


# class TestAircraftParameters:
#     """
#     Test the Aircraft class.
#     build_aircraft uses the following components:
#     - build_aerodynamic, mass 1kg, at pos (0.0,0.0,0.0)
#     - build_thruster_simple, mass 1kg, at pos (0.0,0.0,0.0)
#     - build_thruster_beard, mass 1kg, at pos (1.0,1.0,1.0)
#     - build_battery, mass 0kg, at pos (0.0,0.0,0.0)
#     """
#
#     def test_constructor(self, build_desc_aircraft):
#         test_ac = build_desc_aircraft
#         assert isinstance(test_ac, sim.AircraftParameters)
#
#     def test_components(self, build_desc_aircraft):
#         """
#         - Test that correct number of components is returned
#         - Test that the types of the components are correct
#         """
#         test_ac = build_desc_aircraft
#         test_ac_components = test_ac.components
#         component_types = [type(component) for component in test_ac_components]
#         assert len(test_ac_components) == 3
#         assert component_types == [
#             aero.AerodynamicParameters,
#             propulsion.ThrusterSimpleParameters,
#             systems.ComponentParameters,
#         ]
#
#     def test_get_num_thrusters(self, build_desc_aircraft_2):
#         """
#         Test that correct number of thrusters is returned.
#         - no query: all thrusters are counted.
#         - query: only thrusters of which the use is specified by the query.
#         """
#         test_ac = build_desc_aircraft_2
#         num_thrusters_all = test_ac.get_num_thrusters(thruster_use=None)
#         # is "thruster_use=None" supposed to be explicitly stated? Using no argument fails
#         num_thrusters_multicopter = test_ac.get_num_thrusters(
#             thruster_use=propulsion.ThrusterUseEnum.MULTICOPTER
#         )
#         num_thrusters_airplane = test_ac.get_num_thrusters(
#             thruster_use=propulsion.ThrusterUseEnum.AIRPLANE
#         )
#         tests = [
#             num_thrusters_all == 2,
#             num_thrusters_multicopter == 1,
#             num_thrusters_airplane == 1,
#         ]
#         assert np.all(tests)


class TestAircraft:
    def test_constructor(self, build_desc_aircraft):
        ac = sim.Aircraft("test_aircraft")
        ac.initialize(build_desc_aircraft)
        assert isinstance(ac, sim.Aircraft)

    # def test_airfoils(self, build_desc_aircraft):
    #     ac = sim.Aircraft(build_desc_aircraft)
    #     assert len(ac.airfoils) == 1
    #
    # def test_thrusters(self, build_desc_aircraft):
    #     ac = sim.Aircraft(build_desc_aircraft)
    #     assert len(ac.thrusters) == 1
    #
    # def test_components(self, build_desc_aircraft):
    #     ac = sim.Aircraft(build_desc_aircraft)
    #     assert len(ac.components) == 3
    #
    # def test_mass(self, build_desc_aircraft):
    #     ac = sim.Aircraft(build_desc_aircraft)
    #     assert ac.mass == pytest.approx(2.2)

    def test_freefall(self, build_desc_aircraft):
        ac = sim.Aircraft("test_aircraft")
        ac.initialize(build_desc_aircraft)
        u = Inputs()
        ac.set_input(u)
        t, t_end = 0, 1
        while t < t_end:
            ac.step()
            t += DELTAT

        assert ac.state.position[2] > 0

    def test_glide(self, build_desc_aircraft):
        ac = sim.Aircraft("test_aircraft")
        ac.initialize(build_desc_aircraft)
        vel = 15
        q0 = EulerAngles(pitch=-10, in_degrees=True).to_quaternion()
        ac.set_param("init/velLin", np.array([vel, 0, 0]))
        ac.set_param("init/orientation", np.array([q0.x, q0.y, q0.z, q0.w]))
        ac.update_parameters()
        ac.init()
        u = Inputs()
        ac.set_input(u)
        t, t_end = 0, 5
        while t < t_end:
            ac.step()
            t += DELTAT

        tests = []
        # Position is reasonable
        tests.append(ac.state.position[0] > vel * t_end / 2)
        tests.append(abs(ac.state.position[1]) < 1)
        tests.append(ac.state.position[2] > 0)
        # Angles are reasonable
        angles = ac.state.attitude.to_euler()
        tests.append(angles.pitch < 0)
        tests.append(abs(angles.roll) < np.deg2rad(30))
        tests.append(abs(angles.yaw) < np.deg2rad(30))
        # Velocity is reasonable
        tests.append(ac.state.velocity_linear[0] > vel / 2)
        tests.append(abs(ac.state.velocity_linear[1]) < 2)
        tests.append(ac.state.velocity_linear[2] > 0)
        tests.append(ac.state.velocity_linear[2] < vel / 2)
        # Angular velocity is close to 0
        tests.append(abs(ac.state.velocity_angular[0]) < 0.1)
        tests.append(abs(ac.state.velocity_angular[1]) < 0.1)
        tests.append(abs(ac.state.velocity_angular[2]) < 0.1)
        assert np.all(tests)

    def test_roll(self, build_desc_aircraft):
        ac = sim.Aircraft("test_aircraft")
        ac.initialize(build_desc_aircraft)
        vel = 15
        q0 = EulerAngles(pitch=-10, in_degrees=True).to_quaternion()
        ac.set_param("init/velLin", np.array([vel, 0, 0]))
        ac.set_param("init/orientation", np.array([q0.x, q0.y, q0.z, q0.w]))
        ac.update_parameters()
        ac.init()
        u = Inputs([0.5, 0, 0, 0])
        ac.set_input(u)
        t, t_end = 0, 1
        while t < t_end:
            ac.step()
            t += DELTAT

        tests = []
        # Position is reasonable
        tests.append(ac.state.position[0] > vel * t_end / 2)
        tests.append(ac.state.position[1] > 0)
        tests.append(ac.state.position[2] > 0)
        # Angles are reasonable
        angles = ac.state.attitude.to_euler()
        tests.append(angles.pitch < 0)
        tests.append(0 < angles.roll < np.deg2rad(60))
        tests.append(0 < angles.yaw < np.deg2rad(30))
        # Velocity is reasonable
        tests.append(ac.state.velocity_linear[0] > vel / 2)
        tests.append(abs(ac.state.velocity_linear[1]) < 2)
        tests.append(abs(ac.state.velocity_linear[2]) < 1)
        # Angular velocity is reasonable
        tests.append(ac.state.velocity_angular[0] > 0)
        tests.append(abs(ac.state.velocity_angular[1]) < 0.1)
        tests.append(abs(ac.state.velocity_angular[2]) < 0.5)
        assert np.all(tests)

    def test_pitch(self, build_desc_aircraft):
        ac = sim.Aircraft("test_aircraft")
        ac.initialize(build_desc_aircraft)
        vel = 15
        q0 = EulerAngles(pitch=-10, in_degrees=True).to_quaternion()
        ac.set_param("init/velLin", np.array([vel, 0, 0]))
        ac.set_param("init/orientation", np.array([q0.x, q0.y, q0.z, q0.w]))
        ac.update_parameters()
        ac.init()
        u = Inputs([0, 0.5, 0, 0])
        ac.set_input(u)
        t, t_end = 0, 1
        while t < t_end:
            ac.step()
            t += DELTAT

        tests = []
        # Position is reasonable
        tests.append(ac.state.position[0] > vel * t_end / 2)
        tests.append(abs(ac.state.position[1]) < 1)
        tests.append(abs(ac.state.position[2]) < vel * t_end / 2)
        # Angles are reasonable
        angles = ac.state.attitude.to_euler()
        tests.append(0 < angles.pitch < np.deg2rad(90))
        tests.append(abs(angles.roll) < np.deg2rad(10))
        tests.append(abs(angles.yaw) < np.deg2rad(10))
        # Velocity is reasonable
        tests.append(ac.state.velocity_linear[0] > vel / 2)
        tests.append(abs(ac.state.velocity_linear[1]) < 2)
        tests.append(0 < ac.state.velocity_linear[2] < vel / 2)
        # Angular velocity is reasonable
        tests.append(abs(ac.state.velocity_angular[0]) < 0.1)
        tests.append(ac.state.velocity_angular[1] > 0.1)
        tests.append(abs(ac.state.velocity_angular[2]) < 0.1)
        assert np.all(tests)

    def test_thruster_1(self, build_desc_aircraft):
        ac = sim.Aircraft("test_aircraft")
        ac.initialize(build_desc_aircraft)
        ac.set_param("env/rho", 0)  # Nullify aerodynamic effects.
        ac.update_parameters()
        u = Inputs([0, 0, 1, 0])
        ac.set_input(u)
        ac.step()

        assert ac.state.velocity_linear[0] > 0
        assert ac.state.velocity_linear[1] == pytest.approx(0)
        assert ac.state.velocity_linear[2] > 0
        assert (
            ac.state.velocity_angular[0] < 0
        )  # Propeller torque is negative by default.
        assert ac.state.velocity_angular[1] == pytest.approx(0)
        assert ac.state.velocity_angular[2] < 0

    def test_thruster_2(self, build_desc_aircraft):
        ac = sim.Aircraft("test_aircraft")
        ac.initialize(build_desc_aircraft)
        vel = 15
        q0 = EulerAngles(pitch=-10, in_degrees=True).to_quaternion()
        ac.set_param("init/velLin", np.array([vel, 0, 0]))
        ac.set_param("init/orientation", np.array([q0.x, q0.y, q0.z, q0.w]))
        ac.update_parameters()
        ac.init()
        u = Inputs()
        ac.set_input(u)
        # Let aircraft glide
        t, t_end = 0, 3
        while t < t_end:
            ac.step()
            t += DELTAT
        state_1 = UavState.from_uavstate(ac.state)
        # Then apply full throttle
        u = Inputs([0, 0, 1, 0])
        ac.set_input(u)
        t_end += 3
        while t < t_end:
            ac.step()
            t += DELTAT
        state_2 = UavState.from_uavstate(ac.state)

        tests = []
        tests.append(state_1.velocity_linear[0] < state_2.velocity_linear[0])
        assert np.all(tests)


TEST_MODELS_ROOT = str(Path(__file__).resolve().parents[1] / "cpp" / "test_parameters")


def test_load_model_config_builds_aircraft():
    """A recipe loaded by name initializes an Aircraft that steps."""
    from last_letter.cpp_last_letter.cpp_programming_utils import load_model_config

    config = load_model_config("ardupilot_copter_linear", TEST_MODELS_ROOT)
    assert "linear_copter" in config
    ac = sim.Aircraft("ardupilot_copter_linear")
    ac.initialize(config)
    ac.set_input(Inputs([0.0, 0.0, 0.0, 0.379]))
    for _ in range(10):
        ac.step()
    assert np.all(np.isfinite(ac.state.position))
