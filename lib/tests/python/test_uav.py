#!/usr/bin/env python3
"""Tests for the uav library.

usage:
    Have pytest run the tests in this file.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Fri 21 Jan 2022"
__copyright__ = "Copyright 2022, George Zogopoulos"

import numpy as np
import pytest

import last_letter.utils.uav as uav
from last_letter import aerodynamics
from last_letter.environment import EnvironmentData
from last_letter.utils.math import EulerAngles
from last_letter.utils.math import Pose
from last_letter.utils.math import UnitQuaternion
from last_letter.utils.math import Vector3
from last_letter.utils.uav import Airdata
from last_letter.utils.uav import Inputs
from last_letter.utils.uav import UavState


###############################################################################
# Declare the tests themselves ################################################

# Mirrors num_input_channels in uav_utils.hpp, which is not exposed to Python.
NUM_INPUT_CHANNELS = 32


class TestInputs:
    def test_constructor(self):
        inputs = Inputs()
        assert len(inputs) == NUM_INPUT_CHANNELS
        assert np.allclose(inputs.to_array(), np.zeros(NUM_INPUT_CHANNELS))

    def test_constructor_from_list(self):
        inputs = Inputs([1, 2, 3, 4, 5])
        assert np.allclose(
            inputs.to_array(),
            np.array([1, 2, 3, 4, 5] + [0] * (NUM_INPUT_CHANNELS - 5)),
        )

    def test_constructor_too_long(self):
        with pytest.raises(ValueError):
            Inputs([0] * (NUM_INPUT_CHANNELS + 1))

    def test_to_array(self):
        inputs = Inputs([1, 2, 3, 4, 5])
        assert np.allclose(
            inputs.to_array(), Inputs.from_array([1, 2, 3, 4, 5]).to_array()
        )

    def test_getitem_setitem(self):
        inputs = Inputs()
        inputs[2] = 0.5
        assert inputs[2] == 0.5
        assert inputs.to_array()[2] == 0.5
        with pytest.raises(IndexError):
            inputs[NUM_INPUT_CHANNELS]


class TestUavState:
    def test_contructor(self):
        state = UavState(Vector3(), UnitQuaternion(), Vector3(), Vector3())
        assert isinstance(state, UavState)

    def test_to_array(self):
        state = UavState(
            Vector3(1, 2, 3), UnitQuaternion(), Vector3(4, 5, 6), Vector3(7, 8, 9)
        )
        result = np.array([1, 2, 3, 1, 0, 0, 0, 4, 5, 6, 7, 8, 9])
        assert np.allclose(state.to_array(), result)

    def test_from_array(self):
        state = UavState.from_array([1, 2, 3, 1, 0, 0, 0, 4, 5, 6, 7, 8, 9, 10, 11])
        tests = []
        tests.append(np.all(state.position == [1, 2, 3]))
        tests.append(state.attitude == UnitQuaternion(1, 0, 0, 0))
        tests.append(np.all(state.velocity_linear == [4, 5, 6]))
        tests.append(np.all(state.velocity_angular == [7, 8, 9]))
        tests.append(np.all(state.thrusters_velocity == [10, 11]))
        assert np.all(tests)


def test_calc_airdata_at_link():
    """
    Test the correct way to calculate the position error of an airdata boom.
    """
    probe_pose = Pose(
        position=Vector3(0, 1, 0).to_array(),
        orientation=EulerAngles(0, 45, 0, in_degrees=True).to_quaternion(),
    )
    body_airdata = Airdata()
    body_airdata.airspeed = 10
    body_vel_ang = Vector3(1, 0, 0)
    probe_airdata = uav.calc_airdata_at_link(body_airdata, body_vel_ang, probe_pose)
    probe_wind = probe_airdata.to_u()
    probe_wind_des = np.array([5 * np.sqrt(2), 0, 5 * np.sqrt(2) + 1])
    assert np.allclose(probe_wind, probe_wind_des)


# TODO: Add tests for the AoA sign for the various UAV motions, forward and backward
class TestAerodynamicTriplet:
    """
    Test the correct signs of the aerodynamic angles.

    Sweep AoA for the four quadrants of longitudinal motion.
    """

    def test_aoa_q4(self):
        """Motion towards front and down."""
        # airdata = Airdata().init_from_velocity(vel_body=np.array([1, 0, 1]))
        airdata = Airdata()
        airdata.init_from_velocity(vel_body=np.array([1, 0, 1]))
        assert airdata.alpha > 0 and airdata.alpha < np.pi

    def test_aoa_q1(self):
        """Motion towards front and up."""
        airdata = Airdata()
        airdata.init_from_velocity(vel_body=np.array([1, 0, -1]))
        assert airdata.alpha < 0 and airdata.alpha > -np.pi

    def test_aoa_q2(self):
        """Motion towards back and up."""
        airdata = Airdata()
        airdata.init_from_velocity(vel_body=np.array([-1, 0, -1]))
        assert airdata.alpha < 0 and airdata.alpha > -np.pi

    def test_aoa_q3(self):
        """Motion towards back and down."""
        airdata = Airdata()
        airdata.init_from_velocity(vel_body=np.array([-1, 0, 1]))
        assert airdata.alpha > 0 and airdata.alpha < np.pi
