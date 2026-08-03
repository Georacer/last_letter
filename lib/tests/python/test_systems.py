#!/usr/bin/env python3
"""Tests for the systems module.

usage:
    Have pytest run the tests in this file.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Sat 29 Jan 2022"
__copyright__ = "Copyright 2022, George Zogopoulos"

import numpy as np
import pytest

import last_letter_lib.systems as systems
from last_letter_lib.utils.math import UnitQuaternion
from last_letter_lib.utils.math import Vector3


###############################################################################
# Create pytest fixtures


@pytest.fixture
def build_inertia():
    ixx = 1
    iyy = 2
    izz = 4
    ixz = 0.1
    return np.array([[ixx, 0, -ixz], [0, iyy, 0], [-ixz, 0, izz]])


@pytest.fixture
def build_system(build_inertia):
    return systems.RigidBody6DOF(mass=1, inertia_matrix=build_inertia)


###############################################################################
# Declare the tests themselves ################################################


class TestRigidBody6DOF:
    def test_contructor(self, build_inertia):
        sys = systems.RigidBody6DOF(mass=1, inertia_matrix=build_inertia)
        assert isinstance(sys, systems.RigidBody6DOF)

    def test_mass(self, build_system):
        assert build_system.mass == 1

    def test_set_state(self, build_system):
        sys = build_system
        sys.state = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13])
        tests = []
        tests.append(sys.position == Vector3(1, 2, 3))
        tests.append(sys.orientation == UnitQuaternion(4, 5, 6, 7))
        tests.append(sys.velocity_linear == Vector3(8, 9, 10))
        tests.append(sys.velocity_angular == Vector3(11, 12, 13))
        assert np.all(tests)

    def test_immobile(self, build_system):
        # Test that the system remains immobile with 0 input
        body = build_system
        u = np.zeros(6)
        dt = 0.01
        x, t = body.rk4(u, dt)

        tests = []
        tests.append(np.allclose(x, np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])))
        tests.append(np.allclose(body.x_dot, np.zeros(13)))
        assert np.all(tests)

    def test_accelerate_1(self, build_system):
        # Test forward acceleration response
        body = build_system
        u = np.array([1, 0, 0, 0, 0, 0])
        dt = 0.01
        x, t = body.rk4(u, dt)

        tests = []
        tests.append(body.position == Vector3())
        tests.append(body.orientation == UnitQuaternion())
        tests.append(body.velocity_linear == Vector3(1 / body.mass * dt, 0, 0))
        tests.append(body.velocity_angular == Vector3())

        x, t = body.rk4(u, dt)
        tests.append(body.position.x > 0)
        tests.append(body.position.y == 0)
        tests.append(body.position.z == 0)
        assert np.all(tests)

    def test_accelerate_2(self, build_system):
        # Test backward acceleration response
        body = build_system
        u = np.array([-1, 0, 0, 0, 0, 0])
        dt = 0.01
        x, t = body.rk4(u, dt)

        tests = []
        tests.append(body.position == Vector3())
        tests.append(body.orientation == UnitQuaternion())
        tests.append(body.velocity_linear == Vector3(-1 / body.mass * dt, 0, 0))
        tests.append(body.velocity_angular == Vector3())

        x, t = body.rk4(u, dt)
        tests.append(body.position.x < 0)
        tests.append(body.position.y == 0)
        tests.append(body.position.z == 0)
        assert np.all(tests)

    def test_accelerate_3(self, build_system):
        # Test positive torque response
        body = build_system
        u = np.array([0, 0, 0, 1, 0, 0])
        dt = 0.01
        x, t = body.rk4(u, dt)

        tests = []
        tests.append(body.position == Vector3())
        tests.append(body.orientation == UnitQuaternion())
        tests.append(body.velocity_linear == Vector3())
        tests.append(body.velocity_angular.x > 0)
        tests.append(body.velocity_angular.y == 0)
        # Negative ixz element results in positive angular acceleration z
        tests.append(body.velocity_angular.z > 0)

        x, t = body.rk4(u, dt)
        euler = body.orientation.to_euler()
        tests.append(euler.roll > 0)
        # The cross product of omega x (J omega) yields negative pitch rate
        tests.append(euler.pitch < 0)
        tests.append(euler.yaw > 0)
        assert np.all(tests)

    def test_accelerate_4(self, build_system):
        # Test negative torque response
        body = build_system
        u = np.array([0, 0, 0, -1, 0, 0])
        dt = 0.01
        x, t = body.rk4(u, dt)

        tests = []
        tests.append(body.position == Vector3())
        tests.append(body.orientation == UnitQuaternion())
        tests.append(body.velocity_linear == Vector3())
        tests.append(body.velocity_angular.x < 0)
        tests.append(body.velocity_angular.y == 0)
        # Negative ixz element results in negative angular acceleration z
        tests.append(body.velocity_angular.z < 0)

        x, t = body.rk4(u, dt)
        euler = body.orientation.to_euler()
        tests.append(euler.roll < 0)
        # The cross product of omega x (J omega) yields negative pitch rate
        tests.append(euler.pitch < 0)
        tests.append(euler.yaw < 0)
        assert np.all(tests)
