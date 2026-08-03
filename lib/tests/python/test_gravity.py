#!/usr/bin/env python3
"""Tests for the gravity library.

usage:
    Have pytest run the tests in this file.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Sat 29 Jan 2022"
__copyright__ = "Copyright 2022, George Zogopoulos"

import numpy as np
import pytest

import last_letter_lib.gravity as gravity
from last_letter_lib.utils.math import Inertial
from last_letter_lib.utils.math import Vector3
from last_letter_lib.utils.uav import UavState


###############################################################################
# Create pytest fixtures

G = 9.81


def build_state():
    return UavState()


def build_inertial():
    i = Inertial()
    i.mass = 1
    return i


###############################################################################
# Declare the tests themselves ################################################


class TestGravitySimple:
    def test_simple_contructor(self):
        grav = gravity.GravitySimple()
        assert isinstance(grav, gravity.GravitySimple)

    def test_simple_norm(self):
        grav = gravity.GravitySimple()
        grav.calc_gravity(build_state())
        g_norm = np.linalg.norm(grav.get_wrench(build_state(), build_inertial()).force)
        assert g_norm == pytest.approx(G)

    def test_simple_vector(self):
        grav = gravity.GravitySimple()
        grav.calc_gravity(build_state())
        g_vec = grav.get_wrench(build_state(), build_inertial()).force
        assert np.allclose(g_vec, Vector3(0, 0, G).to_array())


class TestGravityClassic:
    G = 9.78

    def test_classic_contructor(self):
        grav = gravity.GravityClassic()
        assert isinstance(grav, gravity.GravityClassic)

    def test_classic_norm(self):
        grav = gravity.GravityClassic()
        grav.calc_gravity(build_state())
        g_norm = np.linalg.norm(grav.get_wrench(build_state(), build_inertial()).force)
        assert g_norm == pytest.approx(self.G, rel=0.01)

    def test_classic_vector(self):
        grav = gravity.GravityClassic()
        grav.calc_gravity(build_state())
        g_vec = grav.get_wrench(build_state(), build_inertial()).force
        assert np.allclose(g_vec, Vector3(0, 0, self.G).to_array(), 1e-3)
