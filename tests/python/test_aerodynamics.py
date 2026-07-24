#!/usr/bin/env python3
"""Tests for the aerodynamics module.

usage:
    Have pytest run the tests in this file.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Tue 18 Jan 2022"
__copyright__ = "Copyright 2022, George Zogopoulos"

import math

import numpy as np
import pytest

from last_letter_lib import aerodynamics
from last_letter_lib.environment import EnvironmentData
from last_letter_lib.utils.math import Pose
from last_letter_lib.utils.math import UnitQuaternion
from last_letter_lib.utils.math import EulerAngles
from last_letter_lib.utils.math import Vector3
from last_letter_lib.utils.math import Wrench
from last_letter_lib.utils.math import Inertial
from last_letter_lib.utils.uav import Airdata
from last_letter_lib.utils.uav import Inputs
from last_letter_lib.utils.uav import UavState


EPS = 1e-6

###############################################################################
# Generate parameters for passing them to the aerodynamics functionality #####


@pytest.fixture
def build_desc_airfoil_simple():
    """
    Build a simple test airfoil description
    """
    return """
    s: 1
    b: 2
    c: 0.5
    c_L_0: 0.3269
    c_L_alpha: 3.85
    c_D_0: 0.0322
    c_D_alpha: 0.0037
    c_Y_beta: 0.1
    c_l_0: 0
    c_l_deltaa: 1
    c_m_0: 0.1
    c_m_alpha: -0.5
    c_m_deltae: 1
    c_n_0: 0
    c_n_beta: 0.1
    c_n_deltar: 0.1
    alpha_stall: 0.26
    inertial:
        mass: 1.8
        tensor:
            j_xx: 0.8244
            j_yy: 1.135
            j_zz: 1.759
            j_xz: 0.1204
    """


@pytest.fixture
def build_airfoil_simple(build_desc_airfoil_simple):
    """
    Build a simple test airfoil
    """
    aero = aerodynamics.SimpleDrag("test_airfoil")
    aero.initialize(build_desc_airfoil_simple)
    return aero


@pytest.fixture
def build_uav_state():
    """
    Build a test UavState object
    """
    return UavState(Vector3(), UnitQuaternion(), Vector3(20, 0, 0), Vector3())


@pytest.fixture
def build_aero_inputs():
    """
    Build test airfoil Inputs
    """
    return np.array([0, 0, 0])


@pytest.fixture
def build_uav_inputs():
    """
    Build test UAV Inputs
    """
    return Inputs()


@pytest.fixture
def build_environment_data():
    """
    Build a test EnvironmentData object
    """
    return EnvironmentData()


###############################################################################
# Declare the tests themselves ################################################


def test_calc_dynamic_pressure():
    V_a = 10
    rho = 1.225
    assert aerodynamics.calc_dynamic_pressure(rho, V_a) == pytest.approx(
        0.5 * rho * V_a**2
    )


def test_calc_max_l_d():
    """
    Make sure the best lift-to-drag ratio can be found within 0.01deg accuracy.
    """
    Cl_coeffs = [1, 1]
    Cd_coeffs = [1, 0, 1]
    f_max = 1.207
    x_max = 0.4142
    tol = np.deg2rad(0.01)
    best_l_d, best_aoa = aerodynamics.calc_max_l_d(Cl_coeffs, Cd_coeffs)
    assert np.abs(best_l_d - f_max) / f_max < tol and np.abs(best_aoa - x_max) < tol


def test_calc_max_l_d_2():
    """
    Make sure a correct warning is thrown at an error.
    """
    # Issue impossible coefficients
    Cl_coeffs = [1, 0, 1]
    Cd_coeffs = [1, 1]
    with pytest.raises(RuntimeError):
        best_l_d, best_aoa = aerodynamics.calc_max_l_d(Cl_coeffs, Cd_coeffs)


def test_calc_bank_from_radius():
    g = 9.81
    tests = []
    # Test straight and level flight
    tests.append(aerodynamics.calc_bank_from_radius(np.inf, 23, 0, g) == 0)
    tests.append(
        aerodynamics.calc_bank_from_radius(np.inf, 23, np.deg2rad(10), g) == 0
    )  # Test straight climb
    tests.append(
        aerodynamics.calc_bank_from_radius(np.inf, 23, np.deg2rad(-10), g) == 0
    )  # Test straight descend
    tests.append(
        aerodynamics.calc_bank_from_radius(100, 10, 0, g)
        == np.arctan2(10**2 * np.cos(0), 100 * g)
    )  # Test level turn
    assert np.all(tests)


class TestStdLinearAero:
    def test_constructor(self):
        airfoil = aerodynamics.StdLinearAero("test_airfoild")
        assert isinstance(airfoil, aerodynamics.StdLinearAero)

    def test_inertial(self, build_airfoil_simple):
        airfoil = build_airfoil_simple
        assert airfoil.inertial.tensor[0, 0] == pytest.approx(0.8244)
        assert airfoil.inertial.tensor[1, 1] == pytest.approx(1.135)
        assert airfoil.inertial.tensor[2, 2] == pytest.approx(1.759)

    # def test_lift_coeff(
    #     self,
    #     build_airfoil_simple,
    #     build_uav_state,
    #     build_aero_inputs,
    #     build_environment_data,
    # ):
    #     airfoil = build_airfoil_simple
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     c_L = airfoil.lift_coeff(
    #         build_uav_state, build_environment_data, build_aero_inputs
    #     )
    #     assert c_L == pytest.approx(airfoil.params.c_L_0)

    def test_lift_coeff(
        self,
        build_airfoil_simple,
        build_uav_state,
        build_environment_data,
    ):
        airfoil = build_airfoil_simple
        airdata = Airdata()
        airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
        c_L = airfoil.lift_coeff(airdata.alpha)
        assert c_L == pytest.approx(airfoil.get_param_double("c_L_0"), rel=1e-4)

    # def test_drag_coeff(
    #     self,
    #     build_airfoil_simple,
    #     build_uav_state,
    #     build_aero_inputs,
    #     build_environment_data,
    # ):
    #     airfoil = build_airfoil_simple
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     c_d = airfoil.drag_coeff(
    #         build_uav_state, build_environment_data, build_aero_inputs
    #     )
    #     assert c_d == pytest.approx(airfoil.params.c_D_0)

    def test_drag_coeff(
        self,
        build_airfoil_simple,
        build_uav_state,
        build_environment_data,
    ):
        airfoil = build_airfoil_simple
        airdata = Airdata()
        airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
        c_D = airfoil.drag_coeff(airdata.alpha, airdata.beta)
        assert c_D == pytest.approx(airfoil.get_param_double("c_D_0"), rel=1e-4)

    # def test_lift_stall(
    #     self,
    #     build_airfoil_simple,
    #     build_uav_state,
    #     build_aero_inputs,
    #     build_environment_data,
    # ):
    #     airfoil = build_airfoil_simple
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     c_l_1 = airfoil.lift_coeff(
    #         build_uav_state, build_environment_data, build_aero_inputs
    #     )
    #     # Make a state in higher AoA
    #     airdata = Airdata(20, np.deg2rad(10), 0)
    #     airfoil._store_airdata(airdata)
    #     v = airdata.to_u()
    #     state_2 = UavState(
    #         Vector3(), UnitQuaternion(), Vector3(v[0], v[1], v[2]), Vector3()
    #     )
    #     c_l_2 = airfoil.lift_coeff(state_2, build_environment_data, build_aero_inputs)
    #     # Make a state post stall
    #     airdata = Airdata(20, airfoil.params.alpha_stall + np.deg2rad(10), 0)
    #     airfoil._store_airdata(airdata)
    #     v = airdata.to_u()
    #     state_3 = UavState(
    #         Vector3(), UnitQuaternion(), Vector3(v[0], v[1], v[2]), Vector3()
    #     )
    #     c_l_3 = airfoil.lift_coeff(state_3, build_environment_data, build_aero_inputs)
    #
    #     tests = []
    #     tests.append(c_l_1 < c_l_2)
    #     tests.append(c_l_2 > c_l_3)
    #     assert np.all(tests)

    def test_lift_stall(
        self,
        build_airfoil_simple,
        build_uav_state,
        build_environment_data,
    ):
        airfoil = build_airfoil_simple
        airdata = Airdata()
        airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
        c_l_1 = airfoil.lift_coeff(airdata.alpha)
        # Make a state in higher AoA
        airdata = Airdata(20, np.deg2rad(10), 0)
        v = airdata.to_u()
        c_l_2 = airfoil.lift_coeff(airdata.alpha)
        # Make a state post stall
        airdata = Airdata(
            20, airfoil.get_param_double("alpha_stall") + np.deg2rad(10), 0
        )
        v = airdata.to_u()
        c_l_3 = airfoil.lift_coeff(airdata.alpha)

        tests = []
        tests.append(c_l_1 < c_l_2)
        tests.append(c_l_2 > c_l_3)
        assert np.all(tests)

    # def test_drag_coeff_stall(
    #     self,
    #     build_airfoil_simple,
    #     build_uav_state,
    #     build_aero_inputs,
    #     build_environment_data,
    # ):
    #     """Test the correct drag directions in deep stall.
    #
    #     The drag coefficient should always be positive.
    #     Test for all 4 longitudinal quadrants.
    #     """
    #     airfoil = build_airfoil_simple
    #     state = build_uav_state
    #     new_velocity = np.array([20, 0, 0])
    #     new_velocity[0] = 5  # Aircraft is going forward with 0 pitch.
    #     new_velocity[2] = 5  # Aircraft is descending with 0 pitch.
    #     state.velocity_linear = new_velocity
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     c_d_1 = airfoil.drag_coeff(state, build_environment_data, build_aero_inputs)
    #     assert c_d_1 > 0
    #
    #     new_velocity[0] = 5  # Aircraft is going forward with 0 pitch.
    #     new_velocity[2] = -5  # Aircraft is ascending with 0 pitch.
    #     state.velocity_linear = new_velocity
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     c_d_2 = airfoil.drag_coeff(state, build_environment_data, build_aero_inputs)
    #     assert c_d_2 > 0
    #
    #     new_velocity[0] = -5  # Aircraft is going backwards with 0 pitch.
    #     new_velocity[2] = 5  # Aircraft is descending with 0 pitch.
    #     state.velocity_linear = new_velocity
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     c_d_3 = airfoil.drag_coeff(state, build_environment_data, build_aero_inputs)
    #     assert c_d_3 < 0
    #
    #     new_velocity[0] = -5  # Aircraft is going bachwards with 0 pitch.
    #     new_velocity[2] = -5  # Aircraft is ascending with 0 pitch.
    #     state.velocity_linear = new_velocity
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     c_d_4 = airfoil.drag_coeff(state, build_environment_data, build_aero_inputs)
    #     assert c_d_4 < 0

    def test_drag_coeff_stall(
        self,
        build_airfoil_simple,
        build_uav_state,
        # build_aero_inputs,
        build_environment_data,
    ):
        """Test the correct drag directions in deep stall.

        The drag coefficient should always be positive.
        Test for all 4 longitudinal quadrants.
        """
        airfoil = build_airfoil_simple
        state = build_uav_state
        new_velocity = np.array([20, 0, 0])
        new_velocity[0] = 5  # Aircraft is going forward with 0 pitch.
        new_velocity[2] = 5  # Aircraft is descending with 0 pitch.
        state.velocity_linear = new_velocity
        airdata = Airdata()
        airdata.init_from_state_wind(state, build_environment_data.wind)
        c_d_1 = airfoil.drag_coeff(airdata.alpha, airdata.beta)
        assert c_d_1 > 0

        new_velocity[0] = 5  # Aircraft is going forward with 0 pitch.
        new_velocity[2] = -5  # Aircraft is ascending with 0 pitch.
        state.velocity_linear = new_velocity
        airdata = Airdata()
        airdata.init_from_state_wind(state, build_environment_data.wind)
        c_d_2 = airfoil.drag_coeff(airdata.alpha, airdata.beta)
        assert c_d_2 > 0

        new_velocity[0] = -5  # Aircraft is going backwards with 0 pitch.
        new_velocity[2] = 5  # Aircraft is descending with 0 pitch.
        state.velocity_linear = new_velocity
        airdata = Airdata()
        airdata.init_from_state_wind(state, build_environment_data.wind)
        c_d_3 = airfoil.drag_coeff(airdata.alpha, airdata.beta)
        assert c_d_3 < 0

        new_velocity[0] = -5  # Aircraft is going bachwards with 0 pitch.
        new_velocity[2] = -5  # Aircraft is ascending with 0 pitch.
        state.velocity_linear = new_velocity
        airdata = Airdata()
        airdata.init_from_state_wind(state, build_environment_data.wind)
        c_d_4 = airfoil.drag_coeff(airdata.alpha, airdata.beta)
        assert c_d_4 < 0

    # def test_drag_stall(
    #     self,
    #     build_airfoil_simple,
    #     build_uav_state,
    #     build_uav_inputs,
    #     build_environment_data,
    # ):
    #     """Test the correct drag directions in deep stall.
    #
    #     Test for all 4 longitudinal quadrants.
    #     """
    #     airfoil = build_airfoil_simple
    #     state = build_uav_state
    #     state.attitude = UnitQuaternion.from_euler(
    #         EulerAngles(0, math.pi / 4, 0)
    #     )  # Pitch to 45 deg.
    #     new_velocity = np.array([0, 0, 0])
    #     new_velocity[0] = 5  # Aircraft is going forward.
    #     new_velocity[2] = 5  # Aircraft is going forward.
    #     state.velocity_linear = new_velocity
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     w_1 = airfoil.get_wrench_airfoil(
    #         state, build_environment_data, build_uav_inputs
    #     )
    #     assert np.abs(w_1.force[0]) < EPS  # No drag in the body frame.
    #     assert w_1.force[2] < 0  # Positive lift.
    #
    #     state.attitude = UnitQuaternion.from_euler(
    #         EulerAngles(0, -math.pi / 4, 0)
    #     )  # Pitch to -45 deg.
    #     new_velocity[0] = 5  # Aircraft is going forward.
    #     new_velocity[2] = -5  # Aircraft is going forward.
    #     state.velocity_linear = new_velocity
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     w_2 = airfoil.get_wrench_airfoil(
    #         state, build_environment_data, build_uav_inputs
    #     )
    #     assert np.abs(w_2.force[0]) < EPS  # No drag in the body frame.
    #     assert w_2.force[2] > 0  # Negative lift.
    #
    #     state.attitude = UnitQuaternion.from_euler(
    #         EulerAngles(0, math.pi / 4, 0)
    #     )  # Pitch to 45 deg.
    #     new_velocity[0] = -5  # Aircraft is going backwards.
    #     new_velocity[2] = -5  # Aircraft is going backwards.
    #     state.velocity_linear = new_velocity
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     w_3 = airfoil.get_wrench_airfoil(
    #         state, build_environment_data, build_uav_inputs
    #     )
    #     assert np.abs(w_3.force[0]) < EPS  # No drag in the body frame.
    #     assert w_3.force[2] > 0  # Negative lift.
    #
    #     state.attitude = UnitQuaternion.from_euler(
    #         EulerAngles(0, -math.pi / 4, 0)
    #     )  # Pitch to -45 deg.
    #     new_velocity[0] = -5  # Aircraft is going backwards.
    #     new_velocity[2] = 5  # Aircraft is going backwards.
    #     state.velocity_linear = new_velocity
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     w_4 = airfoil.get_wrench_airfoil(
    #         state, build_environment_data, build_uav_inputs
    #     )
    #     assert np.abs(w_4.force[0]) < EPS  # No drag in the body frame.
    #     assert w_4.force[2] < 0  # Positive lift.

    def test_drag_stall(
        self,
        build_airfoil_simple,
        build_uav_state,
        # build_uav_inputs,
        build_environment_data,
    ):
        """Test the correct drag directions in deep stall.

        Test for all 4 longitudinal quadrants.
        """
        airfoil = build_airfoil_simple
        state = build_uav_state
        state.attitude = UnitQuaternion.from_euler(
            EulerAngles(0, math.pi / 4, 0)
        )  # Pitch to 45 deg.
        new_velocity = np.array([0, 0, 0])
        new_velocity[0] = 5  # Aircraft is going forward.
        new_velocity[2] = 5  # Aircraft is going forward.
        state.velocity_linear = new_velocity
        airfoil.update_local_state(state, build_environment_data)
        airfoil.calc_model()
        w_1 = airfoil.wrench_sum.wrench_aero
        assert np.abs(w_1.force[0]) < EPS  # No drag in the body frame.
        assert w_1.force[2] < 0  # Positive lift.

        state.attitude = UnitQuaternion.from_euler(
            EulerAngles(0, -math.pi / 4, 0)
        )  # Pitch to -45 deg.
        new_velocity[0] = 5  # Aircraft is going forward.
        new_velocity[2] = -5  # Aircraft is going forward.
        state.velocity_linear = new_velocity
        airfoil.update_local_state(state, build_environment_data)
        airfoil.calc_model()
        w_2 = airfoil.wrench_sum.wrench_aero
        assert np.abs(w_2.force[0]) < EPS  # No drag in the body frame.
        assert w_2.force[2] > 0  # Negative lift.

        state.attitude = UnitQuaternion.from_euler(
            EulerAngles(0, math.pi / 4, 0)
        )  # Pitch to 45 deg.
        new_velocity[0] = -5  # Aircraft is going backwards.
        new_velocity[2] = -5  # Aircraft is going backwards.
        state.velocity_linear = new_velocity
        airfoil.update_local_state(state, build_environment_data)
        airfoil.calc_model()
        w_3 = airfoil.wrench_sum.wrench_aero
        assert np.abs(w_3.force[0]) < EPS  # No drag in the body frame.
        assert w_3.force[2] > 0  # Negative lift.

        state.attitude = UnitQuaternion.from_euler(
            EulerAngles(0, -math.pi / 4, 0)
        )  # Pitch to -45 deg.
        new_velocity[0] = -5  # Aircraft is going backwards.
        new_velocity[2] = 5  # Aircraft is going backwards.
        state.velocity_linear = new_velocity
        airfoil.update_local_state(state, build_environment_data)
        airfoil.calc_model()
        w_4 = airfoil.wrench_sum.wrench_aero
        assert np.abs(w_4.force[0]) < EPS  # No drag in the body frame.
        assert w_4.force[2] < 0  # Positive lift.

    # def test_sideforce_coeff(
    #     self,
    #     build_airfoil_simple,
    #     build_uav_state,
    #     build_aero_inputs,
    #     build_environment_data,
    # ):
    #     tests = []
    #     airfoil = build_airfoil_simple
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     c_y_1 = airfoil.sideforce_coeff(
    #         build_uav_state, build_environment_data, build_aero_inputs
    #     )
    #     tests.append(c_y_1 == pytest.approx(0))
    #
    #     # Generate positive AoS
    #     airdata = Airdata(20, 0, np.deg2rad(10))
    #     airfoil._store_airdata(airdata)
    #     v = airdata.to_u()
    #     state_2 = UavState(
    #         Vector3(), UnitQuaternion(), Vector3(v[0], v[1], v[2]), Vector3()
    #     )
    #     c_y_2 = airfoil.sideforce_coeff(
    #         state_2, build_environment_data, build_aero_inputs
    #     )
    #     tests.append(c_y_2 > 0)
    #
    #     # Generate negative AoS
    #     airdata = Airdata(20, 0, np.deg2rad(-10))
    #     airfoil._store_airdata(airdata)
    #     v = airdata.to_u()
    #     state_3 = UavState(
    #         Vector3(), UnitQuaternion(), Vector3(v[0], v[1], v[2]), Vector3()
    #     )
    #     c_y_3 = airfoil.sideforce_coeff(
    #         state_3, build_environment_data, build_aero_inputs
    #     )
    #     tests.append(c_y_3 < 0)
    #
    #     assert np.all(tests)

    # def test_pitch_coeff(
    #     self,
    #     build_airfoil_simple,
    #     build_uav_state,
    #     build_aero_inputs,
    #     build_environment_data,
    # ):
    #     tests = []
    #     airfoil = build_airfoil_simple
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     # Generate pitching moment at 0 AoA
    #     c_m_1 = airfoil.pitch_moment_coeff(
    #         build_uav_state, build_environment_data, build_aero_inputs
    #     )
    #     tests.append(c_m_1 == pytest.approx(airfoil.params.c_m_0))
    #
    #     # Generate pitching moment at positive AoA
    #     airdata = Airdata(20, np.deg2rad(10), 0)
    #     airfoil._store_airdata(airdata)
    #     v = airdata.to_u()
    #     state_2 = UavState(
    #         Vector3(), UnitQuaternion(), Vector3(v[0], v[1], v[2]), Vector3()
    #     )
    #     c_m_2 = airfoil.pitch_moment_coeff(
    #         state_2, build_environment_data, build_aero_inputs
    #     )
    #     tests.append(c_m_2 < c_m_1)
    #
    #     # Generate pitching moment at negative AoA
    #     airdata = Airdata(20, np.deg2rad(-10), 0)
    #     airfoil._store_airdata(airdata)
    #     v = airdata.to_u()
    #     state_3 = UavState(
    #         Vector3(), UnitQuaternion(), Vector3(v[0], v[1], v[2]), Vector3()
    #     )
    #     c_m_3 = airfoil.pitch_moment_coeff(
    #         state_3, build_environment_data, build_aero_inputs
    #     )
    #     tests.append(c_m_3 > c_m_1)
    #     # Generate pitching moment at positive deltae
    #     inputs = np.array((0, 0.1, 0))
    #     c_m_4 = airfoil.pitch_moment_coeff(
    #         build_uav_state, build_environment_data, inputs
    #     )
    #     tests.append(c_m_4 > c_m_1)
    #     # Generate pitching moment at negative deltae
    #     inputs = np.array((0, -0.1, 0))
    #     c_m_5 = airfoil.pitch_moment_coeff(
    #         build_uav_state, build_environment_data, inputs
    #     )
    #     tests.append(c_m_5 < c_m_1)
    #
    #     assert np.all(tests)

    # def test_roll_coeff(
    #     self,
    #     build_airfoil_simple,
    #     build_uav_state,
    #     build_aero_inputs,
    #     build_environment_data,
    # ):
    #     tests = []
    #     airfoil = build_airfoil_simple
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     # Generate rolling moment at 0 AoA
    #     c_m_1 = airfoil.roll_moment_coeff(
    #         build_uav_state, build_environment_data, build_aero_inputs
    #     )
    #     tests.append(c_m_1 == pytest.approx(airfoil.params.c_l_0))
    #
    #     # Generate rolling moment at positive deltaa
    #     inputs = np.array((0.1, 0, 0))
    #     c_m_2 = airfoil.roll_moment_coeff(
    #         build_uav_state, build_environment_data, inputs
    #     )
    #     tests.append(c_m_2 > c_m_1)
    #
    #     # Generate rolling moment at negative deltaa
    #     inputs = np.array((-0.1, 0, 0))
    #     c_m_3 = airfoil.roll_moment_coeff(
    #         build_uav_state, build_environment_data, inputs
    #     )
    #     tests.append(c_m_3 < c_m_1)
    #
    #     assert np.all(tests)

    # def test_yaw_coeff(
    #     self,
    #     build_airfoil_simple,
    #     build_uav_state,
    #     build_aero_inputs,
    #     build_environment_data,
    # ):
    #     tests = []
    #     airfoil = build_airfoil_simple
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     # Generate yawing moment at 0 AoS
    #     c_m_1 = airfoil.yaw_moment_coeff(
    #         build_uav_state, build_environment_data, build_aero_inputs
    #     )
    #     tests.append(c_m_1 == pytest.approx(airfoil.params.c_n_0))
    #
    #     # Generate yawing moment at positive AoS
    #     airdata = Airdata(20, 0, np.deg2rad(10))
    #     airfoil._store_airdata(airdata)
    #     v = airdata.to_u()
    #     state_2 = UavState(
    #         Vector3(), UnitQuaternion(), Vector3(v[0], v[1], v[2]), Vector3()
    #     )
    #     c_m_2 = airfoil.yaw_moment_coeff(
    #         state_2, build_environment_data, build_aero_inputs
    #     )
    #     tests.append(c_m_2 > c_m_1)
    #
    #     # Generate yawing moment at negative AoS
    #     airdata = Airdata(20, 0, np.deg2rad(-10))
    #     airfoil._store_airdata(airdata)
    #     v = airdata.to_u()
    #     state_3 = UavState(
    #         Vector3(), UnitQuaternion(), Vector3(v[0], v[1], v[2]), Vector3()
    #     )
    #     c_m_3 = airfoil.yaw_moment_coeff(
    #         state_3, build_environment_data, build_aero_inputs
    #     )
    #
    #     tests.append(c_m_3 < c_m_1)
    #     # Generate yawing moment at positive deltar
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     inputs = np.array((0, 0, 0.1))
    #     c_m_4 = airfoil.yaw_moment_coeff(
    #         build_uav_state, build_environment_data, inputs
    #     )
    #     tests.append(c_m_4 > c_m_1)
    #     # Generate yawing moment at negative deltar
    #     airdata = Airdata()
    #     airdata.init_from_state_wind(build_uav_state, build_environment_data.wind)
    #     airfoil._store_airdata(airdata)
    #     inputs = np.array((0, 0, -0.1))
    #     c_m_5 = airfoil.yaw_moment_coeff(
    #         build_uav_state, build_environment_data, inputs
    #     )
    #     tests.append(c_m_5 < c_m_1)
    #
    #     assert np.all(tests)

    def test_calc_forces(
        self,
        build_airfoil_simple,
        build_uav_state,
        build_uav_inputs,
        build_environment_data,
    ):
        # Test nominal force generation
        airfoil = build_airfoil_simple
        airfoil.set_input(build_uav_inputs)
        airfoil.update_local_state(build_uav_state, build_environment_data)
        airfoil.calc_model()
        forces = airfoil.wrench_sum.wrench_aero.force
        tests = []
        tests.append(isinstance(forces, np.ndarray))
        tests.append(forces[0] < 0)  # Drag is negative in the airfoil frame
        tests.append(forces[1] == pytest.approx(0))  # Sideforce is 0
        tests.append(forces[2] < 0)  # Lift is negative in the airfoil frame

        assert np.all(tests)

    def test_calc_moments(
        self,
        build_airfoil_simple,
        build_uav_state,
        build_uav_inputs,
        build_environment_data,
    ):
        # Test nominal moment generation
        airfoil = build_airfoil_simple
        airfoil.set_input(build_uav_inputs)
        airfoil.update_local_state(build_uav_state, build_environment_data)
        airfoil.calc_model()
        moments = airfoil.wrench_sum.wrench_aero.torque
        tests = []
        tests.append(isinstance(moments, np.ndarray))
        tests.append(moments[0] == pytest.approx(0))  # Roll moment is 0
        tests.append(moments[1] > 0)  # Pitch moment is positive
        tests.append(moments[2] == pytest.approx(0))  # Yaw moment is 0

        assert np.all(tests)

    def test_get_wrench(
        self,
        build_airfoil_simple,
        build_uav_state,
        # build_uav_inputs,
        build_environment_data,
    ):
        airfoil = build_airfoil_simple
        airfoil.update_local_state(build_uav_state, build_environment_data)
        airfoil.calc_model()
        wrench = airfoil.wrench_sum.wrench_aero
        tests = []
        tests.append(isinstance(wrench, Wrench))
        tests.append(isinstance(wrench.force, np.ndarray))
        tests.append(isinstance(wrench.torque, np.ndarray))

        assert np.all(tests)

    # The internally calculated forces and moments are now in the body frame, same as the returned wrench.
    # def test_correct_rotations(
    #     self,
    #     build_airfoil_simple,
    #     build_uav_inputs,
    #     build_environment_data,
    # ):
    #     """
    #     Verify the aerodynamic forces are correctly rotated to the body frame.
    #
    #     Follows Small Unmanned Aircraft Theory and Practice, R. Beard and T. McLain, p.49
    #     """
    #     airfoil = build_airfoil_simple
    #     alpha = np.deg2rad(10)
    #     airdata = Airdata(20, alpha, 0)
    #     v = airdata.to_u()
    #     state = UavState(
    #         Vector3(), UnitQuaternion(), Vector3(v[0], v[1], v[2]), Vector3()
    #     )
    #     # Get forces/moments in the airfoil frame
    #     inputs = np.array([0, 0, 0])
    #     airfoil.set_input(inputs)
    # airfoil.update_local_state(state, build_environment_data)
    # airfoil.calc_model()
    #     forces_a = airfoil.calc_forces(state, build_environment_data, inputs)
    #     moments_a = airfoil.calc_moments(state, build_environment_data, inputs)
    #     # Get wrench in body frame
    #     wrench = airfoil.get_wrench(state, build_environment_data, build_uav_inputs)
    #     tests = []
    #     # Test that forces are properly rotated
    #     tests.append(
    #         wrench.force[0]
    #         == pytest.approx(forces_a[0] * np.cos(alpha) - forces_a[2] * np.sin(alpha))
    #     )
    #     tests.append(
    #         wrench.force[2]
    #         == pytest.approx(forces_a[0] * np.sin(alpha) + forces_a[2] * np.cos(alpha))
    #     )
    #     # Test that moments are not rotated
    #     tests.append(
    #         np.linalg.norm(moments_a.to_array() - wrench.torque) == pytest.approx(0)
    #     )
    #
    #     assert np.all(tests)

    def test_pitch_moment_coeff_sign_1(
        self,
        build_airfoil_simple,
        build_uav_state,
        # build_aero_inputs,
        build_environment_data,
    ):
        """Test that the correct pitching moment sign is generated for forward flight."""
        airfoil = build_airfoil_simple
        state = build_uav_state
        new_velocity = np.array([20, 0, 0])
        new_velocity[2] = 5  # Aircraft is descending with 0 pitch.
        state.velocity_linear = new_velocity
        airfoil.update_local_state(state, build_environment_data)
        airfoil.calc_model()
        moments = airfoil.wrench_sum.wrench_aero.torque
        assert moments[1] < 0  # Pitch moment is negative

        new_velocity[2] = -5  # Aircraft is ascending with 0 pitch.
        state.velocity_linear = new_velocity
        airfoil.update_local_state(state, build_environment_data)
        airfoil.calc_model()
        moments = airfoil.wrench_sum.wrench_aero.torque
        assert moments[1] > 0  # Pitch moment is positive

    def test_pitch_moment_coeff_sign_2(
        self,
        build_airfoil_simple,
        build_uav_state,
        # build_aero_inputs,
        build_environment_data,
    ):
        """Test that the correct pitching moment sign is generated for rearwards flight."""
        airfoil = build_airfoil_simple
        state = build_uav_state
        new_velocity = np.array([20, 0, 0])
        new_velocity[0] = -20
        new_velocity[2] = 5  # Aircraft is descending with 0 pitch.
        state.velocity_linear = new_velocity
        airfoil.update_local_state(state, build_environment_data)
        airfoil.calc_model()
        moments = airfoil.wrench_sum.wrench_aero.torque
        assert moments[1] < 0  # Pitch moment is negative

        new_velocity[2] = -5  # Aircraft is ascending with 0 pitch.
        state.velocity_linear = new_velocity
        airfoil.update_local_state(state, build_environment_data)
        airfoil.calc_model()
        moments = airfoil.wrench_sum.wrench_aero.torque
        assert moments[1] > 0  # Pitch moment is positive
