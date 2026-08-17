#!/usr/bin/env python3
"""Tests for the propulsion module.

usage:
    Have pytest run the tests in this file.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Fri 12 Jan 2022"
__copyright__ = "Copyright 2021, George Zogopoulos"

import numpy as np
import pytest
import yaml

from last_letter import propulsion
from last_letter.environment import EnvironmentData
from last_letter.utils import math
from last_letter.utils.math import Inertial
from last_letter.utils.math import UnitQuaternion
from last_letter.utils.math import Vector3
from last_letter.utils.uav import Inputs
from last_letter.utils.uav import UavState


EPS = 1e-6

###############################################################################
# Generate data for passing them to the propulsion functionality #######


@pytest.fixture
def build_desc_propeller():
    """
    Build a 16x8 propeller
    """
    return """
    diameter: 0.4064
    pitch: 0.2032
    c_thrust:
        polyNo: 2
        coeffs: [0.064295, -0.045845, -0.161243]
    c_power:
        polyNo: 2
        coeffs: [0.018441, 0.016194, -0.084206]
    """


@pytest.fixture
def build_desc_thruster_simple():
    return """
    motorType: 6
    dt: 0.01
    chanMotor: 2
    thrustMin: 0
    thrustMax: 50
    torqueMin: 0
    torqueMax: 1
    """


@pytest.fixture
def build_desc_thruster_beard():
    return """
    motorType: 1
    dt: 0.01
    chanMotor: 2
    k_motor: 30
    c_prop: 0.01
    k_t_p: 0.001
    """


@pytest.fixture
def build_desc_thruster_electric_2():
    return """
    motorType: 5
    dt: 0.01
    chanMotor: 2
    prop_diam: 0.4064
    engInertia: 200e-6
    Kv: 6.5
    Rm: 0.1
    I0: 0.1
    Cells: 3
    propThrustPoly:
        polyType: 0
        polyNo: 2
        coeffs: [0.064295, -0.045845, -0.161243]
    propPowerPoly:
        polyType: 0
        polyNo: 2
        coeffs: [0.018441, 0.016194, -0.084206]
    """


@pytest.fixture
def build_desc_thruster_speed_controlled():
    return """
    motorType: 5
    chanMotor: 2
    omega_max: 942
    RadPSLimits: [0, 942]
    thrust_poly:
        polyType: 0
        polyNo: 2
        coeffs: [0.064295, -0.045845, -0.161243]
    power_poly:
        polyType: 0
        polyNo: 2
        coeffs: [0.018441, 0.016194, -0.084206]
    """


@pytest.fixture
def build_propeller_standard(build_desc_propeller):
    prop = propulsion.PropellerStandard("standard_propeller")
    prop.initialize(build_desc_propeller)
    return prop


@pytest.fixture
def build_propeller_standard_wrench(build_propeller_standard, V, n):
    prop = build_propeller_standard
    rho = 1.225
    wrench = prop.calc_wrench(V, n, rho)
    return wrench


# @pytest.fixture
# def build_motor_electric(build_desc_motor_electric):
#     return propulsion.MotorElectric(build_desc_motor_electric)


@pytest.fixture
def build_thruster_simple(build_desc_thruster_simple):
    thruster = propulsion.ThrusterSimple("simple_thruster")
    thruster.initialize(build_desc_thruster_simple)
    return thruster


@pytest.fixture
def build_thruster_beard(build_desc_thruster_beard):
    thruster = propulsion.EngBeard("beard_engine")
    thruster.initialize(build_desc_thruster_beard)
    return thruster


@pytest.fixture
def build_thruster_electric_2(build_desc_thruster_electric_2):
    thruster = propulsion.ElectricEng2("electric_engine_2")
    thruster.initialize(build_desc_thruster_electric_2)
    return thruster


@pytest.fixture
def build_thruster_speed_controlled(build_desc_thruster_speed_controlled):
    thruster = propulsion.EngOmegaControl("omega_controlled_engine")
    thruster.initialize(build_desc_thruster_speed_controlled)
    return thruster


@pytest.fixture
def build_uav_state():
    """
    Build a test UavState object
    """
    return UavState(Vector3(), UnitQuaternion(), Vector3(20, 0, 0), Vector3())


@pytest.fixture
def build_environment_data():
    """
    Build a test EnvironmentData object
    """
    return EnvironmentData()


###############################################################################
# Declare the tests themselves ################################################


class TestPropellerStandard:
    def test_constructor(self, build_desc_propeller):
        propeller = propulsion.PropellerStandard("test_propeller")
        propeller.initialize(build_desc_propeller)
        assert isinstance(propeller, propulsion.PropellerStandard)

    def test_diameter(self, build_desc_propeller, build_propeller_standard):
        data = yaml.load(build_desc_propeller, Loader=yaml.CLoader)
        assert build_propeller_standard.get_param_double("diameter") == data["diameter"]

    def test_calc_j(self, build_propeller_standard):
        V = 20  # airspeed
        n = 50  # propeller RPS
        D = build_propeller_standard.get_param_double("diameter")
        J = V / (n * D)
        tests = []
        tests.append(build_propeller_standard.calc_advance_ratio(V, n) == J)
        tests.append(build_propeller_standard.calc_advance_ratio(0, n) == 0)
        tests.append(build_propeller_standard.calc_advance_ratio(V, 0) == np.inf)
        tests.append(build_propeller_standard.calc_advance_ratio(-V, 0) == -np.inf)
        tests.append(build_propeller_standard.calc_advance_ratio(0, 0) == 0)
        assert np.all(tests)

    def test_c_thrust(self, build_desc_propeller, build_propeller_standard):
        data = yaml.load(build_desc_propeller, Loader=yaml.CLoader)
        c_thrust_iter = list(reversed(data["c_thrust"]["coeffs"]))
        V = 20  # airspeed
        n = 50  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        c_thrust = np.polyval(c_thrust_iter, J)
        assert build_propeller_standard.calc_coeff_thrust(J) == pytest.approx(c_thrust)

    def test_c_power(self, build_desc_propeller, build_propeller_standard):
        data = yaml.load(build_desc_propeller, Loader=yaml.CLoader)
        c_power_iter = list(reversed(data["c_power"]["coeffs"]))
        V = 20  # airspeed
        n = 50  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        c_power = np.polyval(c_power_iter, J)
        assert build_propeller_standard.calc_coeff_power(J) == c_power

    def test_c_static_thrust(self, build_desc_propeller, build_propeller_standard):
        data = yaml.load(build_desc_propeller, Loader=yaml.CLoader)
        c_thrust_iter = data["c_thrust"]["coeffs"]
        V = 0  # airspeed
        n = 50  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        assert build_propeller_standard.calc_coeff_thrust(J) == c_thrust_iter[0]

    def test_c_static_thrust_2(self, build_desc_propeller, build_propeller_standard):
        data = yaml.load(build_desc_propeller, Loader=yaml.CLoader)
        c_thrust_iter = data["c_thrust"]["coeffs"]
        V = 0  # airspeed
        n = 0  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        assert build_propeller_standard.calc_coeff_thrust(J) == c_thrust_iter[0]

    def test_c_static_power(self, build_desc_propeller, build_propeller_standard):
        data = yaml.load(build_desc_propeller, Loader=yaml.CLoader)
        c_power_iter = data["c_power"]["coeffs"]
        V = 0  # airspeed
        n = 50  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        assert build_propeller_standard.calc_coeff_power(J) == c_power_iter[0]

    def test_c_static_power_2(self, build_desc_propeller, build_propeller_standard):
        data = yaml.load(build_desc_propeller, Loader=yaml.CLoader)
        c_power_iter = data["c_power"]["coeffs"]
        V = 0  # airspeed
        n = 0  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        assert build_propeller_standard.calc_coeff_power(J) == c_power_iter[0]

    def test_efficiency(self, build_desc_propeller, build_propeller_standard):
        data = yaml.load(build_desc_propeller, Loader=yaml.CLoader)
        c_thrust_iter = data["c_thrust"]["coeffs"]
        c_power_iter = data["c_power"]["coeffs"]
        V = 20  # airspeed
        n = 50  # propeller RPS
        J = build_propeller_standard.calc_advance_ratio(V, n)
        c_thrust = np.polyval(list(reversed(c_thrust_iter)), J)
        c_power = np.polyval(list(reversed(c_power_iter)), J)
        efficiency = c_thrust * J / c_power
        assert build_propeller_standard.calc_efficiency(J) == pytest.approx(efficiency)

    def test_efficiency_zero_V(self, build_propeller_standard):
        J = 0
        assert build_propeller_standard.calc_efficiency(J) == 0

    def test_power(self, build_propeller_standard):
        V = 1  # airspeed
        n = 50  # propeller RPS
        assert build_propeller_standard.calc_power(V, n, 1.225) > 0

    def test_torque(self, build_propeller_standard):
        """
        Verify torque is zero at zero airspeed and RPM
        """
        assert build_propeller_standard.calc_torque(0, 0, 1.225) == 0

    @pytest.mark.parametrize("V, n", [(0, 100)])
    def test_wrench(self, build_propeller_standard_wrench):
        """
        Verify correct constructor instance
        """
        assert isinstance(build_propeller_standard_wrench, math.Wrench)

    @pytest.mark.parametrize("V, n", [(0, 100)])
    def test_wrench_2(self, build_propeller_standard_wrench):
        """
        Verify thrust is only on the positive x-axis in absence of sidewind
        """
        wrench = build_propeller_standard_wrench
        tests = []
        tests.append(wrench.force[0] > 0)
        tests.append(wrench.force[1] == 0)
        tests.append(wrench.force[2] == 0)
        tests.append(wrench.torque[0] > 0)
        tests.append(wrench.torque[1] == 0)
        tests.append(wrench.torque[2] == 0)
        assert np.all(tests)

    @pytest.mark.parametrize("V, n", [(10, 0.001)])
    def test_wrench_3(self, build_propeller_standard_wrench):
        """
        Verify propeller produces negative thrust when free-wheeling
        """
        wrench = build_propeller_standard_wrench
        assert wrench.force[0] < 0

    # def test_propeller_factory(self, build_desc_propeller):
    #     propeller = propulsion.build_propeller(build_desc_propeller)
    #     assert isinstance(propeller, propulsion.PropellerStandard)


class TestThruster:
    # Test ThrusterSimple class
    def test_thruster_simple_constructor(self, build_desc_thruster_simple):
        thruster = propulsion.ThrusterSimple("test_thruster")
        thruster.initialize(build_desc_thruster_simple)
        assert isinstance(thruster, propulsion.ThrusterSimple)

    # def test_thruster_simple_factory(self, build_desc_thruster_simple):
    #     thruster = propulsion.build_thruster(build_desc_thruster_simple)
    #     assert isinstance(thruster, propulsion.ThrusterSimple)

    def test_thruster_simple_normal_operation(
        self, build_thruster_simple, build_uav_state, build_environment_data
    ):
        thruster = build_thruster_simple

        delta_t = 0.5
        inputs = Inputs.from_array([0, 0, delta_t, 0])
        thruster.set_input(inputs)
        thruster.update_local_state(build_uav_state, build_environment_data)
        thruster.calc_model()
        wrench = thruster.wrench_sum.wrench_prop

        tests = []
        tests.append(
            wrench.force[0]
            == delta_t
            * (
                thruster.get_param_double("thrustMax")
                - thruster.get_param_double("thrustMin")
            )
            + thruster.get_param_double("thrustMin")
        )
        tests.append(
            wrench.torque[0]
            == -(
                delta_t
                * (
                    thruster.get_param_double("torqueMax")
                    - thruster.get_param_double("torqueMin")
                )
                + thruster.get_param_double("torqueMin")
            )
        )
        assert np.all(tests)

    # def test_thruster_simple_input_range_min(self, build_thruster_simple):
    #     thruster = build_thruster_simple
    #     with pytest.raises(ValueError):
    #         _, _ = thruster.rk4(np.array([-0.1]), 0.01)
    #         _ = thruster.y
    #
    # def test_thruster_simple_input_range_max(self, build_thruster_simple):
    #     thruster = build_thruster_simple
    #     with pytest.raises(ValueError):
    #         _, _ = thruster.rk4(np.array([1.1]), 0.01)
    #         _ = thruster.y

    def test_thruster_simple_direction(
        self, build_thruster_simple, build_uav_state, build_environment_data
    ):
        thruster = build_thruster_simple
        thruster.set_param("rotationDir", -1)
        thruster.update_parameters()

        delta_t = 0.5
        inputs = Inputs.from_array([0, 0, delta_t, 0])
        thruster.set_input(inputs)
        thruster.update_local_state(build_uav_state, build_environment_data)
        thruster.calc_model()
        wrench = thruster.wrench_sum.wrench_prop
        assert wrench.torque[0] > 0

    # Test ThrusterBeard class
    def test_thruster_beard_constructor(self, build_desc_thruster_beard):
        thruster = propulsion.EngBeard("test_thruster")
        thruster.initialize(build_desc_thruster_beard)
        assert isinstance(thruster, propulsion.EngBeard)

    # def test_thruster_beard_factory(self, build_desc_thruster_beard):
    #     thruster = propulsion.build_thruster(build_desc_thruster_beard)
    #     assert isinstance(thruster, propulsion.ThrusterBeard)

    def test_thruster_beard_normal_operation(
        self, build_thruster_beard, build_uav_state, build_environment_data
    ):
        thruster = build_thruster_beard
        state = build_uav_state
        state.velocity_linear = np.array([0, 0, 0])

        delta_t = 0.5
        inputs = Inputs.from_array([0, 0, delta_t, 0])
        thruster.set_input(inputs)
        thruster.update_local_state(build_uav_state, build_environment_data)
        thruster.calc_model()
        wrench = thruster.wrench_sum.wrench_prop

        tests = []
        tests.append(wrench.force[0] > 0)
        tests.append(wrench.torque[0] < 0)
        assert np.all(tests)

    def test_thruster_beard_overspeed(
        self, build_thruster_beard, build_uav_state, build_environment_data
    ):
        thruster = build_thruster_beard
        state = build_uav_state
        pitch_speed = thruster.get_param_double("k_motor") + 10
        state.velocity_linear = np.array([pitch_speed, 0, 0])

        delta_t = 0.5
        inputs = Inputs.from_array([0, 0, delta_t, 0])
        thruster.set_input(inputs)
        thruster.update_local_state(build_uav_state, build_environment_data)
        thruster.calc_model()
        wrench = thruster.wrench_sum.wrench_prop
        print(wrench.force)
        print(wrench.torque)

        tests = []
        tests.append(wrench.force[0] == 0)  # Velocity capped to min 0.
        tests.append(wrench.torque[0] < 0)
        assert np.all(tests)

    # Test ThrusterElectric class
    def test_thruster_electric_2_constructor(self, build_desc_thruster_electric_2):
        thruster = propulsion.ElectricEng2("test_thruster")
        thruster.initialize(build_desc_thruster_electric_2)
        assert isinstance(thruster, propulsion.ElectricEng2)

    # def test_thruster_electric_factory(self, build_desc_thruster_electric):
    #     thruster = propulsion.build_thruster(build_desc_thruster_electric)
    #     assert isinstance(thruster, propulsion.ThrusterElectric)

    def test_thruster_electric_2_normal_operation(
        self, build_thruster_electric_2, build_uav_state, build_environment_data
    ):
        thruster = build_thruster_electric_2
        state = build_uav_state
        state.velocity_linear = np.array([0, 0, 0])

        delta_t = 0.5
        inputs = Inputs.from_array([0, 0, delta_t, 0])
        thruster.set_input(inputs)
        thruster.update_local_state(build_uav_state, build_environment_data)
        for _ in range(100):  # TODO: Once you bind DynamicSystem, check t=1 to stop.
            thruster.calc_model()

        wrench = thruster.wrench_sum.wrench_prop

        tests = []
        tests.append(wrench.force[0] > 0)  # Thrust positive
        tests.append(wrench.torque[0] < 0)  # Torque negative
        tests.append(thruster.omega > 0)  # velocity positive
        # tests.append(wrench.thrust[7] > 0)  # current positive

        assert np.all(tests)

    def test_thruster_electric_2_idle_steady_state(
        self, build_thruster_electric_2, build_uav_state, build_environment_data
    ):
        """
        Make sure the thruster reaches a reasonable steady-state
        """
        thruster = build_thruster_electric_2
        delta_t = 0.5
        inputs = Inputs.from_array([0, 0, delta_t, 0])
        thruster.set_input(inputs)
        state = build_uav_state
        state.velocity_linear = np.array([0, 0, 0])

        omega = 1
        omega_prev = 0
        thruster.update_local_state(build_uav_state, build_environment_data)
        for _ in range(1000):  # TODO: Once you bind DynamicSystem, check t=10 to stop.
            thruster.calc_model()
            omega_prev = omega
            omega = thruster.omega

        tests = []
        tests.append(np.abs(omega - omega_prev) < EPS)  # Speed is constant
        tests.append(omega > math.rpm2radps(1))  # Speed is significant
        tests.append(omega < math.rpm2radps(20000))  # Speed is reasonable

        assert np.all(tests)

    # Test ThrusterSpeedControlled class
    def test_thruster_speed_controlled_constructor(
        self, build_desc_thruster_speed_controlled
    ):
        thruster = propulsion.EngOmegaControl("test_thruster")
        thruster.initialize(build_desc_thruster_speed_controlled)
        assert isinstance(thruster, propulsion.EngOmegaControl)

    # def test_thruster_speed_controlled_factory(
    #     self, build_desc_thruster_speed_controlled
    # ):
    #     thruster = propulsion.build_thruster(build_desc_thruster_speed_controlled)
    #     assert isinstance(thruster, propulsion.ThrusterSpeedControlled)

    def test_thruster_speed_controlled_idle(
        self, build_thruster_speed_controlled, build_uav_state, build_environment_data
    ):
        """
        Make sure the thruster produces zero thrust on idle
        """
        thruster = build_thruster_speed_controlled
        state = build_uav_state
        state.velocity_linear = np.array([0, 0, 0])

        delta_t = 0
        inputs = Inputs.from_array([0, 0, delta_t, 0])
        thruster.set_input(inputs)
        thruster.update_local_state(build_uav_state, build_environment_data)
        thruster.calc_model()

        wrench = thruster.wrench_sum.wrench_prop

        tests = []
        tests.append(wrench.force[0] == 0)
        tests.append(wrench.torque[0] == 0)

        assert np.all(tests)

    def test_thruster_speed_controlled_output(
        self, build_thruster_speed_controlled, build_uav_state, build_environment_data
    ):
        thruster = build_thruster_speed_controlled
        state = build_uav_state
        state.velocity_linear = np.array([0, 0, 0])

        omega_ref = 700
        delta_t = omega_ref / thruster.get_param_double("omega_max")
        inputs = Inputs.from_array([0, 0, delta_t, 0])
        thruster.set_input(inputs)
        thruster.update_local_state(build_uav_state, build_environment_data)
        thruster.calc_model()

        tests = []
        tests.append(omega_ref == thruster.omega)

        assert np.all(tests)

    def test_thruster_speed_controlled_normal_operation(
        self, build_thruster_speed_controlled, build_uav_state, build_environment_data
    ):
        thruster = build_thruster_speed_controlled
        state = build_uav_state
        state.velocity_linear = np.array([0, 0, 0])

        omega_ref = 700
        delta_t = omega_ref / thruster.get_param_double("omega_max")
        inputs = Inputs.from_array([0, 0, delta_t, 0])
        thruster.set_input(inputs)
        thruster.update_local_state(build_uav_state, build_environment_data)
        thruster.calc_model()
        wrench = thruster.wrench_sum.wrench_prop

        tests = []
        tests.append(wrench.force[0] > 0)
        tests.append(wrench.force[0] < 100)
        tests.append(wrench.torque[0] < 0)
        tests.append(wrench.torque[0] > -10)

        assert np.all(tests)
