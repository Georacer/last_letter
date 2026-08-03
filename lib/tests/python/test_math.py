#!/usr/bin/env python3
"""Tests for the math library.

usage:
    Have pytest run the tests in this file.
"""

__authors__ = ["George Zogopoulos"]
__credits__ = []
__date__ = "Tue 18 Jan 2022"
__copyright__ = "Copyright 2021, George Zogopoulos"

import numpy as np
import pytest

from last_letter_lib.utils import math


def test_calc_poly_zero_crossing():
    root_1 = 1
    root_2 = 2
    poly_coeffs = [1, -(root_1 + root_2), root_1 * root_2]
    xtol = 0.01

    root = math.calc_poly_zero_crossing(poly_coeffs, xtol)
    assert abs(root - root_1) < xtol or abs(root - root_2) < xtol


def test_calc_poly_zero_crossing_failure():
    poly_coeffs = [1, 0, 1]

    with pytest.raises(RuntimeError):
        _ = math.calc_poly_zero_crossing(poly_coeffs)


def test_sigmoid():
    inflection_point = 1
    width = 50

    tests = []
    tests.append(
        math.sigmoid(0, inflection_point, width) == pytest.approx(0)
    )  # Sigmoid is 0 close to 0
    tests.append(
        math.sigmoid(10, inflection_point, width) == pytest.approx(1)
    )  # Sigmoid is 1 past positive inflection point
    tests.append(
        math.sigmoid(-10, inflection_point, width) == pytest.approx(1)
    )  # Sigmoid is 1 before negative inflection point
    tests.append(
        math.sigmoid(inflection_point, inflection_point, width) == 0.5
    )  # Sigmoid is 0.5 at inflection point
    assert np.all(tests)


class TestVector3:
    def test_constructor(self):
        assert isinstance(math.Vector3(1, 2, 3), math.Vector3)

    def test_constructor_from_array(self):
        vec = math.build_vector3_from_array(np.array([1, 2, 3]))
        tests = []
        tests.append(vec.x == 1)
        tests.append(vec.y == 2)
        tests.append(vec.z == 3)
        assert np.all(tests)

    def test_to_array(self):
        vec = math.build_vector3_from_array(np.array([1, 2, 3]))
        assert np.allclose(vec.to_array(), np.array([1, 2, 3]))

    def test_to_array_2(self):
        vec = math.build_vector3_from_array(np.array([[1, 2, 3]]).T)
        assert np.allclose(vec.to_array(), np.array([1, 2, 3]))

    def test_addtion(self):
        vec1 = math.build_vector3_from_array(np.array([1, 2, 3]))
        vec2 = math.build_vector3_from_array(np.array([4, 5, 6]))
        vec3 = vec1 + vec2
        tests = []
        tests.append(vec3.x == 5)
        tests.append(vec3.y == 7)
        tests.append(vec3.z == 9)
        assert np.all(tests)

    def test_multiplication(self):
        vec1 = math.build_vector3_from_array(np.array([1, 2, 3]))
        vec2 = math.build_vector3_from_array(np.array([2, 4, 6]))
        assert 2 * vec1 == vec2

    def test_multiplication_2(self):
        vec1 = math.build_vector3_from_array(np.array([1, 2, 3]))
        vec2 = math.build_vector3_from_array(np.array([2, 4, 6]))
        assert vec1 * 2 == vec2

    def test_iterator(self):
        vec = math.build_vector3_from_array(np.array([1, 2, 3]))
        assert list(vec) == [1, 2, 3]


###############################################################################
# Test rotations


class TestEuler:
    def test_constructor_degrees(self):
        euler = math.EulerAngles(roll=90, pitch=90, yaw=90, in_degrees=True)
        tests = []
        tests.append(euler.roll == np.pi / 2)
        tests.append(euler.pitch == np.pi / 2)
        tests.append(euler.yaw == np.pi / 2)
        assert np.all(tests)

    # def test_constructor_from_vector(self):
    #     euler = math.EulerAngles.from_array([10, 20, 30], in_degrees=True)
    #     tests = []
    #     tests.append(euler.roll == np.deg2rad(10))
    #     tests.append(euler.pitch == np.deg2rad(20))
    #     tests.append(euler.yaw == np.deg2rad(30))
    #     assert np.all(tests)

    def test_constructor_from_quaternion(self):
        euler = math.EulerAngles.from_quaternion(math.UnitQuaternion(1, 0, 0, 0))
        tests = []
        tests.append(euler.roll == 0)
        tests.append(euler.pitch == 0)
        tests.append(euler.yaw == 0)
        assert np.all(tests)

    def test_constructor_from_quaternion_2(self):
        euler = math.EulerAngles.from_quaternion(math.UnitQuaternion(1, 0, 0, 0))
        tests = []
        tests.append(euler.roll == 0)
        tests.append(euler.pitch == 0)
        tests.append(euler.yaw == 0)
        assert np.all(tests)

    # Temporarily unsupported
    # def test_to_vector_degrees(self):
    #     euler = math.EulerAngles(10, 20, 30, in_degrees=True)
    #     assert np.allclose(euler.to_array(in_degrees=True), np.array([10, 20, 30]))

    # Equality with vector temporarily unsupported
    # def test_equals(self):
    #     v = np.deg2rad([10, 20, 30])
    #     euler = math.EulerAngles(v[0], v[1], v[2])
    #     assert euler == v

    def test_yaw(self):
        yaw = np.deg2rad(45)
        tests = []

        euler = math.EulerAngles(0, 0, yaw)
        R_bi = euler.R_bi()
        euler_2 = math.EulerAngles.from_rotmat(R_bi)
        tests.append(euler_2.roll == 0)
        tests.append(euler_2.pitch == 0)
        tests.append(euler_2.yaw == pytest.approx(yaw))

        tests.append(R_bi[0, 0] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[0, 1] == pytest.approx(-np.sqrt(2) / 2))
        tests.append(R_bi[0, 2] == pytest.approx(0))
        tests.append(R_bi[1, 0] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[1, 1] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[1, 2] == pytest.approx(0))

        assert np.all(tests)

    def test_yaw_2(self):
        yaw = np.deg2rad(-45)
        tests = []

        euler = math.EulerAngles(0, 0, yaw)
        R_bi = euler.R_bi()

        tests.append(R_bi[0, 0] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[0, 1] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[0, 2] == pytest.approx(0))
        tests.append(R_bi[1, 0] == pytest.approx(-np.sqrt(2) / 2))
        tests.append(R_bi[1, 1] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[1, 2] == pytest.approx(0))

        assert np.all(tests)

    def test_pitch(self):
        pitch = np.deg2rad(45)
        tests = []

        euler = math.EulerAngles(0, pitch, 0)
        R_bi = euler.R_bi()
        euler_2 = math.EulerAngles.from_rotmat(R_bi)
        tests.append(euler_2.roll == 0)
        tests.append(euler_2.pitch == pytest.approx(pitch))
        tests.append(euler_2.yaw == 0)

        tests.append(R_bi[0, 0] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[0, 1] == pytest.approx(0))
        tests.append(R_bi[0, 2] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[2, 0] == pytest.approx(-np.sqrt(2) / 2))
        tests.append(R_bi[2, 1] == pytest.approx(0))
        tests.append(R_bi[2, 2] == pytest.approx(np.sqrt(2) / 2))

        assert np.all(tests)

    def test_pitch_2(self):
        pitch = np.deg2rad(-45)
        tests = []

        euler = math.EulerAngles(0, pitch, 0)
        R_bi = euler.R_bi()

        tests.append(R_bi[0, 0] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[0, 1] == pytest.approx(0))
        tests.append(R_bi[0, 2] == pytest.approx(-np.sqrt(2) / 2))
        tests.append(R_bi[2, 0] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[2, 1] == pytest.approx(0))
        tests.append(R_bi[2, 2] == pytest.approx(np.sqrt(2) / 2))

        assert np.all(tests)

    def test_roll(self):
        roll = np.deg2rad(45)
        tests = []

        euler = math.EulerAngles(roll, 0, 0)
        R_bi = euler.R_bi()
        euler_2 = math.EulerAngles.from_rotmat(R_bi)
        tests.append(euler_2.roll == pytest.approx(roll))
        tests.append(euler_2.pitch == 0)
        tests.append(euler_2.yaw == 0)

        tests.append(R_bi[0, 0] == pytest.approx(1))
        tests.append(R_bi[0, 1] == pytest.approx(0))
        tests.append(R_bi[0, 2] == pytest.approx(0))
        tests.append(R_bi[1, 0] == pytest.approx(0))
        tests.append(R_bi[1, 1] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[1, 2] == pytest.approx(-np.sqrt(2) / 2))

        assert np.all(tests)

    def test_roll_2(self):
        roll = np.deg2rad(-45)
        tests = []

        euler = math.EulerAngles(roll, 0, 0)
        R_bi = euler.R_bi()

        tests.append(R_bi[0, 0] == pytest.approx(1))
        tests.append(R_bi[0, 1] == pytest.approx(0))
        tests.append(R_bi[0, 2] == pytest.approx(0))
        tests.append(R_bi[1, 0] == pytest.approx(0))
        tests.append(R_bi[1, 1] == pytest.approx(np.sqrt(2) / 2))
        tests.append(R_bi[1, 2] == pytest.approx(np.sqrt(2) / 2))

        assert np.all(tests)

    def test_yaw_pitch_roll(self):
        roll, pitch, yaw = np.deg2rad([45, 45, 45])
        euler = math.EulerAngles(roll, pitch, yaw)
        R_bi = euler.R_bi()
        euler_2 = math.EulerAngles.from_rotmat(R_bi)
        tests = []
        tests.append(euler_2.roll == pytest.approx(roll))
        tests.append(euler_2.pitch == pytest.approx(pitch))
        tests.append(euler_2.yaw == pytest.approx(yaw))

        assert np.all(tests)

    def test_yaw_pitch_roll_2(self):
        roll, pitch, yaw = np.deg2rad([90, 89, 90])
        euler = math.EulerAngles(roll, pitch, yaw)
        R_bi = euler.R_bi()
        euler_2 = math.EulerAngles.from_rotmat(R_bi)
        tests = []
        tests.append(euler_2.roll == pytest.approx(roll))
        tests.append(euler_2.pitch == pytest.approx(pitch))
        tests.append(euler_2.yaw == pytest.approx(yaw))

        assert np.all(tests)

    def test_T_eb(self):
        """
        Test T_eb according to Small Unmanned Aircraft Theory and Practice, R. Beard and T. McLain, p.31.
        """
        roll, pitch, yaw = np.deg2rad([45, 45, 45])
        euler = math.EulerAngles(roll, pitch, yaw)
        T_eb = euler.T_eb()
        T_eb_ref = np.array(
            [
                [1, 0, -np.sin(pitch)],
                [0, np.cos(roll), np.sin(roll) * np.cos(pitch)],
                [0, -np.sin(roll), np.cos(roll) * np.cos(pitch)],
            ]
        )

        assert np.array_equal(T_eb, T_eb_ref)

    def test_T_be(self):
        """
        Test T_be according to Small Unmanned Aircraft Theory and Practice, R. Beard and T. McLain, p.31.
        """
        roll, pitch, yaw = np.deg2rad([45, 45, 45])
        euler = math.EulerAngles(roll, pitch, yaw)
        T_be = euler.T_be()
        T_be_ref = np.array(
            [
                [1, np.sin(roll) * np.tan(pitch), np.cos(roll) * np.tan(pitch)],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll) / np.cos(pitch), np.cos(roll) / np.cos(pitch)],
            ]
        )

        assert np.allclose(T_be, T_be_ref)

    def test_individual_matrices(self):
        roll, pitch, yaw = np.deg2rad([45, 45, 45])
        euler = math.EulerAngles(roll, pitch, yaw)
        R = euler.R_ib()
        R_roll = euler.R_roll()
        R_pitch = euler.R_pitch()
        R_yaw = euler.R_yaw()
        R_combined = R_roll @ R_pitch @ R_yaw

        assert np.allclose(R, R_combined)


class TestQuaternion:
    def test_constructor(self):
        q = math.UnitQuaternion(1, 0, 0, 0)
        assert isinstance(q, math.UnitQuaternion)

    def test_constructor_rotmat(self):
        euler = math.EulerAngles(45, 45, 45)
        R_bi = euler.R_bi()
        q = math.UnitQuaternion.from_rotmat(R_bi)
        R_bi_2 = q.R_bi()
        assert np.allclose(R_bi, R_bi_2)

    # Temporarily unsupported
    # def test_constructor_from_two_vectors(self):
    #     v = np.sqrt(2) / 2
    #     v_1 = np.array([1, 0, 0])
    #     v_2 = np.array([0, 1, 0])
    #     q = math.UnitQuaternion.from_two_vectors(v_1, v_2)
    #     assert np.allclose(q.to_array(), [v, 0, 0, v])

    # Temporarily unsupported
    # def test_constructor_from_two_vectors_2(self):
    #     # TODO: This rotate-by-180-deg test could be more intuitive
    #     v = np.sqrt(2) / 2
    #     v_1 = np.array([1, 0, 0])
    #     v_2 = np.array([-1, 0, 0])
    #     q = math.UnitQuaternion.from_two_vectors(v_1, v_2)
    #     assert np.allclose(q.to_array(), [0, 0, -v, v])

    # Temporarily unsupported
    # def test_constructor_from_axisangle_px4(self):
    #     v = np.deg2rad(45)
    #     vec = np.array([1, 0, 0]) * v
    #     q = math.UnitQuaternion.from_axisangle_px4(vec)
    #     assert np.allclose(q.to_array(), [np.cos(v / 2), np.sin(v / 2), 0, 0])

    def test_normalize(self):
        """Ensure a UnitQuaternion always has norm of 1."""
        q = math.UnitQuaternion(2, 0, 0, 0)
        assert q.is_unit()

    def test_equals(self):
        q1 = math.UnitQuaternion()
        q2 = math.UnitQuaternion(1, 0, 0, 0)
        assert q1 == q2

    def test_equals_2(self):
        q1 = math.UnitQuaternion()
        q2 = math.UnitQuaternion(1, 1, 0, 0)
        assert q1 != q2

    def test_conjugate(self):
        q = math.UnitQuaternion(1, 1, 2, 3)
        q_star = q.conjugate()
        tests = []
        tests.append(q.real == pytest.approx(q_star.real))
        tests.append(np.all(q.imag == -q_star.imag))
        assert np.all(tests)

    def test_flipped(self):
        q = math.UnitQuaternion(1, 1, 2, 3)
        q_star = q.flipped()
        tests = []
        tests.append(q.real == pytest.approx(-q_star.real))
        tests.append(np.all(q.imag == q_star.imag))
        assert np.all(tests)

    def test_to_vector(self):
        v = np.sqrt(2) / 2
        q = math.UnitQuaternion(v, v, 0, 0)
        assert np.allclose(q.to_array(), [v, v, 0, 0])

    def test_to_euler(self):
        euler = math.EulerAngles(45, 45, 45, in_degrees=True)
        q = euler.to_quaternion()
        tests = [
            a == pytest.approx(b)
            for a, b in zip(euler.to_array(), q.to_euler().to_array())
        ]
        assert np.all(tests)

    def test_R_bi(self):
        euler = math.EulerAngles(90, 0, 0, in_degrees=True)
        R_e = euler.R_bi()
        v = np.sqrt(2) / 2
        q = math.UnitQuaternion(v, v, 0, 0)
        R_q = q.R_bi()

        vec = [1, 2, 3]
        prod_1 = R_e @ vec
        prod_2 = R_q @ vec
        assert np.allclose(prod_1, prod_2)

    def test_mul_bi(self):
        euler = math.EulerAngles(90, 0, 0, in_degrees=True)
        R_bi = euler.R_bi()
        q = euler.to_quaternion()

        vec = [1, 2, 3]
        prod_1 = q * vec
        prod_2 = R_bi @ vec
        assert np.allclose(prod_1, prod_2)

    def test_mul_ib(self):
        euler = math.EulerAngles(90, 0, 0, in_degrees=True)
        R_ib = euler.R_ib()
        q = euler.to_quaternion()

        vec = [1, 2, 3]
        prod_1 = q.conjugate() * vec
        prod_2 = R_ib @ vec
        assert np.allclose(prod_1, prod_2)

    def test_mul_two(self):
        """
        Verify the correct behaviour when multiplying two quaternions.

        The body frame is rotated from the inertial frame by q_b, with respect to the inertial frame.
        A link frame is rotated from the body frame by q_l with respect to the body frame.
        """
        euler_b = math.EulerAngles(90, 0, 0, in_degrees=True)
        q_b = euler_b.to_quaternion()
        euler_l = math.EulerAngles(0, 0, 90, in_degrees=True)
        q_l = euler_l.to_quaternion()

        vec_l = [1, 2, 3]
        vec_des = [-2, -3, 1]
        vec_b = q_b * q_l * vec_l
        assert np.allclose(vec_b, vec_des)

    def test_mul_numpy(self):
        # We rely on the C++ source to handle Eigen Matrix-Vector multiplication correcly w.r.t. dimensions.
        euler = math.EulerAngles(90, 0, 0, in_degrees=True)
        q = euler.to_quaternion()
        vec = np.array([1, 2, 3]).reshape((3,))
        prod = q.conjugate() * vec
        assert prod.shape == vec.shape

    # This multiplication is undefined. The test is for the warning.
    # def test_mul_scalar(self):
    #     q = math.UnitQuaternion()
    #     with pytest.raises(UserWarning):
    #         _ = 2 * q

    # Quaternion addition is undefined
    # def test_addition(self):
    #     v = np.sqrt(2) / 2
    #     q_1 = math.UnitQuaternion(v, v, 0, 0)
    #     q_2 = math.UnitQuaternion(v, 0, v, 0)
    #     q_sum = q_1 + q_2
    #     q_sum_ref = np.array(q_1.to_array() + q_2.to_array())
    #     q_sum_ref /= np.linalg.norm(q_sum_ref)
    #     assert np.allclose(q_sum.to_array(), q_sum_ref)

    # Quaternion subtraction is undefined
    # def test_subtraction(self):
    #     v = np.sqrt(2) / 2
    #     q_1 = math.UnitQuaternion(v, v, 0, 0)
    #     q_2 = math.UnitQuaternion(v, 0, v, 0)
    #     q_sum = q_1 - q_2
    #     q_sum_ref = np.array(q_1.to_array() - q_2.to_array())
    #     q_sum_ref /= np.linalg.norm(q_sum_ref)
    #     assert np.allclose(q_sum.to_array(), q_sum_ref)

    # Unit quaternion has always norm==1
    # def test_norm(self):
    #     v = np.sqrt(2) / 2
    #     vec = [v, v, 0, 0]
    #     q_1 = math.UnitQuaternion(*vec)
    #     assert q_1.norm == np.linalg.norm(vec)

    def test_iterator(self):
        v = np.sqrt(2) / 2
        vec = [v, v, 0, 0]
        q_1 = math.UnitQuaternion(*vec)
        tests = [a == pytest.approx(b) for a, b in zip(q_1.to_array(), vec)]
        assert np.all(tests)

    def test_q_dot(self):
        """
        Test quaternion derivative according to Aircraft Control and Simulation, Stevens and Lewis, p43
        """
        w = [1, 2, 3]
        p, q, r = w
        euler = math.EulerAngles(45, 45, 45, in_degrees=True)
        quat = euler.to_quaternion()
        q_dot = quat.q_dot(w)
        omega_q = np.array(
            [[0, p, q, r], [-p, 0, -r, q], [-q, r, 0, -p], [-r, -q, p, 0]]
        )
        q_dot_ref = -0.5 * omega_q @ quat.to_array()
        assert np.allclose(q_dot, q_dot_ref)

    def test_repr(self):
        v = np.sqrt(2) / 2
        vec = [v, v, 0, 0]
        q = math.UnitQuaternion(*vec)
        assert (
            repr(q)
            == f"UnitQuaternion({vec[0]:.3f}, {vec[1]:.3f}, {vec[2]:.0f}, {vec[3]:.0f})"
        )


class TestPose:
    def test_constructor(self):
        pose = math.Pose()
        tests = []
        tests.append(isinstance(pose, math.Pose))
        tests.append(np.all(pose.position == math.Vector3().to_array()))
        tests.append(pose.orientation == math.UnitQuaternion())
        assert np.all(tests)

    def test_transform_wrench(self):
        pose_bi = math.Pose(
            position=math.Vector3(1, 0, 0).to_array(),
            orientation=math.UnitQuaternion(1, 0, 0, 0),
        )
        wrench_b = math.Wrench(force=[0, 1, 0], torque=[0, 0, 0])

        result = math.Wrench(force=[0, 1, 0], torque=[0, 0, 1])
        result_expected = pose_bi @ wrench_b
        tests = []
        tests.append(np.allclose(result.force, result_expected.force))
        tests.append(np.allclose(result.torque, result_expected.torque))
        assert np.all(tests)

    def test_transform_wrench_2(self):
        pose_bi = math.Pose(
            position=math.Vector3(1, 0, 0).to_array(),
            orientation=math.UnitQuaternion(1, 0, 1, 0),
        )
        wrench_b = math.Wrench(force=[1, 0, 0], torque=[1, 0, 0])

        result = math.Wrench(force=[0, 0, -1], torque=[0, 1, -1])
        answer = result - pose_bi @ wrench_b
        assert np.allclose(answer.force, [0, 0, 0])
        assert np.allclose(answer.torque, [0, 0, 0])

    def test_inverse(self):
        pose_bi = math.Pose(
            position=math.Vector3(1, 0, 0).to_array(),
            orientation=math.UnitQuaternion(1, 0, 1, 0),
        )
        wrench_b = math.Wrench(force=[1, 0, 0], torque=[1, 0, 0])
        wrench_i = pose_bi @ wrench_b
        pose_ib = pose_bi.T
        wrench_b2 = pose_ib @ wrench_i
        tests = []
        tests.append(np.allclose(wrench_b.force, wrench_b2.force))
        tests.append(np.allclose(wrench_b.torque, wrench_b2.torque))

    def test_inverse_2(self):
        pose_bi = math.Pose(
            position=math.Vector3(1, 0, 0).to_array(),
            orientation=math.UnitQuaternion(1, 0, 0, 0),
        )
        wrench_b = math.Wrench(force=[1, 0, 0], torque=[0, 0, 0])
        wrench_i = pose_bi @ wrench_b
        pose_ib = pose_bi.T
        wrench_b2 = pose_ib @ wrench_i
        tests = []
        tests.append(np.allclose(wrench_b.force, wrench_b2.force))
        tests.append(np.allclose(wrench_b.torque, wrench_b2.torque))

        assert np.all(tests)
