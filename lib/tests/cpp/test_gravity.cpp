#include <iostream>
#include <gtest/gtest.h>
#include "Eigen/Eigen"

#include "last_letter_lib/gravity.hpp"
#include "last_letter_lib/math_utils.hpp"

using namespace std;
using namespace Eigen;

using namespace last_letter_lib::math_utils;
using namespace last_letter_lib::uav_utils;
using namespace last_letter_lib;

TEST(TestGravity, TestGravitySimple)
{
    double g{9.81};
    Vector3d euler(M_PI / 4, M_PI / 4, 0);
    // Vector3d euler(M_PI/2, 0, 0);
    // Vector3d euler(0, M_PI/2, 0);
    // Vector3d euler(0, 0, M_PI/2);

    // cout << "Starting with Euler angles (RPY):\n"
    //  << euler * 180 / M_PI << endl;
    // cout << "Desired transformations are from Earth-frame to Body-frame, i.e. v_b = R*v_e" << endl;

    // cout << "Generating a quaternion q_eb from said Euler angles\n";
    Quaterniond orientation_eb = euler2quat(euler);
    SimState_t states;
    states.pose.orientation = orientation_eb.conjugate();
    Inertial inertial;
    inertial.mass = 1;

    GravitySimple gravityModel;
    auto wrench = gravityModel.getWrench(states, inertial);

    EXPECT_NEAR(wrench.force(0), -g * sqrt(2) / 2, 1e-3);
    EXPECT_NEAR(wrench.force(1), g / 2, 1e-3);
    EXPECT_NEAR(wrench.force(2), g / 2, 1e-3);
    EXPECT_EQ(wrench.force.norm(), g);
    EXPECT_EQ(wrench.torque.norm(), 0);
}

TEST(TestGravity, TestGravityClassic)
{
    double g{9.78};
    Vector3d euler(M_PI / 4, M_PI / 4, 0);
    // Vector3d euler(M_PI/2, 0, 0);
    // Vector3d euler(0, M_PI/2, 0);
    // Vector3d euler(0, 0, M_PI/2);

    // cout << "Starting with Euler angles (RPY):\n"
    //  << euler * 180 / M_PI << endl;
    // cout << "Desired transformations are from Earth-frame to Body-frame, i.e. v_b = R*v_e" << endl;

    // cout << "Generating a quaternion q_eb from said Euler angles\n";
    Quaterniond orientation_eb = euler2quat(euler);
    SimState_t states;
    states.pose.orientation = orientation_eb.conjugate();
    Inertial inertial;
    inertial.mass = 1;

    GravityClassic gravityModel;
    auto wrench = gravityModel.getWrench(states, inertial);

    EXPECT_NEAR(wrench.force(0), -g * sqrt(2) / 2, 1e-3);
    EXPECT_NEAR(wrench.force(1), g / 2, 1e-3);
    EXPECT_NEAR(wrench.force(2), g / 2, 1e-3);
    EXPECT_NEAR(wrench.force.norm(), g, 1e-3);
    EXPECT_EQ(wrench.torque.norm(), 0);
}
