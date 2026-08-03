#include <iostream>
#include <gtest/gtest.h>

#include "last_letter_lib/math_utils.hpp"
#include "yaml-cpp/yaml.h"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/kinematics.hpp"

#include "test_utils.hpp"

using namespace std;
using namespace Eigen;

TEST(TestKinematics, TestKinematics1)
{
    // Read parameter files
    auto config = load_config_aircraft("skywalker_2013");
    SimState_t state = build_aircraft_state_from_config(config);
    state.pose.position.z() = -10; // Lift the aircraft above the ground.
    state.pose.orientation = UnitQuaternion();

    Eigen::Vector3d inpForce(1, 0.1, 2);
    Eigen::Vector3d inpTorque(0.2, 0.1, 0.05);
    Wrench_t inpWrench;
    inpWrench.force = inpForce;
    inpWrench.torque = inpTorque;

    double mass = 2;
    double j_x, j_y, j_z, j_xz;
    j_x = 0.8244;
    j_y = 1.135;
    j_z = 1.759;
    j_xz = 0.1204;
    std::vector<double> J = {
        j_x, 0, -j_xz,
        0, j_y, 0,
        -j_xz, 0, j_z};
    Inertial inertial(mass, J);

    // Create kinematics object
    SimState_t newState;
    auto kinematics = Kinematics();
    auto kinematics_config = config.filter("kinematics/");
    kinematics_config.register_child_mngr(config.filter("world/")); // Point kinematics to the required world parameters.
    kinematics.initialize(kinematics_config);
    newState = kinematics.propagateState(state,inertial, inpWrench); // Use this method to calculate state integral

    EXPECT_GT(newState.pose.position.x(), state.pose.position.x()); // Initial velocity is positive.
    EXPECT_EQ(newState.pose.position.y(), state.pose.position.y());
    EXPECT_EQ(newState.pose.position.z(), state.pose.position.z());

    EXPECT_GT(newState.velocity.linear.x(), state.velocity.linear.x());
    EXPECT_GT(newState.velocity.linear.y(), state.velocity.linear.y());
    EXPECT_GT(newState.velocity.linear.z(), state.velocity.linear.z());

    auto euler = state.pose.orientation.to_euler();
    auto newEuler = newState.pose.orientation.to_euler();
    EXPECT_EQ(newEuler.roll, euler.roll);
    EXPECT_EQ(newEuler.pitch, euler.pitch);
    EXPECT_EQ(newEuler.yaw, euler.yaw);

    EXPECT_GT(newState.velocity.angular.x(), state.velocity.angular.x());
    EXPECT_GT(newState.velocity.angular.y(), state.velocity.angular.y());
    EXPECT_GT(newState.velocity.angular.z(), state.velocity.angular.z());
}
