#include <iostream>
#include <gtest/gtest.h>

#include "last_letter_lib/math_utils.hpp"
#include "yaml-cpp/yaml.h"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/uav_model.hpp"

#include "test_utils.hpp"

using namespace std;
using namespace Eigen;

using namespace last_letter_lib;

TEST(TestUavModel, TestInstantation)
{
    // Read parameter files
    auto config = load_config_aircraft("skywalker_2013");
    SimState_t state = build_aircraft_state_from_config(config);

    // Create model
    last_letter_lib::UavModel uavModel("skywalker_2013");
    uavModel.initialize(config);

    ASSERT_TRUE(true);
}

TEST(TestUavModel, TestRoll)
{
    auto config = load_config_aircraft("skywalker_2013");
    double deltaT = 0.01;
    config.set("world/deltaT", deltaT);
    config.set("env/windRef", 0);
    last_letter_lib::UavModel aircraft("skywalker_2013");
    aircraft.initialize(config);

    SimState_t state = build_aircraft_state_from_config(config);
    double vel = 15;
    state.velocity.linear.x() = vel;
    auto euler = EulerAngles(0, -10, 0, true);
    state.pose.orientation = UnitQuaternion(euler);
    aircraft.state = state;
    auto z_init = state.pose.position.z();

    auto u = Input({0.5});
    aircraft.setInput(u);
    double t = 0;
    double t_end = 1;
    while (t < t_end) {
        aircraft.step();
        // auto att = aircraft.state.pose.orientation.to_euler();
        // printf("t=%.2f  pos=[%.2f %.2f %.2f]  vel=[%.2f %.2f %.2f]  rpy=[%.1f %.1f %.1f]  pqr=[%.3f %.3f %.3f]\n",
        //     t,
        //     aircraft.state.pose.position.x(), aircraft.state.pose.position.y(), aircraft.state.pose.position.z(),
        //     aircraft.state.velocity.linear.x(), aircraft.state.velocity.linear.y(), aircraft.state.velocity.linear.z(),
        //     rad_to_deg(att.roll), rad_to_deg(att.pitch), rad_to_deg(att.yaw),
        //     aircraft.state.velocity.angular.x(), aircraft.state.velocity.angular.y(), aircraft.state.velocity.angular.z());
        t += deltaT;
    }

    // Position is reasonable.
    ASSERT_GT(aircraft.state.pose.position.x(), vel*t_end/2);
    ASSERT_GT(aircraft.state.pose.position.y(), 0);
    ASSERT_GT(aircraft.state.pose.position.z(), z_init);

    // Angles are reasonable.
    auto attitude = aircraft.state.pose.orientation.to_euler();
    ASSERT_LT(rad_to_deg(attitude.pitch), 0);
    ASSERT_GT(rad_to_deg(attitude.roll), 0);
    ASSERT_LT(rad_to_deg(attitude.roll), 60);
    ASSERT_GT(rad_to_deg(attitude.yaw), 0);
    ASSERT_LT(rad_to_deg(attitude.yaw), 30);

    // Velocity is reasonable.
    ASSERT_GT(aircraft.state.velocity.linear.x(), vel/2);
    ASSERT_LT(fabs(aircraft.state.velocity.linear.y()), 2);
    ASSERT_LT(fabs(aircraft.state.velocity.linear.z()), 1);

    // Angular velocity is reasonable.
    ASSERT_GT(aircraft.state.velocity.angular.x(), 0);
    ASSERT_LT(fabs(aircraft.state.velocity.angular.y()), 0.5);
    ASSERT_LT(fabs(aircraft.state.velocity.angular.z()), 0.5);
}

//////////////////////////////////////////////////////////////////////////////
// The ardupilot_copter recipe, a copy of the ArduPilot SITL default quad frame.

// Load the copter recipe with no wind, so the body drag stays out of the way.
static void initialize_copter(last_letter_lib::UavModel &aircraft)
{
    auto config = load_config_aircraft("ardupilot_copter");
    config.set("env/windRef", 0.0);
    aircraft.initialize(config);
    aircraft.init();
}

// Step the copter at a fixed state high above the ground, so the slew limit
// completes while the state stays put and the ground model has no contact.
static void step_copter_in_place(last_letter_lib::UavModel &aircraft, const Input &input)
{
    aircraft.state.pose.position.z() = -10.0;
    const SimState_t held_state = aircraft.state;
    aircraft.setInput(input);
    for (int i = 0; i < 20; i++)
    {
        aircraft.state = held_state;
        aircraft.step();
    }
}

TEST(TestUavModel, TestCopterRest)
{
    last_letter_lib::UavModel aircraft("ardupilot_copter");
    initialize_copter(aircraft);
    const double dt = aircraft.get_param<double>("world/deltaT");

    // The legs need a moment to settle, so the position reference is taken at 0.5 s.
    Vector3d settled_position = aircraft.state.pose.position;
    for (double t = 0; t < 2.0; t += dt)
    {
        aircraft.step();
        if (t < 0.5)
        {
            settled_position = aircraft.state.pose.position;
        }
        else
        {
            EXPECT_LT((aircraft.state.pose.position - settled_position).norm(), 0.01);
        }
        auto attitude = aircraft.state.pose.orientation.to_euler();
        ASSERT_LT(fabs(rad_to_deg(attitude.roll)), 0.5);
        ASSERT_LT(fabs(rad_to_deg(attitude.pitch)), 0.5);
    }
}

// Find the command that holds altitude at the current air density by bisection
// on the net vertical force. The four motors must then carry the weight equally.
TEST(TestUavModel, TestCopterHover)
{
    last_letter_lib::UavModel aircraft("ardupilot_copter");
    initialize_copter(aircraft);

    auto net_force_z = [&aircraft](double d) {
        step_copter_in_place(aircraft, Input({d, d, d, d}));
        return aircraft.dynamics.wrench_sum.sum().force.z();
    };

    // The net force falls as the command rises, so its sign drives the bisection.
    double d_low = 0.0;
    double d_high = 1.0;
    ASSERT_GT(net_force_z(d_low), 0.0); // The copter falls with no command.
    ASSERT_LT(net_force_z(d_high), 0.0); // The full command lifts it.
    for (int i = 0; i < 40; i++)
    {
        const double d_mid = 0.5 * (d_low + d_high);
        if (net_force_z(d_mid) > 0.0)
        {
            d_low = d_mid;
        }
        else
        {
            d_high = d_mid;
        }
    }
    const double d_hover = 0.5 * (d_low + d_high);
    const double force_z = net_force_z(d_hover);

    const double weight = aircraft.dynamics.wrench_sum.wrenchGrav.force.z();
    EXPECT_NEAR(force_z, 0.0, 1e-6 * weight);
    for (auto thruster : aircraft.dynamics.thrusters)
    {
        EXPECT_NEAR(thruster->wrench_sum.wrenchProp.force.x(), weight / 4, 1e-3 * weight);
    }
    std::cout << "copter hover: rho " << aircraft.environmentModel.environment.density
              << ", command " << d_hover << ", thrust per motor "
              << aircraft.dynamics.thrusters[0]->wrench_sum.wrenchProp.force.x()
              << " N, weight " << weight << " N" << std::endl;
}

TEST(TestUavModel, TestCopterTorqueSigns)
{
    last_letter_lib::UavModel aircraft("ardupilot_copter");
    initialize_copter(aircraft);
    const SimState_t initial_state = aircraft.state;

    // The right motor above the left one rolls the copter to the left.
    step_copter_in_place(aircraft, Input({0.7, 0.4, 0.0, 0.0}));
    const double torque_x = aircraft.dynamics.wrench_sum.wrenchProp.torque.x();
    EXPECT_LT(torque_x, 0.0);

    // The front motor above the rear one pitches the nose up.
    aircraft.state = initial_state;
    step_copter_in_place(aircraft, Input({0.0, 0.0, 0.7, 0.4}));
    const double torque_y = aircraft.dynamics.wrench_sum.wrenchProp.torque.y();
    EXPECT_GT(torque_y, 0.0);

    // The CCW motors above the CW ones yaw the nose right.
    aircraft.state = initial_state;
    step_copter_in_place(aircraft, Input({0.7, 0.7, 0.4, 0.4}));
    const double torque_z = aircraft.dynamics.wrench_sum.wrenchProp.torque.z();
    EXPECT_GT(torque_z, 0.0);

    std::cout << "copter torques: roll " << torque_x << ", pitch " << torque_y << ", yaw " << torque_z
              << " N m" << std::endl;
}
