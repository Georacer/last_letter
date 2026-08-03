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
