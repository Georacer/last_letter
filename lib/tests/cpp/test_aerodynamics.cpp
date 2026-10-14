#include <iostream>
#include <gtest/gtest.h>

#include "yaml-cpp/yaml.h"
#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/aerodynamics.hpp"

#include "test_utils.hpp"

using namespace std;
using namespace Eigen;

using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib::math_utils;
using namespace last_letter_lib::aerodynamics;
using namespace last_letter_lib;

TEST(TestAerodynamics, TestAerodynamics1)
{
    auto config = load_config_aircraft("skywalker_2013");
    SimState_t state = build_aircraft_state_from_config(config);

    EnvironmentModel environmentModel = EnvironmentModel();
    environmentModel.initialize(config.filter("env"));
    environmentModel.calcEnvironment(state);

    Input input;
    input.value[0] = 0.1;
    input.value[1] = 0.1;
    input.value[2] = 0.5;
    input.value[3] = 0.0;

    auto airfoil1 = buildAerodynamics(config.filter("dynamics/aero/airfoil1/"));
    airfoil1->setInput(input);

    airfoil1->update_local_state(state, environmentModel.environment);
    airfoil1->calc_model(); // perform one step in the aerodynamics
    EXPECT_NEAR(environmentModel.environment.wind(1), -5, 1e-3);
    // cout << "Body-frame wind:\n"
    //      << environmentModel.environment.wind << endl;
    EXPECT_NEAR(airfoil1->airdata.airspeed, 11.180, 1e-3);
    EXPECT_NEAR(airfoil1->airdata.alpha, 0, 1e-3);
    EXPECT_NEAR(airfoil1->airdata.beta, 0.464, 1e-3);
    // cout << "Body-frame relative airdata:\n"
    //      << airfoil1->airspeed_ << "\n"
    //      << airfoil1->alpha_ << "\n"
    //      << airfoil1->beta_ << endl;
    EXPECT_TRUE(airfoil1->wrench_sum.wrenchAero.force(1) < 0);
    EXPECT_TRUE(airfoil1->wrench_sum.wrenchAero.force.norm() < 1000);
    // cout << "Aerodynamic force:\n"
    //      << airfoil1->wrenchAero.force << endl;
    EXPECT_TRUE(airfoil1->wrench_sum.wrenchAero.torque.norm() < 1000);
    // cout << "Aerodynamic torque:\n"
    //      << airfoil1->wrenchAero.torque << endl;
}

// Build an BluffBody with the ArduPilot quad values and step it once in
// the given body velocity, with no wind.
static void step_bluff_body(BluffBody &body, const Vector3d &linear, const Vector3d &angular)
{
    auto pm = ParameterManager("pm");
    pm.set<vector<double>>("area_cd", {0.1743, 0.1743, 0.1743}, false);
    pm.set<vector<double>>("rate_damping", {0.001337, 0.001337, 0.002674}, false);
    body.initialize(pm);

    SimState_t state;
    state.velocity.linear = linear;
    state.velocity.angular = angular;
    Environment_t environment;
    environment.density = 1.225;
    body.update_local_state(state, environment);
    body.calc_model();
}

TEST(TestAerodynamics, TestBluffBodyDrag)
{
    auto body = BluffBody("bluff_body");
    step_bluff_body(body, Vector3d(1, 0, 0), Vector3d::Zero());
    const Vector3d force = body.wrench_sum.wrenchAero.force;
    EXPECT_NEAR(force.x(), -0.1068, 0.01 * 0.1068);
    EXPECT_DOUBLE_EQ(force.y(), 0.0);
    EXPECT_DOUBLE_EQ(force.z(), 0.0);

    step_bluff_body(body, Vector3d(0, -2, 0), Vector3d::Zero());
    EXPECT_NEAR(body.wrench_sum.wrenchAero.force.y(), 0.427, 0.01 * 0.427);
}

TEST(TestAerodynamics, TestBluffBodyRateDamping)
{
    auto body = BluffBody("bluff_body");
    step_bluff_body(body, Vector3d::Zero(), Vector3d(0, 0, 1));
    const Vector3d torque = body.wrench_sum.wrenchAero.torque;
    EXPECT_DOUBLE_EQ(torque.x(), 0.0);
    EXPECT_DOUBLE_EQ(torque.y(), 0.0);
    EXPECT_DOUBLE_EQ(torque.z(), -0.002674);
}

TEST(TestAerodynamics, TestBluffBodyAtRest)
{
    auto body = BluffBody("bluff_body");
    step_bluff_body(body, Vector3d::Zero(), Vector3d::Zero());
    EXPECT_TRUE(body.wrench_sum.wrenchAero.force.isZero());
    EXPECT_TRUE(body.wrench_sum.wrenchAero.torque.isZero());
}
