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
