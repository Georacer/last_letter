#include <iostream>
#include <gtest/gtest.h>

#include "yaml-cpp/yaml.h"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/propulsion/propulsion.hpp"
#include "last_letter_lib/environment.hpp"

#include "test_utils.hpp"

using namespace std;
using namespace Eigen;

using namespace last_letter_lib;
using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib::math_utils;
using namespace last_letter_lib::propulsion;
using namespace last_letter_lib::uav_utils;

TEST(TestPropulsion, TestThrusterSimple)
{
    auto thruster = ThrusterSimple("simple_thruster");
    auto pm = ParameterManager("pm");
    pm.set("chanMotor", 2, false);
    thruster.initialize(pm);
    auto input = Input({0, 0, 1, 0});

    SimState_t states;
    Environment_t environment;
    thruster.setInput(input);
    thruster.update_local_state(states, environment);
    thruster.calc_model();

    EXPECT_EQ(thruster.wrench_sum.wrenchProp.force.x(), thruster.get_param<double>("thrustMax"));
    EXPECT_EQ(thruster.wrench_sum.wrenchProp.torque.x(), -thruster.get_param<double>("torqueMax"));
}

TEST(TestPropulsion, TestOmegaControlledEngine)
{
    auto thruster = EngOmegaControl("omega_controlled_engine");
    auto pm = ParameterManager("pm");
    pm.set("chanMotor", 2, false);
    thruster.initialize(pm);
    auto input = Input({0, 0, 1, 0});

    SimState_t states;
    Environment_t environment;
    thruster.setInput(input);
    thruster.update_local_state(states, environment);
    thruster.calc_model();

    EXPECT_EQ(thruster.omega, thruster.get_param<double>("omega_max"));
    EXPECT_GT(thruster.wrench_sum.wrenchProp.force.x(), 0);
}

TEST(TestPropulsion, TestElectricEngine2)
{
    auto thruster = ElectricEng2("electric_engine_2");
    auto pm = ParameterManager("pm");
    pm.set("chanMotor", 2, false);
    thruster.initialize(pm);
    auto input = Input({0, 0, 1, 0});

    SimState_t states;
    Environment_t environment;
    thruster.setInput(input);
    thruster.update_local_state(states, environment);
    for (uint16_t idx=0; idx<1000; idx++) {
        thruster.calc_model();
    }

    EXPECT_GT(thruster.omega, 0);
    EXPECT_GT(thruster.wrench_sum.wrenchProp.force.x(), 0);
}

TEST(TestPropulsion, TestPropulsion1)
{
    ParameterManager config = load_config_aircraft("skywalker_2013");
    auto prop_config = config.filter("dynamics/prop/motor1/");
    prop_config.register_child_mngr(config.filter("world/"));
    auto motor1 = buildThruster(prop_config);
    SimState_t states = build_aircraft_state_from_config(config);

    Input input;
    input.value[2] = 0.5;

    EnvironmentModel environmentModel = EnvironmentModel();
    environmentModel.initialize(config.filter("env"));
    environmentModel.calcEnvironment(states);

    motor1->setInput(input);
    motor1->update_local_state(states, environmentModel.environment); // perform one step in the propdynamics
    motor1->calc_model();
    EXPECT_GT(motor1->omega, 0);
    EXPECT_GT(motor1->wrench_sum.wrenchProp.force(0), 0);
    EXPECT_EQ(motor1->wrench_sum.wrenchProp.force(1), 0);
    EXPECT_EQ(motor1->wrench_sum.wrenchProp.force(2), 0);
    EXPECT_LT(motor1->wrench_sum.wrenchProp.torque(0), 0);
    EXPECT_EQ(motor1->wrench_sum.wrenchProp.torque(1), 0);
    EXPECT_EQ(motor1->wrench_sum.wrenchProp.torque(2), 0);
}
