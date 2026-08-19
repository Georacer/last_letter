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

// The PWM path is what the ArduPilot bridge drives the model with, and it is
// the only input path with no other coverage. 1500 us is mid-stick, which
// PwmToHalfRange maps to half throttle.
TEST(TestPropulsion, TestThrusterSetInputPwm)
{
    auto thruster = ThrusterSimple("pwm_thruster");
    auto pm = ParameterManager("pm");
    pm.set("chanMotor", 2, false);
    thruster.initialize(pm);

    SimState_t states;
    Environment_t environment;
    thruster.update_local_state(states, environment);

    InputPwm_t input; // All channels at 1000 us, i.e. zero throttle.
    input.value[2] = 1500;
    thruster.setInputPwm(input);
    thruster.calc_model();

    const double thrust_max = thruster.get_param<double>("thrustMax");
    const double thrust_min = thruster.get_param<double>("thrustMin");
    EXPECT_DOUBLE_EQ(thruster.wrench_sum.wrenchProp.force.x(),
                     0.5 * (thrust_max - thrust_min) + thrust_min);

    // A full-scale PWM must match what the normalized setter does with 1.0.
    input.value[2] = 2000;
    thruster.setInputPwm(input);
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.wrench_sum.wrenchProp.force.x(), thrust_max);

    // An unwired channel leaves the throttle where it was.
    thruster.chanMotor = -1;
    input.value[2] = 1000;
    thruster.setInputPwm(input);
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.wrench_sum.wrenchProp.force.x(), thrust_max);
}

// Nothing downstream of set_throttle range-checks the command: PistonEng and the
// electric engines scale engine power by inputMotor linearly, so an out-of-range
// throttle just pins the propeller at its RadPSLimits ceiling and looks plausible.
// Both setters funnel through set_throttle, so clamping there covers every path.
TEST(TestPropulsion, TestThrottleIsConstrained)
{
    auto thruster = ThrusterSimple("clamped_thruster");
    auto pm = ParameterManager("pm");
    pm.set("chanMotor", 0, false);
    thruster.initialize(pm);

    thruster.set_throttle(10.0);
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 1.0);

    thruster.set_throttle(-10.0);
    EXPECT_DOUBLE_EQ(thruster.inputMotor, -1.0);

    // In-range commands, including reverse thrust, pass through untouched.
    thruster.set_throttle(0.25);
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 0.25);
    thruster.set_throttle(-0.5);
    EXPECT_DOUBLE_EQ(thruster.inputMotor, -0.5);

    // The channel setter funnels through it too.
    thruster.setInput(Input({7.0, 0, 0, 0}));
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 1.0);

    // As does the PWM one: 3000 us is well outside the 1000-2000 us band.
    InputPwm_t input;
    input.value[0] = 3000;
    thruster.setInputPwm(input);
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 1.0);
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
