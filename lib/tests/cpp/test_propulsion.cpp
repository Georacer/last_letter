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
// The command reaches inputMotor at the next step, so each check steps once.
TEST(TestPropulsion, TestThrottleIsConstrained)
{
    auto thruster = ThrusterSimple("clamped_thruster");
    auto pm = ParameterManager("pm");
    pm.set("chanMotor", 0, false);
    thruster.initialize(pm);

    SimState_t states;
    Environment_t environment;
    thruster.update_local_state(states, environment);

    thruster.set_throttle(10.0);
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 1.0);

    thruster.set_throttle(-10.0);
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.inputMotor, -1.0);

    // In-range commands, including reverse thrust, pass through untouched.
    thruster.set_throttle(0.25);
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 0.25);
    thruster.set_throttle(-0.5);
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.inputMotor, -0.5);

    // The channel setter funnels through it too.
    thruster.setInput(Input({7.0, 0, 0, 0}));
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 1.0);

    // As does the PWM one: 3000 us is well outside the 1000-2000 us band.
    InputPwm_t input;
    input.value[0] = 3000;
    thruster.setInputPwm(input);
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 1.0);
}

// ArduPilot limits how fast a motor command may change, and has no motor lag
// besides. At 150 per second and 1200 Hz a full step takes eight frames.
TEST(TestPropulsion, TestThrusterSlewLimit)
{
    auto thruster = ThrusterSimple("slew_thruster");
    auto pm = ParameterManager("pm");
    pm.set("chanMotor", 0, false);
    pm.set("slew_max", 150.0, false);
    thruster.initialize(pm);
    thruster.set_dt(1.0 / 1200);

    SimState_t states;
    Environment_t environment;
    thruster.update_local_state(states, environment);

    thruster.set_throttle(1.0);
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 0.0); // The setter alone does not move it.
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 0.125);
    for (int i = 1; i < 8; i++)
    {
        thruster.calc_model();
    }
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 1.0);
}

// The PWM endpoints stand in for the ArduPilot MOT_PWM_MIN, MOT_PWM_MAX,
// spin_min and spin_max chain, which is one line clipped to 0 to 1.
TEST(TestPropulsion, TestThrusterPwmEndpoints)
{
    auto thruster = ThrusterSimple("pwm_endpoint_thruster");
    auto pm = ParameterManager("pm");
    pm.set("chanMotor", 0, false);
    pm.set("pwm_min", 1150.0, false);
    pm.set("pwm_max", 1950.0, false);
    thruster.initialize(pm);

    SimState_t states;
    Environment_t environment;
    thruster.update_local_state(states, environment);

    InputPwm_t input;
    input.value[0] = 1150;
    thruster.setInputPwm(input);
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 0.0);
    input.value[0] = 1950;
    thruster.setInputPwm(input);
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 1.0);
    input.value[0] = 1550;
    thruster.setInputPwm(input);
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 0.5);
    input.value[0] = 1000;
    thruster.setInputPwm(input);
    thruster.calc_model();
    EXPECT_DOUBLE_EQ(thruster.inputMotor, 0.0);

    // The defaults keep the old 1000 to 2000 us mapping.
    auto default_thruster = ThrusterSimple("pwm_default_thruster");
    auto default_pm = ParameterManager("pm");
    default_pm.set("chanMotor", 0, false);
    default_thruster.initialize(default_pm);
    default_thruster.update_local_state(states, environment);
    input.value[0] = 1500;
    default_thruster.setInputPwm(input);
    default_thruster.calc_model();
    EXPECT_DOUBLE_EQ(default_thruster.inputMotor, 0.5);
}

TEST(TestPropulsion, TestThrusterPwmEndpointsMustBeOrdered)
{
    auto thruster = ThrusterSimple("pwm_bad_thruster");
    auto pm = ParameterManager("pm");
    pm.set("pwm_min", 1500.0, false);
    pm.set("pwm_max", 1500.0, false);
    EXPECT_THROW(thruster.initialize(pm), std::runtime_error);
}

// Build a full-throttle ThrusterSimple with the given momentum drag settings
// and step it once in the given relative wind, which enters as the body velocity.
static ThrusterSimple step_thruster_with_momentum_drag(ParameterManager pm, const Vector3d &relative_wind,
                                                       double density = 1.225)
{
    auto thruster = ThrusterSimple("momentum_drag_thruster");
    pm.set("chanMotor", 0, false);
    thruster.initialize(pm);

    SimState_t states;
    states.velocity.linear = relative_wind;
    Environment_t environment;
    environment.density = density;
    thruster.set_throttle(1.0);
    thruster.update_local_state(states, environment);
    thruster.calc_model();
    return thruster;
}

// The linear law gives -0.01 * 20 N * 2 m/s of lateral wind, which is -0.4 N on y.
// The axial switch is off by default, so the thrust on x stays at 20 N.
TEST(TestPropulsion, TestMomentumDragLinear)
{
    auto pm = ParameterManager("pm");
    pm.set("thrustMax", 20.0, false);
    pm.set("momentum_drag/type", 1, false);
    pm.set("momentum_drag/coeff", 0.01, false);
    auto thruster = step_thruster_with_momentum_drag(pm, Vector3d(0, 2, 0));

    EXPECT_NEAR(thruster.wrench_sum.wrenchProp.force.y(), -0.4, 1e-12);
    EXPECT_DOUBLE_EQ(thruster.wrench_sum.wrenchProp.force.x(), 20.0);
}

// The ArduPilot disc law at the copter hover point: sqrt(1.1568 * 0.09625 * 7.355)
// is 0.905, times mdrag 0.2 per m/s of relative wind.
TEST(TestPropulsion, TestMomentumDragDisc)
{
    auto pm = ParameterManager("pm");
    pm.set("thrustMax", 7.355, false);
    pm.set("momentum_drag/type", 2, false);
    pm.set("momentum_drag/coeff", 0.2, false);
    pm.set("momentum_drag/disc_area", 0.09625, false);
    pm.set("momentum_drag/axial", true, false);
    auto thruster = step_thruster_with_momentum_drag(pm, Vector3d(1, 1, 0), 1.1568);

    EXPECT_NEAR(thruster.wrench_sum.wrenchProp.force.x(), 7.355 - 0.181, 0.02 * (7.355 - 0.181));
    EXPECT_NEAR(thruster.wrench_sum.wrenchProp.force.y(), -0.181, 0.02 * 0.181);
}

// This is the disc case above with the axial switch off. The axial wind then
// adds no drag, so the thrust on x stays at exactly 7.355 N.
TEST(TestPropulsion, TestMomentumDragDiscNotAxial)
{
    auto pm = ParameterManager("pm");
    pm.set("thrustMax", 7.355, false);
    pm.set("momentum_drag/type", 2, false);
    pm.set("momentum_drag/coeff", 0.2, false);
    pm.set("momentum_drag/disc_area", 0.09625, false);
    pm.set("momentum_drag/axial", false, false);
    auto thruster = step_thruster_with_momentum_drag(pm, Vector3d(1, 1, 0), 1.1568);

    EXPECT_DOUBLE_EQ(thruster.wrench_sum.wrenchProp.force.x(), 7.355);
}

// Type 0 is the default and applies no drag. A nonzero coeff and the axial
// switch do not change that, so the force is the pure thrust.
TEST(TestPropulsion, TestMomentumDragNone)
{
    auto pm = ParameterManager("pm");
    pm.set("thrustMax", 20.0, false);
    pm.set("momentum_drag/coeff", 0.01, false);
    pm.set("momentum_drag/axial", true, false);
    auto thruster = step_thruster_with_momentum_drag(pm, Vector3d(3, -2, 1));

    EXPECT_DOUBLE_EQ(thruster.wrench_sum.wrenchProp.force.x(), 20.0);
    EXPECT_DOUBLE_EQ(thruster.wrench_sum.wrenchProp.force.y(), 0.0);
    EXPECT_DOUBLE_EQ(thruster.wrench_sum.wrenchProp.force.z(), 0.0);
}

// The drag is added to the wrench of the step, so it must not pile up across steps.
TEST(TestPropulsion, TestMomentumDragDoesNotAccumulate)
{
    auto thruster = ThrusterSimple("steady_drag_thruster");
    auto pm = ParameterManager("pm");
    pm.set("chanMotor", 0, false);
    pm.set("momentum_drag/type", 1, false);
    pm.set("momentum_drag/coeff", 0.01, false);
    thruster.initialize(pm);

    SimState_t states;
    states.velocity.linear = Vector3d(0, 2, 0);
    Environment_t environment;
    thruster.set_throttle(1.0);
    thruster.update_local_state(states, environment);

    thruster.calc_model();
    const double first_force_y = thruster.wrench_sum.wrenchProp.force.y();
    for (int i = 1; i < 10; i++)
    {
        thruster.calc_model();
    }
    EXPECT_NE(first_force_y, 0.0);
    EXPECT_DOUBLE_EQ(thruster.wrench_sum.wrenchProp.force.y(), first_force_y);
}

// ElectricEng2 selects the linear law by default, with the coefficient it
// used to carry as momentumDragCoeff.
TEST(TestPropulsion, TestElectricEngine2MomentumDrag)
{
    auto thruster = ElectricEng2("electric_engine_2_drag");
    auto pm = ParameterManager("pm");
    pm.set("chanMotor", 2, false);
    thruster.initialize(pm);

    SimState_t states;
    states.velocity.linear = Vector3d(0, 2, 0);
    Environment_t environment;
    thruster.setInput(Input({0, 0, 1, 0}));
    thruster.update_local_state(states, environment);
    for (int idx = 0; idx < 1000; idx++)
    {
        thruster.calc_model();
    }

    const Vector3d force = thruster.wrench_sum.wrenchProp.force;
    const double expected = -0.0047 * force.x() * 2;
    EXPECT_GT(force.x(), 0);
    EXPECT_NEAR(force.y(), expected, 0.01 * std::fabs(expected));
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
