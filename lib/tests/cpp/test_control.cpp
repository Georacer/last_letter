#include <gtest/gtest.h>

#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "Eigen/Eigen"

#include "last_letter_lib/body/linear_copter.hpp"
#include "last_letter_lib/body/ardupilot_mixer.hpp"
#include "last_letter_lib/control/control_map.hpp"
#include "last_letter_lib/control/evaluate.hpp"
#include "last_letter_lib/control/trim.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/systems.hpp"
#include "last_letter_lib/uav_model.hpp"

#include "test_utils.hpp"

using namespace Eigen;

using namespace last_letter_lib;
using namespace last_letter_lib::control;

// Load a recipe with no wind and, unless asked, no ground reaction.
static void initialize_aircraft(UavModel &aircraft, const std::string &name, bool keep_ground = false)
{
    auto config = load_config_aircraft(name);
    config.set("env/windRef", 0.0);
    if (!keep_ground)
    {
        config.set("dynamics/ground/groundReactionType", 0);
    }
    aircraft.initialize(config);
}

// A state well clear of the ground, at rest.
static SimState_t hover_state(const UavModel &aircraft)
{
    SimState_t state = aircraft.state;
    state.pose.position = Vector3d(0.0, 0.0, -10.0);
    state.velocity.linear = Vector3d::Zero();
    state.velocity.angular = Vector3d::Zero();
    return state;
}

// A component whose wrench grows by one at every call, so it never settles.
class NeverSettles : public systems::Component
{
public:
    NeverSettles() : Component("never_settles") {}

protected:
    void calc_model_impl() override
    {
        Component::calc_model_impl();
        calls_ += 1.0;
        wrench_sum.wrenchAero.force = Vector3d(calls_, 0.0, 0.0);
    }

private:
    double calls_{0.0};
};

///////////////////////////////////////////////////////////////////////////////
// The settle loop

// The EngBeard slew of the copter recipe moves 0.375 per step, so a step of
// the command up to that size arrives in one call and the next call confirms it.
TEST(TestControl, SettleEngBeard)
{
    UavModel aircraft("ardupilot_copter");
    initialize_aircraft(aircraft, "ardupilot_copter");
    const SimState_t state = hover_state(aircraft);

    // From rest the command climbs 0.5 in two calls.
    const Evaluation first = evaluate(aircraft, state, Input({0.5, 0.5, 0.5, 0.5}));
    EXPECT_LE(first.iterations, 3);

    const Evaluation second = evaluate(aircraft, state, Input({0.6, 0.6, 0.6, 0.6}));
    EXPECT_LE(second.iterations, 3);
    for (auto thruster : aircraft.dynamics.thrusters)
    {
        EXPECT_DOUBLE_EQ(thruster->inputMotor, 0.6);
    }

    // The same input again settles at once.
    const Evaluation again = evaluate(aircraft, state, Input({0.6, 0.6, 0.6, 0.6}));
    EXPECT_EQ(again.iterations, 2);
    EXPECT_TRUE(again.wrench.force.isApprox(second.wrench.force, 1e-12));
    std::cout << "settle calls: " << first.iterations << " from rest, " << second.iterations
              << " after a step, " << again.iterations << " at the same input" << std::endl;
}

// The settle loop holds the rigid body state.
TEST(TestControl, SettleHoldsState)
{
    UavModel aircraft("ardupilot_copter");
    initialize_aircraft(aircraft, "ardupilot_copter");
    SimState_t state = hover_state(aircraft);
    state.velocity.linear = Vector3d(1.0, -0.5, 0.2);
    state.velocity.angular = Vector3d(0.1, -0.2, 0.05);

    const double time = aircraft.get_time();
    evaluate(aircraft, state, Input({0.5, 0.5, 0.5, 0.5}));
    EXPECT_EQ(aircraft.state.pose.position, state.pose.position);
    EXPECT_EQ(aircraft.state.velocity.linear, state.velocity.linear);
    EXPECT_EQ(aircraft.state.velocity.angular, state.velocity.angular);
    EXPECT_EQ(aircraft.get_time(), time);
}

// A component that never settles gives an error at the cap.
TEST(TestControl, SettleCapThrows)
{
    UavModel aircraft("ardupilot_copter");
    initialize_aircraft(aircraft, "ardupilot_copter");
    auto mock = std::make_unique<NeverSettles>();
    mock->initialize();
    aircraft.dynamics.components.push_back(std::move(mock));

    SettleOptions options;
    options.max_iterations = 50;
    EXPECT_THROW(evaluate(aircraft, hover_state(aircraft), Input({0.5, 0.5, 0.5, 0.5}), options),
                 std::runtime_error);
}

// The linear copter at its hover throttle: the full wrench is zero, the model
// wrench is the trim thrust, and the derivatives are zero.
TEST(TestControl, SettleLinearCopterHover)
{
    UavModel aircraft("ardupilot_copter_linear");
    initialize_aircraft(aircraft, "ardupilot_copter_linear");
    const auto &copter = dynamic_cast<const body::LinearCopter &>(*aircraft.dynamics.bodies.at(0));
    const double u_hover = copter.u_hover;

    const Evaluation result = evaluate(aircraft, hover_state(aircraft), Input({0.0, 0.0, 0.0, u_hover}));
    const double weight = result.inertial.mass * 9.81;
    EXPECT_LT(result.wrench.force.norm(), 1e-9 * weight);
    EXPECT_LT(result.wrench.torque.norm(), 1e-9);
    EXPECT_NEAR(result.wrench_model.force.z(), -weight, 1e-9 * weight);
    EXPECT_LT(result.wrench_model.force.head<2>().norm(), 1e-9);
    EXPECT_LT(result.wrench_model.torque.norm(), 1e-9);
    EXPECT_LT(result.state_dot.speedDot.norm(), 1e-9);
    EXPECT_LT(result.state_dot.rateDot.norm(), 1e-9);
    EXPECT_DOUBLE_EQ(result.inertial.mass, 3.0);
}

///////////////////////////////////////////////////////////////////////////////
// The ground reaction guard

TEST(TestControl, RequireNoGroundReaction)
{
    UavModel with_ground("ardupilot_copter");
    initialize_aircraft(with_ground, "ardupilot_copter", true);
    try
    {
        require_no_ground_reaction(with_ground);
        FAIL() << "A model with a ground reaction passed the guard.";
    }
    catch (const std::runtime_error &error)
    {
        EXPECT_NE(std::string(error.what()).find("dynamics/ground/groundReactionType"), std::string::npos);
    }

    UavModel without_ground("ardupilot_copter");
    initialize_aircraft(without_ground, "ardupilot_copter");
    EXPECT_NO_THROW(require_no_ground_reaction(without_ground));
}

///////////////////////////////////////////////////////////////////////////////
// The copter control map

static std::shared_ptr<body::MatrixMixer> make_mixer(const std::string &yaml)
{
    return std::shared_ptr<body::MatrixMixer>(body::buildMixer(ParameterManager::from_yaml_string("mixer", yaml)));
}

TEST(TestControl, CopterMapBounds)
{
    UavModel aircraft("ardupilot_copter");
    initialize_aircraft(aircraft, "ardupilot_copter");
    const CopterControlMap map(make_mixer("mixerType: 0"), aircraft);
    EXPECT_EQ(map.size(), 4);
    EXPECT_TRUE(map.lower_bounds().isApprox(Vector4d(-1.0, -1.0, -1.0, 0.0)));
    EXPECT_TRUE(map.upper_bounds().isApprox(Vector4d(1.0, 1.0, 1.0, 1.0)));
    EXPECT_THROW(map.to_input(VectorXd::Zero(3)), std::invalid_argument);
}

// With the base mixer the thrust fraction of each motor is its command.
TEST(TestControl, CopterMapMatrix)
{
    UavModel aircraft("ardupilot_copter");
    initialize_aircraft(aircraft, "ardupilot_copter");
    const auto mixer = make_mixer("mixerType: 0");
    const CopterControlMap map(mixer, aircraft);

    const Vector4d controls(0.1, -0.05, 0.02, 0.4);
    const Input input = map.to_input(controls);
    const VectorXd thrust = mixer->mix(controls);
    for (int i = 0; i < mixer->n_motors(); i++)
    {
        EXPECT_DOUBLE_EQ(input.value[mixer->channels()[i]], thrust(i));
    }
    for (size_t channel = mixer->n_motors(); channel < input.value.size(); channel++)
    {
        EXPECT_EQ(input.value[channel], 0.0);
    }
}

// With the ArduPilot mixer each thruster converts the PWM of its channel. The
// thruster band of the stock recipe equals the spin band, so the command is
// the linearized throttle of thrust_to_actuator.
TEST(TestControl, CopterMapArduPilot)
{
    UavModel aircraft("ardupilot_copter");
    initialize_aircraft(aircraft, "ardupilot_copter");
    const auto mixer = make_mixer("mixerType: 1");
    const CopterControlMap map(mixer, aircraft);

    const Vector4d controls(0.1, -0.05, 0.02, 0.4);
    const Input input = map.to_input(controls);
    const std::vector<double> pwm = mixer->mix_to_pwm(controls);
    ASSERT_EQ(aircraft.dynamics.thrusters.size(), 4u);
    for (const auto *thruster : aircraft.dynamics.thrusters)
    {
        const int channel = thruster->chanMotor;
        EXPECT_DOUBLE_EQ(input.value[channel], thruster->pwm_to_throttle(pwm[channel]));
        const double actuator = (pwm[channel] - 1000.0) / 1000.0;
        EXPECT_NEAR(input.value[channel], (actuator - 0.15) / 0.8, 1e-12);
    }
    // The inverse mixer recovers the controls from the same PWM.
    EXPECT_TRUE(mixer->invert_pwm(pwm).isApprox(controls, 1e-9));
}

///////////////////////////////////////////////////////////////////////////////
// The identity control map

// Each control goes to its own channel and every other channel stays at zero.
TEST(TestControl, IdentityMap)
{
    const IdentityControlMap map({0, 1, 2, 3}, Vector4d(-1.0, -1.0, -1.0, 0.0), Vector4d(1.0, 1.0, 1.0, 1.0));
    EXPECT_EQ(map.size(), 4);
    EXPECT_TRUE(map.lower_bounds().isApprox(Vector4d(-1.0, -1.0, -1.0, 0.0)));
    EXPECT_TRUE(map.upper_bounds().isApprox(Vector4d(1.0, 1.0, 1.0, 1.0)));

    const Input input = map.to_input(Vector4d(0.1, -0.2, 0.3, 0.4));
    EXPECT_EQ(input.value[0], 0.1);
    EXPECT_EQ(input.value[1], -0.2);
    EXPECT_EQ(input.value[2], 0.3);
    EXPECT_EQ(input.value[3], 0.4);
    for (size_t channel = 4; channel < input.value.size(); channel++)
    {
        EXPECT_EQ(input.value[channel], 0.0);
    }
    EXPECT_THROW(map.to_input(VectorXd::Zero(3)), std::invalid_argument);

    // The channels need not be in order or adjacent.
    const IdentityControlMap sparse({5, 2}, Vector2d(0.0, 0.0), Vector2d(1.0, 1.0));
    const Input sparse_input = sparse.to_input(Vector2d(0.7, 0.8));
    EXPECT_EQ(sparse_input.value[5], 0.7);
    EXPECT_EQ(sparse_input.value[2], 0.8);
}

// The map rejects a bad channel list or bad bounds.
TEST(TestControl, IdentityMapRejects)
{
    const Vector2d lower(0.0, 0.0);
    const Vector2d upper(1.0, 1.0);
    EXPECT_THROW(IdentityControlMap({0, 1, 2}, lower, upper), std::invalid_argument);
    EXPECT_THROW(IdentityControlMap({0, -1}, lower, upper), std::out_of_range);
    EXPECT_THROW(IdentityControlMap({0, 32}, lower, upper), std::out_of_range);
    EXPECT_THROW(IdentityControlMap({1, 1}, lower, upper), std::invalid_argument);
    EXPECT_THROW(IdentityControlMap({0, 1}, upper, lower), std::invalid_argument);
}

// The linear copter recipe reads the ideal inputs from channels 0 to 3, so
// the identity map drives it at its hover throttle with zero acceleration.
TEST(TestControl, IdentityMapLinearCopter)
{
    UavModel aircraft("ardupilot_copter_linear");
    initialize_aircraft(aircraft, "ardupilot_copter_linear");
    const auto &copter = dynamic_cast<const body::LinearCopter &>(*aircraft.dynamics.bodies.at(0));
    const IdentityControlMap map({0, 1, 2, 3}, Vector4d(-1.0, -1.0, -1.0, 0.0), Vector4d(1.0, 1.0, 1.0, 1.0));

    const Evaluation result = evaluate(aircraft, hover_state(aircraft), map.to_input(Vector4d(0.0, 0.0, 0.0, copter.u_hover)));
    EXPECT_LT(result.state_dot.speedDot.norm(), 1e-9);
    EXPECT_LT(result.state_dot.rateDot.norm(), 1e-9);
    EXPECT_EQ(copter.u_thr, copter.u_hover);
}

///////////////////////////////////////////////////////////////////////////////
// The trimmer

// The mixer/ block of the linear copter recipe, the stock SITL block.
static std::shared_ptr<body::MatrixMixer> stock_mixer()
{
    auto config = load_config_aircraft("ardupilot_copter_linear");
    return std::shared_ptr<body::MatrixMixer>(body::buildMixer(config.filter("dynamics/body/body1/mixer/")));
}

// The map of a LinearCopter recipe, the ideal inputs on channels 0 to 3.
static IdentityControlMap linear_copter_map()
{
    return IdentityControlMap({0, 1, 2, 3}, Vector4d(-1.0, -1.0, -1.0, 0.0), Vector4d(1.0, 1.0, 1.0, 1.0));
}

// The hover trim of the stock quad through the ArduPilot mixer.
TEST(TestControl, TrimHoverArduPilotCopter)
{
    UavModel aircraft("ardupilot_copter");
    initialize_aircraft(aircraft, "ardupilot_copter");
    const auto mixer = stock_mixer();
    ASSERT_EQ(mixer->mixer_type(), 1);
    const CopterControlMap map(mixer, aircraft);

    Trimmer trimmer;
    const TrimPoint point = trimmer.trim(aircraft, map, TrimTarget());
    const EulerAngles euler(point.state.pose.orientation);
    std::cout << std::setprecision(10) << "trim of ardupilot_copter: phi " << euler.roll << ", theta "
              << euler.pitch << ", controls " << point.controls.transpose() << ", residual " << point.residual
              << ", evaluations " << point.evaluations << ", return code " << point.return_code << std::endl;

    EXPECT_TRUE(point.converged);
    EXPECT_LT(point.residual, 1e-5);
    EXPECT_NEAR(euler.roll, 0.0, 1e-6);
    EXPECT_NEAR(euler.pitch, 0.0, 1e-6);
    EXPECT_NEAR(euler.yaw, 0.0, 1e-12);
    EXPECT_NEAR(point.controls(0), 0.0, 1e-6);
    EXPECT_NEAR(point.controls(1), 0.0, 1e-6);
    EXPECT_NEAR(point.controls(2), 0.0, 1e-6);
    EXPECT_NEAR(point.controls(3), 0.37, 0.02);

    // The trim sits at the initial position of the recipe, at rest.
    EXPECT_TRUE(point.state.pose.position.isApprox(Vector3d(0.0, 0.0, -0.17)));
    EXPECT_EQ(point.state.velocity.linear, Vector3d::Zero());
    EXPECT_EQ(point.state.velocity.angular, Vector3d::Zero());
}

// The model stays at the trim point, so the next evaluation there needs no
// settling and gives zero accelerations.
TEST(TestControl, TrimLeavesModelAtTrim)
{
    UavModel aircraft("ardupilot_copter");
    initialize_aircraft(aircraft, "ardupilot_copter");
    const CopterControlMap map(stock_mixer(), aircraft);

    Trimmer trimmer;
    const TrimPoint point = trimmer.trim(aircraft, map, TrimTarget());
    EXPECT_EQ(aircraft.state.pose.position, point.state.pose.position);
    EXPECT_EQ(aircraft.state.pose.orientation.coeffs(), point.state.pose.orientation.coeffs());
    EXPECT_EQ(aircraft.input.value, point.input.value);

    const Evaluation result = evaluate(aircraft, point.state, point.input);
    EXPECT_EQ(result.iterations, 2);
    EXPECT_LT(result.state_dot.speedDot.norm(), 1e-5);
    EXPECT_LT(result.state_dot.rateDot.norm(), 1e-5);
}

// The linear copter trims at its own hover throttle.
TEST(TestControl, TrimHoverLinearCopter)
{
    UavModel aircraft("ardupilot_copter_linear");
    initialize_aircraft(aircraft, "ardupilot_copter_linear");
    const auto &copter = dynamic_cast<const body::LinearCopter &>(*aircraft.dynamics.bodies.at(0));
    const IdentityControlMap map = linear_copter_map();

    Trimmer trimmer;
    const TrimPoint point = trimmer.trim(aircraft, map, TrimTarget());
    const EulerAngles euler(point.state.pose.orientation);
    EXPECT_TRUE(point.converged);
    EXPECT_NEAR(euler.roll, 0.0, 1e-6);
    EXPECT_NEAR(euler.pitch, 0.0, 1e-6);
    EXPECT_NEAR(point.controls(0), 0.0, 1e-6);
    EXPECT_NEAR(point.controls(1), 0.0, 1e-6);
    EXPECT_NEAR(point.controls(2), 0.0, 1e-6);
    EXPECT_NEAR(point.controls(3), copter.u_hover, 1e-6);
}

// The trimmer refuses a model with a ground reaction and names the key.
TEST(TestControl, TrimRefusesGroundReaction)
{
    UavModel aircraft("ardupilot_copter");
    initialize_aircraft(aircraft, "ardupilot_copter", true);
    const CopterControlMap map(stock_mixer(), aircraft);
    Trimmer trimmer;
    try
    {
        trimmer.trim(aircraft, map, TrimTarget());
        FAIL() << "The trimmer accepted a model with a ground reaction.";
    }
    catch (const std::runtime_error &error)
    {
        EXPECT_NE(std::string(error.what()).find("dynamics/ground/groundReactionType"), std::string::npos);
    }
}

// An error inside an evaluation reaches the caller with its own message.
TEST(TestControl, TrimPassesErrors)
{
    UavModel aircraft("ardupilot_copter");
    initialize_aircraft(aircraft, "ardupilot_copter");
    const CopterControlMap map(stock_mixer(), aircraft);
    TrimOptions options;
    options.settle.max_iterations = 1;
    Trimmer trimmer(options);
    try
    {
        trimmer.trim(aircraft, map, TrimTarget());
        FAIL() << "The trimmer hid an error of the settle loop.";
    }
    catch (const std::invalid_argument &error)
    {
        EXPECT_NE(std::string(error.what()).find("max_iterations"), std::string::npos);
    }
}
