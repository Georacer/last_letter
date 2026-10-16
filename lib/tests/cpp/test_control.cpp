#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "Eigen/Eigen"

#include "last_letter_lib/body/linear_copter.hpp"
#include "last_letter_lib/control/evaluate.hpp"
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
