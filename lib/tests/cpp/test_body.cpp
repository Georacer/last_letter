#include <gtest/gtest.h>

#include "Eigen/Eigen"

#include "unsupported/Eigen/MatrixFunctions"

#include "last_letter_lib/body/body.hpp"
#include "last_letter_lib/body/linear_copter.hpp"
#include "last_letter_lib/dynamics.hpp"
#include "last_letter_lib/environment.hpp"
#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"

#include "test_utils.hpp"

using namespace std;
using namespace Eigen;

using namespace last_letter_lib;
using namespace last_letter_lib::body;

static const string GRAVITY_ONLY_BODY = R"(
name: plain_body
bodyType: 0
inertial:
  mass: 2.0
  tensor:
    j_xx: 0.1
    j_yy: 0.1
    j_zz: 0.2
)";

// A body of type 0 outputs its gravity wrench and nothing else.
TEST(TestBody, TestGravityOnlyBody)
{
    auto body = buildBody(ParameterManager::from_yaml_string("body1", GRAVITY_ONLY_BODY));

    SimState_t state;
    Environment_t environment;
    body->update_local_state(state, environment);
    body->calc_model();

    const double weight = 2.0 * 9.81; // GravitySimple, the default model.
    EXPECT_NEAR(body->wrench_sum.wrenchGrav.force.z(), weight, 1e-9);
    EXPECT_NEAR(body->wrench_sum.wrenchGrav.force.head<2>().norm(), 0.0, 1e-9);
    EXPECT_TRUE(body->wrench_sum.wrenchGrav.torque.isZero());
    EXPECT_TRUE(body->wrench_sum.wrenchAero.force.isZero());
    EXPECT_TRUE(body->wrench_sum.wrenchAero.torque.isZero());
    EXPECT_TRUE(body->wrench_sum.wrenchProp.force.isZero());
    EXPECT_TRUE(body->wrench_sum.wrenchProp.torque.isZero());
}

// A recipe without body.yaml gets an empty body section.
TEST(TestBody, TestNoBodyFile)
{
    auto config = load_config_aircraft("skywalker_2013");
    EXPECT_EQ(config.get<int>("dynamics/body/nBodies"), 0);
}

// The roll fit roll_chirp_sweep_fe58022f, with the fitted vertical lag.
static const string LINEAR_COPTER_BODY = R"(
name: linear_copter
bodyType: 1
world:
  deltaT: 0.001
roll:
  Yv: -0.614
  Ylat: -0.0669
  Lv: -0.511
  Lp: -2.57
  Llat: 305.3
  wlg: 174.0
vertical:
  Zw: -0.24
  Zcoll: -26.5
  wlg: 333.0
  u_hover: 0.37
)";

// Step the component at a fixed level attitude and integrate v and p with
// forward Euler. Return the final [v, p].
static Vector2d integrate_roll_axis(LinearCopter &copter, int n_steps)
{
    SimState_t state;
    Environment_t environment;
    const Matrix3d J_inv = copter.inertial.tensor.inverse();
    for (int i = 0; i < n_steps; i++)
    {
        copter.update_local_state(state, environment);
        copter.calc_model();
        const auto &ws = copter.wrench_sum;
        const Vector3d force = ws.wrenchAero.force + ws.wrenchProp.force;
        const Vector3d torque = ws.wrenchAero.torque + ws.wrenchProp.torque;
        const Vector3d rate_dot = J_inv * torque;
        state.velocity.linear.y() += copter.dt * force.y() / copter.inertial.mass;
        state.velocity.angular.x() += copter.dt * rate_dot.x();
    }
    return Vector2d(state.velocity.linear.y(), state.velocity.angular.x());
}

// A roll step matches the closed form response of the fitted model.
TEST(TestBody, TestLinearCopterRollStep)
{
    // The gravity model only acts on the z axis at a level attitude, so it
    // does not reach v or p.
    auto body = buildBody(ParameterManager::from_yaml_string("body1", LINEAR_COPTER_BODY));
    auto &copter = dynamic_cast<LinearCopter &>(*body);
    const double step = 0.1;
    copter.setInput(Input({step}));

    const int n_steps = 500;
    const Vector2d vp = integrate_roll_axis(copter, n_steps);

    // x = [v, p, d], x_dot = A x + B u. The step response from rest is
    // A^-1 (e^(A t) - I) B u.
    Matrix3d A;
    A << -0.614, 0.0, -0.0669,
         -0.511, -2.57, 305.3,
         0.0, 0.0, -174.0;
    const Vector3d B(0.0, 0.0, 174.0);
    const double t = n_steps * copter.dt;
    const Matrix3d expAt = (A * t).exp();
    const Vector3d x = A.inverse() * (expAt - Matrix3d::Identity()) * B * step;

    EXPECT_NEAR(vp(1), x(1), 0.01 * std::fabs(x(1)));
    EXPECT_NEAR(vp(0), x(0), 0.01 * std::fabs(x(0)));
}

// A negative lag bandwidth is an unstable lag and the loader rejects it.
TEST(TestBody, TestLinearCopterNegativeLagThrows)
{
    auto params = ParameterManager::from_yaml_string("body1", LINEAR_COPTER_BODY);
    params.set("roll/wlg", -1.0);
    EXPECT_THROW(buildBody(params), std::runtime_error);
}

// At hover the state terms vanish and the control force cancels gravity.
TEST(TestBody, TestLinearCopterHoverTrim)
{
    auto body = buildBody(ParameterManager::from_yaml_string("body1", LINEAR_COPTER_BODY));
    auto &copter = dynamic_cast<LinearCopter &>(*body);
    copter.setInput(Input({0.0, 0.0, 0.0, copter.u_hover}));

    SimState_t state;
    Environment_t environment;
    for (int i = 0; i < 1000; i++)
    {
        copter.update_local_state(state, environment);
        copter.calc_model();
    }
    const auto &ws = copter.wrench_sum;
    EXPECT_TRUE(ws.wrenchAero.force.isZero());
    EXPECT_TRUE(ws.wrenchAero.torque.isZero());
    EXPECT_NEAR(ws.wrenchProp.force.x(), 0.0, 1e-9);
    EXPECT_NEAR(ws.wrenchProp.force.y(), 0.0, 1e-9);
    EXPECT_NEAR(ws.wrenchProp.force.z(), -3.0 * 9.81, 1e-9);
    EXPECT_TRUE((ws.wrenchProp.force + ws.wrenchGrav.force).isZero(1e-9));
    EXPECT_TRUE(ws.wrenchProp.torque.isZero());
}

// Without an inertial block the aircraft has 3 kg and a 0.25 m disc tensor.
TEST(TestBody, TestLinearCopterDefaultInertia)
{
    auto config = ParameterManager::from_yaml_string("dynamics", R"(
world:
  deltaT: 0.0025
aero:
  nWings: 0
prop:
  nMotors: 0
body:
  nBodies: 1
  body1:
    name: linear_copter
    bodyType: 1
ground:
  groundReactionType: 0
  name: no_contact
)");
    Dynamics dynamics;
    dynamics.initialize(config);
    ASSERT_EQ(dynamics.nBodies, 1);
    ASSERT_EQ(dynamics.bodies.size(), 1u);

    SimState_t state;
    Environment_t environment;
    dynamics.update_local_state(state, environment);
    dynamics.calc_model(state);

    const double m = 3.0;
    const double r = 0.25;
    EXPECT_DOUBLE_EQ(dynamics.inertial.mass, m);
    EXPECT_DOUBLE_EQ(dynamics.inertial.tensor(0, 0), m * r * r / 4.0);
    EXPECT_DOUBLE_EQ(dynamics.inertial.tensor(1, 1), m * r * r / 4.0);
    EXPECT_DOUBLE_EQ(dynamics.inertial.tensor(2, 2), m * r * r / 2.0);
    EXPECT_DOUBLE_EQ(dynamics.inertial.tensor(0, 2), 0.0);
}
