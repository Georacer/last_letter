#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Eigen"

#include "unsupported/Eigen/MatrixFunctions"

#include "last_letter_lib/body/ardupilot_mixer.hpp"
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

///////////////////////////////////////////////////////////////////////////////
// The inverse mixer

// The ArduPilot plus frame: angle in degrees and yaw factor per motor.
static const vector<pair<double, double>> PLUS_FRAME = {
    {90.0, 1.0}, {-90.0, 1.0}, {0.0, -1.0}, {180.0, -1.0}};

// Return the motor PWMs of the ArduPilot mixer for [T, roll, pitch, yaw].
// A short forward copy of AP_MotorsMatrix::output_armed_stabilizing without
// saturation, Thrust_Linearization::thrust_to_actuator and
// AP_MotorsMulticopter::output_to_pwm, with the stock SITL quad parameters.
static vector<double> ardupilot_mixer(const Vector4d &cmd, double pressure = 94105.0, double bat_volt_max = 12.8)
{
    const double throttle = cmd(0), roll = cmd(1), pitch = cmd(2), yaw = cmd(3);
    const double pwm_min = 1000.0, pwm_max = 2000.0;
    const double spin_min = 0.15, spin_max = 0.95, expo = 0.65;
    const double batt_voltage = 12.6;
    double bat_volt_min = 9.6;

    // Battery compensation, update_lift_max_from_batt_voltage.
    double bv = 1.0, lift_max = 1.0;
    if (bat_volt_max > 0.0 && bat_volt_min < bat_volt_max)
    {
        bat_volt_min = std::max(bat_volt_min, 0.6 * bat_volt_max);
        bv = std::clamp(batt_voltage, bat_volt_min, bat_volt_max) / bat_volt_max;
        lift_max = bv * (1.0 - expo) + expo * bv * bv;
    }
    // Density compensation, get_compensation_gain.
    const double density_ratio = std::pow(pressure / 101325.0, 0.8097);
    const double gain = 1.0 / lift_max / std::clamp(density_ratio, 0.5, 1.25);

    // Thrust factors, add_motor and normalise_rpy_factors.
    const size_t n = PLUS_FRAME.size();
    vector<double> rf(n), pf(n), yf(n);
    for (size_t i = 0; i < n; i++)
    {
        const double angle = PLUS_FRAME[i].first * M_PI / 180.0;
        rf[i] = std::cos(angle + M_PI / 2.0);
        pf[i] = std::cos(angle);
        yf[i] = PLUS_FRAME[i].second;
    }
    const auto normalise = [](vector<double> &f)
    {
        double largest = 0.0;
        for (const double x : f) { largest = std::max(largest, std::fabs(x)); }
        for (double &x : f) { x = 0.5 * x / largest; }
    };
    normalise(rf);
    normalise(pf);
    normalise(yf);

    vector<double> pwms(n);
    for (size_t i = 0; i < n; i++)
    {
        const double thrust = gain * (throttle + roll * rf[i] + pitch * pf[i] + yaw * yf[i]);
        // Thrust curve, apply_thrust_curve_and_volt_scaling.
        const double ratio = ((expo - 1.0) + std::sqrt((1.0 - expo) * (1.0 - expo) + 4.0 * expo * lift_max * thrust))
                             / (2.0 * expo);
        const double actuator = spin_min + (spin_max - spin_min) * std::clamp(ratio / bv, 0.0, 1.0);
        pwms[i] = pwm_min + (pwm_max - pwm_min) * actuator;
    }
    return pwms;
}

struct MixerCase
{
    string copter_recipe;
    string mixer_recipe;
    double pressure;
    double bat_volt_max;
};

// The inverse mixer recovers the inputs of the forward mixer.
TEST(TestBody, TestInverseMixerRoundTrip)
{
    const vector<MixerCase> cases = {
        {"", "", 94105.0, 12.8},
        {"mixer:\n  bat_volt_max: 0.0\n", "bat_volt_max: 0.0\n", 94105.0, 0.0},
        {"mixer:\n  baro_pressure: 101325.0\n", "baro_pressure: 101325.0\n", 101325.0, 12.8},
    };
    const vector<double> throttles = {0.3, 0.37, 0.45};
    const vector<double> rolls = {-0.1, 0.0, 0.1};
    const vector<double> pitches = {-0.1, 0.05};
    const vector<double> yaws = {-0.1, 0.1};

    for (const auto &c : cases)
    {
        SCOPED_TRACE(c.copter_recipe);
        LinearCopter copter("linear_copter");
        copter.initialize(c.copter_recipe);
        ArduPilotMixerInverse mixer;
        mixer.initialize(c.mixer_recipe);

        for (const double throttle : throttles)
        for (const double roll : rolls)
        for (const double pitch : pitches)
        for (const double yaw : yaws)
        {
            const Vector4d cmd(throttle, roll, pitch, yaw);
            const vector<double> pwms = ardupilot_mixer(cmd, c.pressure, c.bat_volt_max);
            // The inverse on its own, with the exact PWM.
            const Vector4d rpyt = mixer.invert(pwms);
            EXPECT_NEAR(rpyt(0), roll, 1e-9);
            EXPECT_NEAR(rpyt(1), pitch, 1e-9);
            EXPECT_NEAR(rpyt(2), yaw, 1e-9);
            EXPECT_NEAR(rpyt(3), throttle, 1e-9);
            // Through the component, with the whole microseconds of the servo
            // frame. The rounding to 1 us moves the inputs by up to about 2e-3.
            uav_utils::InputPwm_t input;
            vector<double> rounded(pwms.size());
            for (size_t i = 0; i < pwms.size(); i++)
            {
                input.value[i] = static_cast<uint16_t>(std::lround(pwms[i]));
                rounded[i] = input.value[i];
            }
            copter.setInputPwm(input);
            const Vector4d expected = mixer.invert(rounded);
            EXPECT_NEAR(copter.u_rol, expected(0), 1e-12);
            EXPECT_NEAR(copter.u_pit, expected(1), 1e-12);
            EXPECT_NEAR(copter.u_yaw, expected(2), 1e-12);
            EXPECT_NEAR(copter.u_thr, expected(3), 1e-12);
            EXPECT_NEAR(copter.u_rol, roll, 2e-3);
            EXPECT_NEAR(copter.u_pit, pitch, 2e-3);
            EXPECT_NEAR(copter.u_yaw, yaw, 2e-3);
            EXPECT_NEAR(copter.u_thr, throttle, 2e-3);
        }
    }
}

// At pwm_min every input comes out zero.
TEST(TestBody, TestInverseMixerDisarmed)
{
    LinearCopter copter("linear_copter");
    copter.initialize("");
    uav_utils::InputPwm_t input; // Every channel at 1000 us.
    copter.setInputPwm(input);
    EXPECT_EQ(copter.u_rol, 0.0);
    EXPECT_EQ(copter.u_pit, 0.0);
    EXPECT_EQ(copter.u_yaw, 0.0);
    EXPECT_EQ(copter.u_thr, 0.0);
}

// The defaults give the gain of a default SITL flight at CMAC.
TEST(TestBody, TestInverseMixerGain)
{
    ArduPilotMixerInverse mixer;
    mixer.initialize("");
    EXPECT_NEAR(mixer.get_battery_ratio(), 0.984, 1e-3);
    EXPECT_NEAR(mixer.get_lift_max(), 0.974, 1e-3);
    EXPECT_NEAR(mixer.get_air_density_ratio(), 0.942, 1e-3);
    EXPECT_NEAR(mixer.get_compensation_gain(), 1.09, 1e-2);
}
