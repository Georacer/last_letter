#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include "last_letter_lib/body/linear_copter.hpp"

#include "last_letter_lib/logging.hpp"
#include "logging/log_types.hpp"

namespace last_letter_lib {
namespace body {

// The exact fraction of the gap u - d that a first order lag closes in one
// step of a constant input. A bandwidth of zero is a sentinel for no lag, so
// the function is not continuous there. The limit from the right is zero, a
// lag that never moves, and the value at zero is one, a lag that copies the
// input in one step. A negative bandwidth is an unstable lag and is rejected
// in update_parameters(), so it never reaches this function.
static double lag_factor(double wlg, double dt)
{
    if (wlg == 0.0)
    {
        return 1.0;
    }
    return 1.0 - std::exp(-wlg * dt);
}

// Reject a negative lag bandwidth. Zero is the no-lag sentinel.
static void check_lag_bandwidth(const std::string &axis, double wlg)
{
    if (wlg < 0.0)
    {
        std::stringstream ss;
        ss << "LinearCopter: " << axis << "/wlg must be zero or positive, got " << wlg;
        throw std::runtime_error(ss.str());
    }
}

void LinearCopter::initialize(ParameterManager config)
{
    // A tree without a mixer/ block gets the default mixer.
    const ParameterManager mixer_config = config.exists("mixer") ? config.filter("mixer/") : ParameterManager("mixer");
    mixer = buildMixer(mixer_config);
    add_child(*mixer);
    Body::initialize(config);
}

void LinearCopter::initialize(const std::string yaml_str)
{
    initialize(ParameterManager::from_yaml_string(get_name(), yaml_str));
}

void LinearCopter::initialize_parameters()
{
    Body::initialize_parameters();

    // The identified aircraft. Override the zero default of Component.
    set_param("inertial/mass", 3.0, false);
    set_param("inertial/radius", 0.25, false);

    set_param("chanRoll", 0, false);
    set_param("chanPitch", 1, false);
    set_param("chanYaw", 2, false);
    set_param("chanThrottle", 3, false);

    set_param("roll/Yv", 0.0, false);
    set_param("roll/Ylat", 0.0, false);
    set_param("roll/Lv", 0.0, false);
    set_param("roll/Lp", 0.0, false);
    set_param("roll/Llat", 0.0, false);
    set_param("roll/wlg", 0.0, false);

    set_param("pitch/Xu", 0.0, false);
    set_param("pitch/Xlon", 0.0, false);
    set_param("pitch/Mu", 0.0, false);
    set_param("pitch/Mq", 0.0, false);
    set_param("pitch/Mlon", 0.0, false);
    set_param("pitch/wlg", 0.0, false);

    set_param("yaw/Nr", 0.0, false);
    set_param("yaw/Nped", 0.0, false);
    set_param("yaw/Npedp", 0.0, false);
    set_param("yaw/wlg", 0.0, false);

    set_param("vertical/Zw", 0.0, false);
    set_param("vertical/Zcoll", 0.0, false);
    set_param("vertical/wlg", 0.0, false);
    set_param("vertical/u_hover", 0.37, false);
}

void LinearCopter::update_parameters()
{
    Body::update_parameters();

    // Without a tensor in the recipe, use a thin uniform disc.
    if (get_param<double>("inertial/tensor/j_xx") == 0.0
        && get_param<double>("inertial/tensor/j_yy") == 0.0
        && get_param<double>("inertial/tensor/j_zz") == 0.0
        && get_param<double>("inertial/tensor/j_xz") == 0.0)
    {
        const double radius = get_param<double>("inertial/radius");
        const double mr2 = inertial.mass * radius * radius;
        inertial.tensor = Eigen::Matrix3d::Zero();
        inertial.tensor(0, 0) = mr2 / 4.0;
        inertial.tensor(1, 1) = mr2 / 4.0;
        inertial.tensor(2, 2) = mr2 / 2.0;
    }

    chanRoll = get_param<int>("chanRoll");
    chanPitch = get_param<int>("chanPitch");
    chanYaw = get_param<int>("chanYaw");
    chanThrottle = get_param<int>("chanThrottle");

    Yv = get_param<double>("roll/Yv");
    Ylat = get_param<double>("roll/Ylat");
    Lv = get_param<double>("roll/Lv");
    Lp = get_param<double>("roll/Lp");
    Llat = get_param<double>("roll/Llat");
    wlg_rol = get_param<double>("roll/wlg");

    Xu = get_param<double>("pitch/Xu");
    Xlon = get_param<double>("pitch/Xlon");
    Mu = get_param<double>("pitch/Mu");
    Mq = get_param<double>("pitch/Mq");
    Mlon = get_param<double>("pitch/Mlon");
    wlg_pit = get_param<double>("pitch/wlg");

    Nr = get_param<double>("yaw/Nr");
    Nped = get_param<double>("yaw/Nped");
    Npedp = get_param<double>("yaw/Npedp");
    wlg_yaw = get_param<double>("yaw/wlg");

    Zw = get_param<double>("vertical/Zw");
    Zcoll = get_param<double>("vertical/Zcoll");
    wlg_thr = get_param<double>("vertical/wlg");
    u_hover = get_param<double>("vertical/u_hover");

    check_lag_bandwidth("roll", wlg_rol);
    check_lag_bandwidth("pitch", wlg_pit);
    check_lag_bandwidth("yaw", wlg_yaw);
    check_lag_bandwidth("vertical", wlg_thr);

    if (mixer)
    {
        mixer->update_parameters();
    }
    update_lag_factors();
}

void LinearCopter::set_dt(double dt_p)
{
    Body::set_dt(dt_p);
    update_lag_factors();
}

void LinearCopter::update_lag_factors()
{
    lag_rol_ = lag_factor(wlg_rol, dt);
    lag_pit_ = lag_factor(wlg_pit, dt);
    lag_yaw_ = lag_factor(wlg_yaw, dt);
    lag_thr_ = lag_factor(wlg_thr, dt);
}

void LinearCopter::setInput(const Input &input)
{
    if (chanRoll > -1) { u_rol = input.value[chanRoll]; }
    if (chanPitch > -1) { u_pit = input.value[chanPitch]; }
    if (chanYaw > -1) { u_yaw = input.value[chanYaw]; }
    if (chanThrottle > -1) { u_thr = input.value[chanThrottle]; }
}

void LinearCopter::setInputPwm(const InputPwm_t &input)
{
    if (!mixer)
    {
        throw std::runtime_error("LinearCopter: initialize() must run before setInputPwm()");
    }
    const std::vector<double> pwm(input.value.begin(), input.value.end());
    const Eigen::Vector4d u = mixer->invert_pwm(pwm);
    u_rol = u(0);
    u_pit = u(1);
    u_yaw = u(2);
    u_thr = u(3);
}

void LinearCopter::calc_model_impl()
{
    // Fill wrenchGrav. The gravity model applies the exact R^T m g, so the
    // linear gravity terms of the fitted models stay out of this component.
    Body::calc_model_impl();

    // Advance the lag states with the exact step for a constant input.
    d_rol += lag_rol_ * (u_rol - d_rol);
    d_pit += lag_pit_ * (u_pit - d_pit);
    d_yaw += lag_yaw_ * (u_yaw - d_yaw);
    d_thr += lag_thr_ * (u_thr - d_thr);

    const double u = local_state.velocity.linear.x();
    const double v = local_state.velocity.linear.y();
    const double w = local_state.velocity.linear.z();
    const double p = local_state.velocity.angular.x();
    const double q = local_state.velocity.angular.y();
    const double r = local_state.velocity.angular.z();

    // The trim thrust balances the weight at hover. Take g from the gravity
    // wrench, so the trim term cancels the gravity model exactly.
    const double mass = inertial.mass;
    const double g = (mass > 0.0) ? wrench_sum.wrenchGrav.force.norm() / mass : 0.0;

    // The state terms have a body velocity or a body rate as their factor.
    const Eigen::Vector3d acc_state(Xu * u, Yv * v, Zw * w);
    const Eigen::Vector3d alpha_state(Lv * v + Lp * p, Mu * u + Mq * q, Nr * r);
    // The control terms have a lag state, an input or the trim thrust as their factor.
    const Eigen::Vector3d acc_control(Xlon * d_pit, Ylat * d_rol, Zcoll * (d_thr - u_hover) - g);
    const Eigen::Vector3d alpha_control(Llat * d_rol, Mlon * d_pit, Nped * d_yaw + Npedp * u_yaw);

    // The identification gives accelerations, not forces. We scale them by a
    // nominal mass and inertia so that Kinematics can divide them out again.
    // The values of m and J cancel in J^-1 J alpha. Only the gyroscopic term
    // omega x J omega depends on them, and that term is second order at hover.
    wrench_sum.wrenchAero.force = mass * acc_state;
    wrench_sum.wrenchAero.torque = inertial.tensor * alpha_state;
    wrench_sum.wrenchProp.force = mass * acc_control;
    wrench_sum.wrenchProp.torque = inertial.tensor * alpha_control;
}

void LinearCopter::register_log_channels()
{
    Body::register_log_channels();

    auto channel = logging::get_channel(get_name());
    channel.register_value("d_rol", &d_rol);
    channel.register_value("d_pit", &d_pit);
    channel.register_value("d_yaw", &d_yaw);
    channel.register_value("d_thr", &d_thr);
    channel.register_value("u_rol", &u_rol);
    channel.register_value("u_pit", &u_pit);
    channel.register_value("u_yaw", &u_yaw);
    channel.register_value("u_thr", &u_thr);
}

} // namespace body
} // namespace last_letter_lib
