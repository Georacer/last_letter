// The linearizer. Central differences of the wrench accelerations about a
// trim point, and the lookup onto the parameters of a LinearCopter.

#include <sstream>
#include <stdexcept>
#include <utility>

#include "last_letter_lib/control/linearize.hpp"
#include "last_letter_lib/math_utils.hpp"

namespace last_letter_lib {
namespace control {

using math_utils::UnitQuaternion;
using uav_utils::Input;
using uav_utils::SimState_t;
using Acceleration = Eigen::Matrix<double, 6, 1>;

namespace {

// The rows of the Jacobians.
enum Row { ax = 0, ay, az, alpha_x, alpha_y, alpha_z };
// The columns of B for a copter map, [roll, pitch, yaw, throttle].
enum Control { roll = 0, pitch, yaw, throttle };

// The acceleration [a; alpha] that the linearizer differentiates. It starts
// from the wrench without gravity and ground, and adds each term that the
// options switch on.
Acceleration acceleration(const UavModel &uav, const Evaluation &result, const SimState_t &state,
                          const LinearizerOptions &options)
{
    Eigen::Vector3d force = result.wrench_model.force;
    Eigen::Vector3d torque = result.wrench_model.torque;
    if (options.include_gravity)
    {
        // The gravity wrench is about the body frame origin. Move it to the CoG.
        const auto &gravity = uav.dynamics.wrench_sum.wrenchGrav;
        force += gravity.force;
        torque += gravity.torque - uav.dynamics.cog.cross(gravity.force);
    }

    const double mass = result.inertial.mass;
    const Eigen::Matrix3d &tensor = result.inertial.tensor;
    const Eigen::Vector3d &velocity = state.velocity.linear;
    const Eigen::Vector3d &rate = state.velocity.angular;

    Eigen::Vector3d linear = force / mass;
    if (options.include_coriolis)
    {
        linear -= rate.cross(velocity);
    }
    Eigen::Vector3d angular_moment = torque;
    if (options.include_gyroscopic)
    {
        const Eigen::Vector3d momentum = tensor * rate;
        angular_moment -= rate.cross(momentum);
    }
    const Eigen::Vector3d angular = tensor.ldlt().solve(angular_moment);

    Acceleration a;
    a << linear, angular;
    return a;
}

} // namespace

Linearizer::Linearizer(LinearizerOptions options) : options_(std::move(options))
{
}

LinearModel Linearizer::linearize(UavModel &uav, const ControlMap &map, const TrimPoint &trim)
{
    require_no_ground_reaction(uav);
    if (trim.controls.size() != map.size())
    {
        throw std::invalid_argument("Linearizer: the trim controls do not match the size of the control map");
    }
    if (!(options_.step_velocity > 0.0 && options_.step_rate > 0.0 && options_.step_attitude > 0.0
          && options_.step_control > 0.0))
    {
        throw std::invalid_argument("Linearizer: every step size must be above 0");
    }

    const SimState_t &base = trim.state;
    const Eigen::VectorXd &controls = trim.controls;
    const Input trim_input = map.to_input(controls);

    auto sample = [&](const SimState_t &state, const Input &input) -> Acceleration {
        const Evaluation result = evaluate(uav, state, input, options_.settle);
        return acceleration(uav, result, state, options_);
    };

    LinearModel model;
    model.options = options_;
    model.trim = trim;

    const double hv = options_.step_velocity;
    const double hw = options_.step_rate;
    const double ha = options_.step_attitude;
    const double hc = options_.step_control;

    for (int j = 0; j < 3; j++)
    {
        // The body velocity.
        SimState_t plus = base;
        SimState_t minus = base;
        plus.velocity.linear(j) += hv;
        minus.velocity.linear(j) -= hv;
        const Acceleration a_plus = sample(plus, trim_input);
        const Acceleration a_minus = sample(minus, trim_input);
        model.A_v.col(j) = (a_plus - a_minus) / (2.0 * hv);
    }

    for (int j = 0; j < 3; j++)
    {
        // The body rates.
        SimState_t plus = base;
        SimState_t minus = base;
        plus.velocity.angular(j) += hw;
        minus.velocity.angular(j) -= hw;
        const Acceleration a_plus = sample(plus, trim_input);
        const Acceleration a_minus = sample(minus, trim_input);
        model.A_w.col(j) = (a_plus - a_minus) / (2.0 * hw);
    }

    const Eigen::Quaterniond trim_orientation(base.pose.orientation);
    for (int j = 0; j < 3; j++)
    {
        // A small rotation of the trim quaternion about one body axis. The
        // orientation turns body vectors into NED, so a body rotation
        // multiplies from the right.
        const Eigen::Vector3d axis = Eigen::Vector3d::Unit(j);
        const Eigen::Quaterniond rotation_plus(Eigen::AngleAxisd(ha, axis));
        const Eigen::Quaterniond rotation_minus(Eigen::AngleAxisd(-ha, axis));
        const Eigen::Quaterniond q_plus = trim_orientation * rotation_plus;
        const Eigen::Quaterniond q_minus = trim_orientation * rotation_minus;
        SimState_t plus = base;
        SimState_t minus = base;
        plus.pose.orientation = UnitQuaternion(q_plus.normalized());
        minus.pose.orientation = UnitQuaternion(q_minus.normalized());
        const Acceleration a_plus = sample(plus, trim_input);
        const Acceleration a_minus = sample(minus, trim_input);
        model.A_att.col(j) = (a_plus - a_minus) / (2.0 * ha);
    }

    model.B = Eigen::MatrixXd::Zero(6, map.size());
    for (int j = 0; j < map.size(); j++)
    {
        Eigen::VectorXd plus = controls;
        Eigen::VectorXd minus = controls;
        plus(j) += hc;
        minus(j) -= hc;
        const Acceleration a_plus = sample(base, map.to_input(plus));
        const Acceleration a_minus = sample(base, map.to_input(minus));
        model.B.col(j) = (a_plus - a_minus) / (2.0 * hc);
    }

    // Leave the model at the trim point, state set and components settled.
    evaluate(uav, base, trim_input, options_.settle);
    return model;
}

LinearCopterCoefficients Linearizer::to_linear_copter(const LinearModel &model)
{
    const LinearizerOptions &options = model.options;
    if (options.include_gravity || options.include_coriolis || options.include_gyroscopic)
    {
        throw std::invalid_argument(
            "Linearizer: a LinearCopter needs include_gravity, include_coriolis and include_gyroscopic off. "
            "Its base Component applies gravity, and Kinematics adds the Coriolis and gyroscopic terms.");
    }
    if (model.B.rows() != 6 || model.B.cols() != 4)
    {
        std::ostringstream oss;
        oss << "Linearizer: a LinearCopter needs B of 6 by 4, got " << model.B.rows() << " by " << model.B.cols();
        throw std::invalid_argument(oss.str());
    }
    if (model.trim.controls.size() != 4)
    {
        throw std::invalid_argument("Linearizer: a LinearCopter needs four trim controls");
    }

    const Eigen::Matrix<double, 6, 3> &A_v = model.A_v;
    const Eigen::Matrix<double, 6, 3> &A_w = model.A_w;
    const Eigen::MatrixXd &B = model.B;

    LinearCopterCoefficients c;
    c.Yv = A_v(ay, 1);
    c.Ylat = B(ay, roll);
    c.Lv = A_v(alpha_x, 1);
    c.Lp = A_w(alpha_x, 0);
    c.Llat = B(alpha_x, roll);

    c.Xu = A_v(ax, 0);
    c.Xlon = B(ax, pitch);
    c.Mu = A_v(alpha_y, 0);
    c.Mq = A_w(alpha_y, 1);
    c.Mlon = B(alpha_y, pitch);

    // With no lag the lag state equals the input, so the whole yaw control
    // power goes to Nped.
    c.Nr = A_w(alpha_z, 2);
    c.Nped = B(alpha_z, yaw);

    c.Zw = A_v(az, 2);
    c.Zcoll = B(az, throttle);
    c.u_hover = model.trim.controls(throttle);
    return c;
}

} // namespace control
} // namespace last_letter_lib
