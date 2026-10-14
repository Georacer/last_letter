#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "last_letter_lib/body/ardupilot_mixer.hpp"
#include "last_letter_lib/math_utils.hpp"

namespace last_letter_lib {
namespace body {

// Sea level pressure of the 1976 standard atmosphere, in Pa.
static constexpr double sea_level_pressure{101325.0};

void ArduPilotMixerInverse::initialize_parameters()
{
    // The defaults mirror the stock ArduPilot SITL quad.
    set_param("pwm_min", 1000.0, false);
    set_param("pwm_max", 2000.0, false);
    set_param("spin_min", 0.15, false);
    set_param("spin_max", 0.95, false);
    set_param("thst_expo", 0.65, false);
    set_param("baro_pressure", 94105.0, false);
    set_param("batt_voltage", 12.6, false);
    set_param("bat_volt_max", 12.8, false);
    set_param("bat_volt_min", 9.6, false);

    // The plus frame, from the frame table in AP_MotorsMatrix.cpp.
    set_param("nMotors", 4, false);
    const double angles[4] = {90.0, -90.0, 0.0, 180.0};
    const double yaw_factors[4] = {1.0, 1.0, -1.0, -1.0};
    // Declare every motor slot, so that a recipe for a larger frame can set them.
    for (int i = 0; i < max_motors; i++)
    {
        const std::string prefix = "motor" + std::to_string(i + 1) + "/";
        set_param(prefix + "chanMotor", i, false);
        set_param(prefix + "angle_deg", (i < 4) ? angles[i] : 0.0, false);
        set_param(prefix + "yaw_factor", (i < 4) ? yaw_factors[i] : 0.0, false);
    }
}

void ArduPilotMixerInverse::update_parameters()
{
    pwm_min_ = get_param<double>("pwm_min");
    pwm_max_ = get_param<double>("pwm_max");
    spin_min_ = get_param<double>("spin_min");
    spin_max_ = get_param<double>("spin_max");
    thst_expo_ = std::clamp(get_param<double>("thst_expo"), -1.0, 1.0);
    if (pwm_max_ <= pwm_min_)
    {
        throw std::runtime_error("ardupilot_mixer.cpp: pwm_max must be above pwm_min");
    }
    if (spin_max_ <= spin_min_)
    {
        throw std::runtime_error("ardupilot_mixer.cpp: spin_max must be above spin_min");
    }

    // Battery compensation, as update_lift_max_from_batt_voltage. Through the
    // bridge the battery voltage is constant, so there is no filter.
    const double batt_voltage = get_param<double>("batt_voltage");
    const double bat_volt_max = get_param<double>("bat_volt_max");
    double bat_volt_min = get_param<double>("bat_volt_min");
    if (bat_volt_max <= 0.0 || bat_volt_min >= bat_volt_max || batt_voltage < 0.25 * bat_volt_min)
    {
        battery_ratio_ = 1.0;
        lift_max_ = 1.0;
    }
    else
    {
        bat_volt_min = std::max(bat_volt_min, 0.6 * bat_volt_max);
        battery_ratio_ = std::clamp(batt_voltage, bat_volt_min, bat_volt_max) / bat_volt_max;
        lift_max_ = battery_ratio_ * (1.0 - thst_expo_) + thst_expo_ * battery_ratio_ * battery_ratio_;
    }

    // Density compensation, as get_compensation_gain. SITL uses the 1976
    // standard atmosphere, so the pressure gives the density in the troposphere.
    air_density_ratio_ = std::pow(get_param<double>("baro_pressure") / sea_level_pressure, 0.8097);
    compensation_gain_ = (lift_max_ > 0.0) ? 1.0 / lift_max_ : 1.0;
    if (air_density_ratio_ > 0.3 && air_density_ratio_ < 1.5)
    {
        compensation_gain_ /= std::clamp(air_density_ratio_, 0.5, 1.25);
    }

    // The frame matrix, as add_motor and normalise_rpy_factors.
    const int n_motors = get_param<int>("nMotors");
    if (n_motors < 1 || n_motors > max_motors)
    {
        throw std::runtime_error("ardupilot_mixer.cpp: nMotors must be 1 to " + std::to_string(max_motors));
    }
    channels_.assign(n_motors, 0);
    frame_ = Eigen::MatrixXd::Zero(n_motors, 4);
    for (int i = 0; i < n_motors; i++)
    {
        const std::string prefix = "motor" + std::to_string(i + 1) + "/";
        const double angle = math_utils::deg_to_rad(get_param<double>(prefix + "angle_deg"));
        channels_[i] = get_param<int>(prefix + "chanMotor");
        frame_(i, 0) = 1.0;
        frame_(i, 1) = std::cos(angle + M_PI / 2.0);
        frame_(i, 2) = std::cos(angle);
        frame_(i, 3) = get_param<double>(prefix + "yaw_factor");
    }
    // ArduPilot scales the roll, pitch and yaw factors to a largest magnitude of 0.5.
    for (int axis = 1; axis < 4; axis++)
    {
        const double largest = frame_.col(axis).cwiseAbs().maxCoeff();
        if (largest > 0.0)
        {
            frame_.col(axis) *= 0.5 / largest;
        }
    }
    // A quad gives a square solve. A larger frame gives a least squares solve.
    frame_inverse_ = frame_.completeOrthogonalDecomposition().pseudoInverse();
}

double ArduPilotMixerInverse::actuator_to_thrust(double actuator) const
{
    const double throttle = (actuator - spin_min_) / (spin_max_ - spin_min_);
    const double battery_scale = 1.0 / battery_ratio_;
    double thrust;
    if (thst_expo_ == 0.0)
    {
        thrust = throttle / (lift_max_ * battery_scale);
    }
    else
    {
        thrust = (throttle / battery_scale) * (2.0 * thst_expo_) - (thst_expo_ - 1.0);
        thrust = thrust * thrust - (1.0 - thst_expo_) * (1.0 - thst_expo_);
        thrust /= 4.0 * thst_expo_ * lift_max_;
    }
    return std::clamp(thrust, 0.0, 1.0);
}

Eigen::Vector4d ArduPilotMixerInverse::invert(const std::vector<double> &pwm) const
{
    const int n_motors = static_cast<int>(channels_.size());
    Eigen::VectorXd thrust(n_motors);
    for (int i = 0; i < n_motors; i++)
    {
        const int channel = channels_[i];
        if (channel < 0 || channel >= static_cast<int>(pwm.size()))
        {
            throw std::out_of_range("ardupilot_mixer.cpp: chanMotor outside the PWM vector");
        }
        const double actuator = (pwm[channel] - pwm_min_) / (pwm_max_ - pwm_min_);
        thrust(i) = actuator_to_thrust(actuator);
    }
    // Solve for [throttle, roll, pitch, yaw], then remove the compensation gain.
    const Eigen::Vector4d tRPY = frame_inverse_ * thrust / compensation_gain_;
    return Eigen::Vector4d(tRPY(1), tRPY(2), tRPY(3), tRPY(0));
}

} // namespace body
} // namespace last_letter_lib
