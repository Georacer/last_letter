#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "last_letter_lib/body/ardupilot_mixer.hpp"
#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"

namespace last_letter_lib {
namespace body {

// Sea level pressure of the 1976 standard atmosphere, in Pa.
static constexpr double sea_level_pressure{101325.0};

// The PWM band of the base mixer, the convention of PwmToHalfRange.
static constexpr double base_pwm_min{1000.0};
static constexpr double base_pwm_max{2000.0};

///////////////////////////////////////////////////////////////////////////////
// MatrixMixer

void MatrixMixer::initialize_parameters()
{
    set_param("mixerType", 0, false);
    // ArduPilot scales each roll, pitch and yaw column to a largest magnitude
    // of 0.5. A plain allocation matrix uses 1.
    set_param("rpy_factor_max", 0.5, false);

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

void MatrixMixer::update_parameters()
{
    const double rpy_factor_max = get_param<double>("rpy_factor_max");
    if (!(rpy_factor_max > 0.0))
    {
        throw std::runtime_error("ardupilot_mixer.cpp: rpy_factor_max must be above 0");
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
        if (channels_[i] < 0)
        {
            throw std::runtime_error("ardupilot_mixer.cpp: " + prefix + "chanMotor must be 0 or more");
        }
        frame_(i, 0) = 1.0;
        frame_(i, 1) = std::cos(angle + M_PI / 2.0);
        frame_(i, 2) = std::cos(angle);
        frame_(i, 3) = get_param<double>(prefix + "yaw_factor");
    }
    // Scale the roll, pitch and yaw factors to a largest magnitude of rpy_factor_max.
    for (int axis = 1; axis < 4; axis++)
    {
        const double largest = frame_.col(axis).cwiseAbs().maxCoeff();
        if (largest > 0.0)
        {
            frame_.col(axis) *= rpy_factor_max / largest;
        }
    }
    // A quad gives a square solve. A larger frame gives a least squares solve.
    frame_inverse_ = frame_.completeOrthogonalDecomposition().pseudoInverse();
}

Eigen::VectorXd MatrixMixer::mix(const Eigen::Vector4d &rpyt) const
{
    const Eigen::Vector4d tRPY(rpyt(3), rpyt(0), rpyt(1), rpyt(2));
    return frame_ * tRPY;
}

Eigen::Vector4d MatrixMixer::invert(const Eigen::VectorXd &thrust) const
{
    if (thrust.size() != n_motors())
    {
        throw std::invalid_argument("ardupilot_mixer.cpp: the thrust vector needs one entry per motor");
    }
    const Eigen::Vector4d tRPY = frame_inverse_ * thrust;
    return Eigen::Vector4d(tRPY(1), tRPY(2), tRPY(3), tRPY(0));
}

std::vector<double> MatrixMixer::pwm_vector(double value) const
{
    size_t size = uav_utils::num_input_channels;
    for (const int channel : channels_)
    {
        size = std::max(size, static_cast<size_t>(channel) + 1);
    }
    return std::vector<double>(size, value);
}

double MatrixMixer::motor_pwm(const std::vector<double> &pwm, int motor) const
{
    const int channel = channels_[motor];
    if (channel >= static_cast<int>(pwm.size()))
    {
        throw std::out_of_range("ardupilot_mixer.cpp: chanMotor outside the PWM vector");
    }
    return pwm[channel];
}

std::vector<double> MatrixMixer::mix_to_pwm(const Eigen::Vector4d &rpyt) const
{
    const Eigen::VectorXd thrust = mix(rpyt);
    std::vector<double> pwm = pwm_vector(base_pwm_min);
    for (int i = 0; i < n_motors(); i++)
    {
        pwm[channels_[i]] = base_pwm_min + thrust(i) * (base_pwm_max - base_pwm_min);
    }
    return pwm;
}

Eigen::Vector4d MatrixMixer::invert_pwm(const std::vector<double> &pwm) const
{
    Eigen::VectorXd thrust(n_motors());
    for (int i = 0; i < n_motors(); i++)
    {
        thrust(i) = std::clamp((motor_pwm(pwm, i) - base_pwm_min) / (base_pwm_max - base_pwm_min), 0.0, 1.0);
    }
    return invert(thrust);
}

///////////////////////////////////////////////////////////////////////////////
// ArduPilotMixer

void ArduPilotMixer::initialize_parameters()
{
    MatrixMixer::initialize_parameters();
    set_param("mixerType", 1, false);

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
}

void ArduPilotMixer::update_parameters()
{
    MatrixMixer::update_parameters();

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
}

double ArduPilotMixer::thrust_to_actuator(double thrust) const
{
    thrust = std::clamp(thrust, 0.0, 1.0);
    // Port of apply_thrust_curve_and_volt_scaling.
    const double battery_scale = 1.0 / battery_ratio_;
    double throttle;
    if (thst_expo_ == 0.0)
    {
        throttle = lift_max_ * thrust * battery_scale;
    }
    else
    {
        const double root = std::sqrt(std::max(
            0.0, (1.0 - thst_expo_) * (1.0 - thst_expo_) + 4.0 * thst_expo_ * lift_max_ * thrust));
        const double ratio = ((thst_expo_ - 1.0) + root) / (2.0 * thst_expo_);
        throttle = std::clamp(ratio * battery_scale, 0.0, 1.0);
    }
    return spin_min_ + (spin_max_ - spin_min_) * throttle;
}

double ArduPilotMixer::actuator_to_thrust(double actuator) const
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

std::vector<double> ArduPilotMixer::mix_to_pwm(const Eigen::Vector4d &rpyt) const
{
    // The frame matrix is linear, so the gain can act on the inputs.
    const Eigen::Vector4d compensated = compensation_gain_ * rpyt;
    const Eigen::VectorXd thrust = mix(compensated);
    std::vector<double> pwm = pwm_vector(pwm_min_);
    for (int i = 0; i < n_motors(); i++)
    {
        const double actuator = thrust_to_actuator(thrust(i));
        pwm[channels_[i]] = pwm_min_ + actuator * (pwm_max_ - pwm_min_);
    }
    return pwm;
}

Eigen::Vector4d ArduPilotMixer::invert_pwm(const std::vector<double> &pwm) const
{
    Eigen::VectorXd thrust(n_motors());
    for (int i = 0; i < n_motors(); i++)
    {
        const double actuator = (motor_pwm(pwm, i) - pwm_min_) / (pwm_max_ - pwm_min_);
        thrust(i) = actuator_to_thrust(actuator);
    }
    // Solve for the inputs, then remove the compensation gain.
    const Eigen::Vector4d rpyt = invert(thrust) / compensation_gain_;
    return rpyt;
}

///////////////////////////////////////////////////////////////////////////////
// The factory

std::unique_ptr<MatrixMixer> buildMixer(ParameterManager config)
{
    const int mixerType = config.exists("mixerType") ? config.get<int>("mixerType") : 0;

    std::unique_ptr<MatrixMixer> mixer;
    switch (mixerType)
    {
    case 0:
        mixer = std::make_unique<MatrixMixer>();
        break;
    case 1:
        mixer = std::make_unique<ArduPilotMixer>();
        break;
    default:
        std::stringstream ss;
        ss << "Error while constructing mixer: Unknown type " << mixerType;
        throw std::runtime_error(ss.str());
    }
    mixer->initialize(config);
    return mixer;
}

} // namespace body
} // namespace last_letter_lib
