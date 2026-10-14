//////////////////////////////////////////////////////
// ArduPilotMixerInverse class related declarations //
//////////////////////////////////////////////////////

// The inverse of the ArduPilot multicopter matrix mixer. It turns the motor
// PWM outputs back into the ideal roll, pitch, yaw and throttle commands that
// the rate controller sent into the mixer.

#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "last_letter_lib/prog_utils.hpp"

namespace last_letter_lib {
namespace body {

using last_letter_lib::programming_utils::Parametrized;

class ArduPilotMixerInverse : public Parametrized
{
public:
    // The most motors a recipe can declare, as AP_MOTORS_MAX_NUM_MOTORS.
    static constexpr int max_motors{12};

    ArduPilotMixerInverse() : Parametrized("mixer") {};
    ~ArduPilotMixerInverse() override {};
    void initialize_parameters() override;
    void update_parameters() override;

    // Recover [roll, pitch, yaw, throttle] from the PWM of every channel, in
    // microseconds. The vector is indexed by channel.
    Eigen::Vector4d invert(const std::vector<double> &pwm) const;

    // Battery ratio, as Thrust_Linearization::update_lift_max_from_batt_voltage.
    double get_battery_ratio() const { return battery_ratio_; }
    double get_lift_max() const { return lift_max_; }
    // Air density over the sea level density, after the ArduPilot limits.
    double get_air_density_ratio() const { return air_density_ratio_; }
    // The gain ArduPilot multiplies every input with, get_compensation_gain.
    double get_compensation_gain() const { return compensation_gain_; }

private:
    // Port of Thrust_Linearization::actuator_to_thrust.
    double actuator_to_thrust(double actuator) const;

    double pwm_min_{1000}, pwm_max_{2000};
    double spin_min_{0.15}, spin_max_{0.95};
    double thst_expo_{0.65};
    double battery_ratio_{1}, lift_max_{1};
    double air_density_ratio_{1};
    double compensation_gain_{1};
    std::vector<int> channels_;
    // Maps [throttle, roll, pitch, yaw] to the thrust of each motor.
    Eigen::MatrixXd frame_;
    // The least squares inverse of frame_.
    Eigen::MatrixXd frame_inverse_;
};

} // namespace body
} // namespace last_letter_lib
