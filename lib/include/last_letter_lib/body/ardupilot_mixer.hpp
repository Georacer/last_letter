/////////////////////////////////////////////
// Multicopter mixer related declarations //
/////////////////////////////////////////////

// Two mixers between the ideal roll, pitch, yaw and throttle commands of a
// multicopter and the thrust of each motor. MatrixMixer is the frame matrix and
// its pseudo inverse. ArduPilotMixer adds the compensation gain, the thrust
// curve, the spin band and the PWM band of the ArduPilot multicopter mixer.
// The recipe key mixerType selects the class through buildMixer.

#pragma once

#include <memory>
#include <vector>

#include <Eigen/Eigen>

#include "last_letter_lib/prog_utils.hpp"

namespace last_letter_lib {
namespace body {

using last_letter_lib::programming_utils::ParameterManager;
using last_letter_lib::programming_utils::Parametrized;

// mixerType 0. The frame matrix F, n_motors by 4, between the ideal inputs and
// the thrust fraction of each motor.
class MatrixMixer : public Parametrized
{
public:
    // The most motors a recipe can declare, as AP_MOTORS_MAX_NUM_MOTORS.
    static constexpr int max_motors{12};

    MatrixMixer() : Parametrized("mixer") {};
    ~MatrixMixer() override {};
    void initialize_parameters() override;
    void update_parameters() override;

    // The value of mixerType that selects this class.
    virtual int mixer_type() const { return 0; }
    int n_motors() const { return static_cast<int>(channels_.size()); }
    // The chanMotor of each motor, indexed by motor.
    const std::vector<int> &channels() const { return channels_; }

    // Ideal [roll, pitch, yaw, throttle] to the thrust fraction of each motor,
    // 0 to 1. The vector is indexed by motor, as the motorN/ entries.
    Eigen::VectorXd mix(const Eigen::Vector4d &rpyt) const;
    // The least squares inverse. A quad is a square solve.
    Eigen::Vector4d invert(const Eigen::VectorXd &thrust) const;
    // The same two through PWM in microseconds, indexed by channel, unrounded.
    // The base maps 1000 to 2000 us onto 0 to 1 and nothing else.
    virtual std::vector<double> mix_to_pwm(const Eigen::Vector4d &rpyt) const;
    virtual Eigen::Vector4d invert_pwm(const std::vector<double> &pwm) const;

protected:
    // A PWM vector at the given value, long enough for every channel.
    std::vector<double> pwm_vector(double value) const;
    // The PWM of the channel of a motor. Throws when the vector is too short.
    double motor_pwm(const std::vector<double> &pwm, int motor) const;

    std::vector<int> channels_;
    // Maps [throttle, roll, pitch, yaw] to the thrust of each motor.
    Eigen::MatrixXd frame_;
    // The least squares inverse of frame_.
    Eigen::MatrixXd frame_inverse_;
};

// mixerType 1. The ArduPilot multicopter mixer, AP_MotorsMatrix with
// Thrust_Linearization and the PWM output of AP_MotorsMulticopter.
class ArduPilotMixer : public MatrixMixer
{
public:
    ArduPilotMixer() : MatrixMixer() {};
    ~ArduPilotMixer() override {};
    void initialize_parameters() override;
    void update_parameters() override;

    int mixer_type() const override { return 1; }

    // The order of output_armed_stabilizing without its saturation logic: the
    // compensation gain, the frame matrix, thrust_to_actuator and the PWM band.
    std::vector<double> mix_to_pwm(const Eigen::Vector4d &rpyt) const override;
    // Recover [roll, pitch, yaw, throttle] from the PWM of every channel, in
    // microseconds. The vector is indexed by channel.
    Eigen::Vector4d invert_pwm(const std::vector<double> &pwm) const override;

    // Battery ratio, as Thrust_Linearization::update_lift_max_from_batt_voltage.
    double get_battery_ratio() const { return battery_ratio_; }
    double get_lift_max() const { return lift_max_; }
    // Air density over the sea level density, after the ArduPilot limits.
    double get_air_density_ratio() const { return air_density_ratio_; }
    // The gain ArduPilot multiplies every input with, get_compensation_gain.
    double get_compensation_gain() const { return compensation_gain_; }

private:
    // Port of Thrust_Linearization::thrust_to_actuator.
    double thrust_to_actuator(double thrust) const;
    // Port of Thrust_Linearization::actuator_to_thrust.
    double actuator_to_thrust(double actuator) const;

    double pwm_min_{1000}, pwm_max_{2000};
    double spin_min_{0.15}, spin_max_{0.95};
    double thst_expo_{0.65};
    double battery_ratio_{1}, lift_max_{1};
    double air_density_ratio_{1};
    double compensation_gain_{1};
};

// Build a mixer from its mixer/ parameters. The mixerType key selects the
// class, 0 when the key is missing.
std::unique_ptr<MatrixMixer> buildMixer(ParameterManager config);

} // namespace body
} // namespace last_letter_lib
