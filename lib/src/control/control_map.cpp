#include <stdexcept>
#include <string>
#include <utility>

#include "last_letter_lib/control/control_map.hpp"

namespace last_letter_lib {
namespace control {

CopterControlMap::CopterControlMap(std::shared_ptr<body::MatrixMixer> mixer, const UavModel &uav)
    : mixer_(std::move(mixer))
{
    if (!mixer_)
    {
        throw std::invalid_argument("CopterControlMap: the mixer is null");
    }
    for (const auto *thruster : uav.dynamics.thrusters)
    {
        thrusters_.push_back(thruster);
    }
}

uav_utils::Input CopterControlMap::to_input(const Eigen::VectorXd &controls) const
{
    if (controls.size() != size())
    {
        throw std::invalid_argument("CopterControlMap: expected 4 controls, got " + std::to_string(controls.size()));
    }
    const Eigen::Vector4d rpyt = controls;
    uav_utils::Input input;

    if (mixer_->mixer_type() == 0)
    {
        // The thrust fraction of a motor is its command.
        const Eigen::VectorXd thrust = mixer_->mix(rpyt);
        const std::vector<int> &channels = mixer_->channels();
        for (int i = 0; i < mixer_->n_motors(); i++)
        {
            input.value.at(channels[i]) = thrust(i);
        }
        return input;
    }

    // Each thruster converts the PWM of its channel through its own band.
    const std::vector<double> pwm = mixer_->mix_to_pwm(rpyt);
    for (const auto *thruster : thrusters_)
    {
        const int channel = thruster->chanMotor;
        if (channel < 0)
        {
            continue;
        }
        if (channel >= static_cast<int>(pwm.size()))
        {
            throw std::out_of_range("CopterControlMap: chanMotor of a thruster outside the PWM vector");
        }
        input.value.at(channel) = thruster->pwm_to_throttle(pwm[channel]);
    }
    return input;
}

Eigen::VectorXd CopterControlMap::lower_bounds() const
{
    return Eigen::Vector4d(-1.0, -1.0, -1.0, 0.0);
}

Eigen::VectorXd CopterControlMap::upper_bounds() const
{
    return Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
}

IdentityControlMap::IdentityControlMap(std::vector<int> channels, Eigen::VectorXd lower, Eigen::VectorXd upper)
    : channels_(std::move(channels)), lower_(std::move(lower)), upper_(std::move(upper))
{
    const auto n = static_cast<Eigen::Index>(channels_.size());
    if (lower_.size() != n || upper_.size() != n)
    {
        throw std::invalid_argument("IdentityControlMap: give one lower and one upper bound per channel");
    }
    for (Eigen::Index i = 0; i < n; i++)
    {
        const int channel = channels_[i];
        if (channel < 0 || channel >= static_cast<int>(uav_utils::num_input_channels))
        {
            throw std::out_of_range("IdentityControlMap: channel " + std::to_string(channel)
                                    + " is outside the Input");
        }
        if (!(lower_(i) <= upper_(i)))
        {
            throw std::invalid_argument("IdentityControlMap: a lower bound is above its upper bound");
        }
    }
    for (Eigen::Index i = 0; i < n; i++)
    {
        for (Eigen::Index j = i + 1; j < n; j++)
        {
            if (channels_[i] == channels_[j])
            {
                throw std::invalid_argument("IdentityControlMap: channel " + std::to_string(channels_[i])
                                            + " appears twice");
            }
        }
    }
}

int IdentityControlMap::size() const
{
    return static_cast<int>(channels_.size());
}

uav_utils::Input IdentityControlMap::to_input(const Eigen::VectorXd &controls) const
{
    if (controls.size() != size())
    {
        throw std::invalid_argument("IdentityControlMap: expected " + std::to_string(size()) + " controls, got "
                                    + std::to_string(controls.size()));
    }
    uav_utils::Input input;
    for (int i = 0; i < size(); i++)
    {
        input.value.at(channels_[i]) = controls(i);
    }
    return input;
}

Eigen::VectorXd IdentityControlMap::lower_bounds() const
{
    return lower_;
}

Eigen::VectorXd IdentityControlMap::upper_bounds() const
{
    return upper_;
}

} // namespace control
} // namespace last_letter_lib
