///////////////////////////////////////////////
// Control maps from a control vector to Input //
///////////////////////////////////////////////

// The trimmer and the linearizer search and perturb a small control vector. A
// control map turns that vector into the Input of the model and gives its
// bounds, so the solver does not need to know the airframe.

#pragma once

#include <memory>
#include <vector>

#include <Eigen/Eigen>

#include "last_letter_lib/body/ardupilot_mixer.hpp"
#include "last_letter_lib/propulsion/propulsion.hpp"
#include "last_letter_lib/uav_model.hpp"
#include "last_letter_lib/uav_utils.hpp"

namespace last_letter_lib {
namespace control {

class ControlMap {
public:
    virtual ~ControlMap() = default;
    virtual int size() const = 0;
    virtual uav_utils::Input to_input(const Eigen::VectorXd &controls) const = 0;
    virtual Eigen::VectorXd lower_bounds() const = 0;
    virtual Eigen::VectorXd upper_bounds() const = 0;
};

// [roll, pitch, yaw, throttle] through a mixer to one command per motor.
// With a MatrixMixer the thrust fraction of each motor is its command, at the
// chanMotor of the mixer. With an ArduPilotMixer each thruster of the model
// converts the PWM at its own chanMotor through pwm_to_throttle(). The model
// must outlive the map, since the map keeps pointers to its thrusters.
class CopterControlMap : public ControlMap {
public:
    CopterControlMap(std::shared_ptr<body::MatrixMixer> mixer, const UavModel &uav);
    int size() const override { return 4; }
    uav_utils::Input to_input(const Eigen::VectorXd &controls) const override;
    Eigen::VectorXd lower_bounds() const override;  // -1, -1, -1, 0
    Eigen::VectorXd upper_bounds() const override;  //  1,  1,  1, 1

private:
    std::shared_ptr<body::MatrixMixer> mixer_;
    std::vector<const propulsion::Thruster *> thrusters_;
};

// Each control goes straight to one channel of the Input. For a LinearCopter
// recipe, whose setInput reads the ideal inputs from its channels, and for a
// conventional plane later.
class IdentityControlMap : public ControlMap {
public:
    IdentityControlMap(std::vector<int> channels, Eigen::VectorXd lower, Eigen::VectorXd upper);
    int size() const override;
    uav_utils::Input to_input(const Eigen::VectorXd &controls) const override;
    Eigen::VectorXd lower_bounds() const override;
    Eigen::VectorXd upper_bounds() const override;

private:
    std::vector<int> channels_;
    Eigen::VectorXd lower_;
    Eigen::VectorXd upper_;
};

} // namespace control
} // namespace last_letter_lib
