//////////////////////////////////////
// Trimmer related declarations //
//////////////////////////////////////

// The trimmer finds the state and the controls that give zero body
// accelerations at a target. It runs NLopt LN_BOBYQA on the squared residual
// of [speedDot; rateDot]. NLopt stays behind a pimpl, so this header names
// Eigen and the library types only.

#pragma once

#include <memory>

#include <Eigen/Eigen>

#include "last_letter_lib/control/control_map.hpp"
#include "last_letter_lib/control/evaluate.hpp"
#include "last_letter_lib/uav_model.hpp"
#include "last_letter_lib/uav_utils.hpp"

namespace last_letter_lib {
namespace control {

struct TrimTarget {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};      // NED, the init position by default
    Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};  // inertial velocity
    double heading{0.0};                                    // psi, rad
    double yaw_rate{0.0};                                   // rad/s
};

struct TrimOptions {
    double ftol_abs{1e-12};
    int max_evaluations{5000};
    Eigen::VectorXd weights;           // six, on [speedDot; rateDot]. Empty means ones
    Eigen::VectorXd initial_controls;  // empty means [0, 0, 0, 0.5] for a copter
    SettleOptions settle;
};

struct TrimPoint {
    uav_utils::SimState_t state;
    Eigen::VectorXd controls;
    uav_utils::Input input;
    double residual{0.0};     // |r| at the result
    int evaluations{0};
    int return_code{0};       // nlopt::result as an int
    bool converged{false};
};

class Trimmer {
public:
    explicit Trimmer(TrimOptions options = {});
    ~Trimmer();
    TrimPoint trim(UavModel &uav, const ControlMap &map, const TrimTarget &target);
private:
    struct Impl;                  // holds nlopt::opt, defined in trim.cpp
    std::unique_ptr<Impl> impl_;
};

} // namespace control
} // namespace last_letter_lib
