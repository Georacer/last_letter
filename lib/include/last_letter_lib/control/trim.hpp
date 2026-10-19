//////////////////////////////////////
// Trimmer related declarations //
//////////////////////////////////////

// The trimmer finds the state and the controls that give zero body
// accelerations at a target. It runs NLopt LN_BOBYQA on the squared residual
// of [speedDot; rateDot]. NLopt stays behind a pimpl, so this header names
// Eigen and the library types only.

#pragma once

#include <memory>
#include <string>

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

// The objects of one recipe trim. The aircraft sits at the trim point with
// its components settled, and the map is the one that trimmed it. The map
// keeps pointers into the aircraft, so the struct holds both and they go out
// of scope together.
struct RecipeTrim {
    std::unique_ptr<UavModel> aircraft;
    std::shared_ptr<body::MatrixMixer> mixer;
    std::unique_ptr<CopterControlMap> map;
    TrimPoint trim;
};

// Trim a multicopter recipe through a mixer. The function loads the recipe,
// sets dynamics/ground/groundReactionType to 0 and env/windRef to 0, builds
// the aircraft and a CopterControlMap through the mixer, and trims at the
// target. The mixer tree is a mixer/ block with its mixerType. An empty models
// root means $HOME/last_letter_models. Throws std::runtime_error when the trim
// does not converge.
RecipeTrim trim_recipe(const std::string &recipe, const programming_utils::ParameterManager &mixer,
                       const std::string &models_root = "", const TrimTarget &target = {},
                       const TrimOptions &options = {});

} // namespace control
} // namespace last_letter_lib
