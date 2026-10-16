//////////////////////////////////////////////////
// The evaluation primitive of the control tools //
//////////////////////////////////////////////////

// The trimmer and the linearizer evaluate the model at a state and an input.
// Each evaluation holds the rigid body state and calls calc_wrench() until the
// internal states of the components settle.

#pragma once

#include "last_letter_lib/math_utils.hpp"
#include "last_letter_lib/uav_model.hpp"
#include "last_letter_lib/uav_utils.hpp"

namespace last_letter_lib {
namespace control {

struct SettleOptions {
    double tolerance{1e-9};     // largest change of any wrench element between two calls
    int max_iterations{2000};   // calls of calc_wrench() before an error
};

struct Evaluation {
    uav_utils::Wrench_t wrench;        // about the CoG, every source
    uav_utils::Wrench_t wrench_model;  // the same minus wrenchGrav and wrenchGround
    Derivatives_t state_dot;           // from Kinematics
    math_utils::Inertial inertial;     // lumped, about the CoG
    int iterations{0};                 // settle calls spent
};

// Throws std::runtime_error when dynamics.ground_reaction is not a NoGroundReaction.
void require_no_ground_reaction(const UavModel &uav);

// Set the state and the input, settle the components, return the result.
Evaluation evaluate(UavModel &uav, const uav_utils::SimState_t &state,
                    const uav_utils::Input &input, const SettleOptions &options = {});

} // namespace control
} // namespace last_letter_lib
