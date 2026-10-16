#include <algorithm>
#include <sstream>
#include <stdexcept>

#include "last_letter_lib/control/evaluate.hpp"
#include "last_letter_lib/ground_reaction/ground_reaction.hpp"

namespace last_letter_lib {
namespace control {

using uav_utils::Wrench_t;

namespace {

// Remove one source from a wrench about the CoG. The source is about the body
// frame origin, so its torque moves to the CoG first.
void remove_source(Wrench_t &wrench, const Wrench_t &source, const Eigen::Vector3d &cog)
{
    wrench.force -= source.force;
    const Eigen::Vector3d torque_about_cog = source.torque - cog.cross(source.force);
    wrench.torque -= torque_about_cog;
}

// The largest change of any element between two wrenches.
double largest_change(const Wrench_t &a, const Wrench_t &b)
{
    const double force_change = (a.force - b.force).cwiseAbs().maxCoeff();
    const double torque_change = (a.torque - b.torque).cwiseAbs().maxCoeff();
    return std::max(force_change, torque_change);
}

} // namespace

void require_no_ground_reaction(const UavModel &uav)
{
    const auto *ground = uav.dynamics.ground_reaction.get();
    if (dynamic_cast<const ground_reaction::NoGroundReaction *>(ground) == nullptr)
    {
        throw std::runtime_error(
            "control: the model has a ground reaction. Set dynamics/ground/groundReactionType to 0 "
            "before you build the aircraft.");
    }
}

Evaluation evaluate(UavModel &uav, const uav_utils::SimState_t &state,
                    const uav_utils::Input &input, const SettleOptions &options)
{
    if (options.max_iterations < 2)
    {
        throw std::invalid_argument("control: SettleOptions::max_iterations must be 2 or more");
    }

    // Set the input first. A reset channel in the input can call init(),
    // which overwrites the state.
    uav.setInput(input);
    uav.set_state(state);

    Evaluation result;
    Wrench_t previous = uav.calc_wrench();
    result.iterations = 1;
    while (true)
    {
        const Wrench_t current = uav.calc_wrench();
        result.iterations++;
        const double change = largest_change(current, previous);
        if (change < options.tolerance)
        {
            result.wrench = current;
            break;
        }
        if (result.iterations >= options.max_iterations)
        {
            std::ostringstream oss;
            oss << "control: the components did not settle in " << result.iterations
                << " calls. The last change was " << change << ", the tolerance is "
                << options.tolerance << ".";
            throw std::runtime_error(oss.str());
        }
        previous = current;
    }

    const auto &dynamics = uav.dynamics;
    result.wrench_model = result.wrench;
    remove_source(result.wrench_model, dynamics.wrench_sum.wrenchGrav, dynamics.cog);
    remove_source(result.wrench_model, dynamics.wrench_sum.wrenchGround, dynamics.cog);

    result.inertial = dynamics.inertial;
    uav.kinematics.calcDerivatives(uav.state, result.inertial, result.wrench);
    result.state_dot = uav.kinematics.stateDot;
    return result;
}

} // namespace control
} // namespace last_letter_lib
