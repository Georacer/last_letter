////////////////////////////////////////
// Define GroundReaction interface class
////////////////////////////////////////

#include "last_letter_lib/ground_reaction/ground_reaction.hpp"

namespace last_letter_lib {
namespace ground_reaction {

// Class constructor
GroundReaction::GroundReaction(string name): Parametrized(name)
{
    inputSteer = 0.0;
    inputBrake = 0.0;
}

void GroundReaction::initialize_parameters()
{
    set_param<double>("world/deltaT", 0.0025, false);
    set_param<int>("chanSteer", -1, false);
    set_param<int>("chanBrake", -1, false);
    set_param<double>("steerAngle_max", 0, false);
}

void GroundReaction::update_parameters()
{
    dt = get_param<double>("world/deltaT");
    chanSteer = get_param<int>("chanSteer");
    chanBrake = get_param<int>("chanBrake");
    steerAngle_max = get_param<double>("steerAngle_max");
}

void GroundReaction::set_dt(double dt_p)
{
    dt = dt_p;
    set_param("world/deltaT", dt_p);
}

// Store the steering and brake control inputs
void GroundReaction::setInput(const Input &input)
{
    if (chanSteer > -1) { set_steer(input.value[chanSteer]); }
    if (chanBrake > -1) { set_brake(input.value[chanBrake]); }
}

// Store the steering and brake control inputs
void GroundReaction::setInputPwm(const InputPwm_t &p_input)
{
    // Steering is a two-sided deflection, braking a one-sided fraction.
    if (chanSteer > -1) { set_steer(PwmToFullRange(p_input.value[chanSteer])); }
    if (chanBrake > -1) { set_brake(PwmToHalfRange(p_input.value[chanBrake])); }
}

// Build ground reactions model
std::unique_ptr<GroundReaction> buildGroundReaction(ParameterManager config)
{
    int groundReactionType;
    groundReactionType = config.get<double>("groundReactionType");
    string name = config.get<string>("name");
    std::unique_ptr<GroundReaction> ground_reaction;
    std::cout << "building ground reactions model: ";
    switch (groundReactionType)
    {
    case 0:
        std::cout << "selecting no ground reactions" << std::endl;
        ground_reaction = std::make_unique<NoGroundReaction>(name);
        break;
    case 1:
        std::cout << "selecting Panos ground reactions" << std::endl;
        ground_reaction = std::make_unique<PanosContactPoints>(name);
        break;
    case 2:
        std::cout << "selecting PointFriction ground reactions" << std::endl;
        ground_reaction = std::make_unique<PointFriction>(name);
        break;
    default:
        throw runtime_error("Error while constructing ground reactions");
        break;
    }

    ground_reaction->initialize(config);
    return ground_reaction;
}

} // namespace ground_reaction
} // namespace last_letter_lib
