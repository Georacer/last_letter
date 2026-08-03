/////////////////////////////////
// Define NoGroundReactions class
/////////////////////////////////

#include "last_letter_lib/ground_reaction/ground_reaction.hpp"

namespace last_letter_lib {
namespace ground_reaction {

// Force calculation function
Vector3d NoGroundReaction::getForce(const SimState_t /*states*/, const WrenchSum_t /*wrenchSum*/)
{
	return wrenchGround.force;
}

// Torque calculation function
Vector3d NoGroundReaction::getTorque(const SimState_t /*states*/, const WrenchSum_t /* wrenchSum */)
{
	return wrenchGround.torque;
}

} // namespace ground_reaction
} // namespace last_letter_lib
