//////////////////
// No Engine Model
//////////////////

#include "last_letter_lib/propulsion/propulsion.hpp"

namespace last_letter_lib
{
namespace propulsion
{

// Constructor
NoEngine::NoEngine(string name_p)
    : Thruster(name_p)
{
	omega = 0.0;
}

} // namespace propulsion
} // namespace last_letter_lib
