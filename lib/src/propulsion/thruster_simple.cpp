////////////////////////
// Simple thruster model
////////////////////////

#include "last_letter_lib/propulsion/propulsion.hpp"

namespace last_letter_lib {
namespace propulsion {

// Constructor
ThrusterSimple::ThrusterSimple(string name_p)
    : Thruster(name_p)
{
	omega = 0.0;
}

void ThrusterSimple::calc_wrench(SimState_t /*states*/, Environment_t /*environment*/)
{
    const double thrust = (thrustMax - thrustMin)*inputMotor + thrustMin;
    const double torque = -torque_sign()*((torqueMax - torqueMin)*inputMotor + torqueMin);
    wrench_sum.wrenchProp.force = Vector3d(thrust, 0, 0);
    wrench_sum.wrenchProp.torque = Vector3d(torque, 0, 0);
}

} // namespace propulsion
} // namespace last_letter_lib
