////////////////////////////////////////
// Engine model found in R. Beard's book
////////////////////////////////////////

#include "last_letter_lib/propulsion/propulsion.hpp"

namespace last_letter_lib
{
namespace propulsion
{

// Constructor
EngOmegaControl::EngOmegaControl(string name):Thruster(name)
{
	std::cout << "Building new Controlled-Omega Engine" << std::endl;;
	omega = 0; // Initialize engine rotational speed
}

// Destructor
EngOmegaControl::~EngOmegaControl()
{
}

void EngOmegaControl::update_parameters()
{
	Thruster::update_parameters();

	prop_diam = get_param<double>("prop_diam");
	omega_max = get_param<double>("omega_max");
	// Create propeller thrust polynomial
	ParameterManager thrustPolyConfig = params_.filter("thrust_poly/");
	thrust_poly =  buildPolynomial(thrustPolyConfig);
	// Create propeller power polynomial
	ParameterManager powerPolyConfig = params_.filter("power_poly/");
	power_poly =  buildPolynomial(powerPolyConfig);
}

void EngOmegaControl::post_propagation()
{
	omega = inputMotor*omega_max; // Direct omega setting from user
}

// Calculate propulsion forces
void EngOmegaControl::calc_wrench(SimState_t /* states */, Environment_t environment)
{
	rho = environment.density;
	double eps = 1e-4;

	double advRatio = normalWind / (std::fabs(omega)/2.0/M_PI + eps) / prop_diam; // Convert advance ratio to dimensionless units, not 1/rad
	torque = power_poly->evaluate(advRatio) /2/M_PI * rho * pow(std::fabs(omega)/2.0/M_PI,2) * pow(prop_diam,5);
	thrust = thrust_poly->evaluate(advRatio) * rho * pow(std::fabs(omega)/2.0/M_PI,2) * pow(prop_diam,4);

	wrench_sum.wrenchProp.force = Vector3d(thrust, 0, 0);
	if (!wrench_sum.wrenchProp.force.allFinite()) {throw runtime_error("propulsion.cpp/EngOmegaControl: State NaN in wrenchProp.force");}

	wrench_sum.wrenchProp.torque = Vector3d(-rotationDir * torque, 0, 0);
	if (!wrench_sum.wrenchProp.torque.allFinite()) {throw runtime_error("propulsion.cpp/EngOmegaControl: State NaN in wrenchProp.torque");}
}

} // namespace propulsion
} // namespace last_letter_lib
