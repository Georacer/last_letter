/////////////////////////
// Piston engine model //
/////////////////////////

#include "last_letter_lib/propulsion/propulsion.hpp"

namespace last_letter_lib
{
namespace propulsion
{

using namespace std;

// Constructor
PistonEng::PistonEng(string name)
    :Thruster(name)
{
	omega = omegaMin; // Initialize engine rotational speed

    stateType x_0 = {0};
    stateType u_0 = {0};
    reset(x_0, u_0);
}

// Destructor
PistonEng::~PistonEng()
{
	delete npPoly;
	// delete engPowerPoly; //@TODO examine if I need to uncomment this
	delete propPowerPoly;
}

void PistonEng::update_parameters()
{
	Thruster::update_parameters();

	propDiam = get_param<double>("propDiam");
	engInertia = get_param<double>("engInertia");
	vector<double> doubleVect;
	doubleVect = get_param<vector<double>>("RadPSLimits");
	// // Initialize RadPS limits
	omegaMin = doubleVect[0];
	omegaMax = doubleVect[1];

	// Create engine power polynomial
	ParameterManager engPowerPolyConfig = params_.filter("engPowerPoly");
	engPowerPoly = buildPolynomial(engPowerPolyConfig);
	// Create propeller efficiency polynomial
	ParameterManager engCoeffPolyConfig = params_.filter("engCoeffPoly");
	npPoly = buildPolynomial(engCoeffPolyConfig);
	// Create propeller power polynomial
	ParameterManager propPowerPolyConfig = params_.filter("propPowerPoly");
	propPowerPoly = buildPolynomial(propPowerPolyConfig);
}

void PistonEng::pre_propagation(SimState_t, Environment_t environment)
{
	rho = environment.density; // Read current air density

	// Calculate current engine power
	double powerHP = engPowerPoly->evaluate(omega/2.0/M_PI*60);
	double engPower = inputMotor * powerHP * 745.7;

	double advRatio = normalWind/ (omega/2.0/M_PI) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	advRatio = std::max(advRatio, 0.0); // Force advance ratio above zero, in lack of a better propeller model
	double propPower = propPowerPoly->evaluate(advRatio) * rho * pow(omega/2.0/M_PI,3) * pow(propDiam,5);

	double deltaT = (engPower - propPower)/omega;
	double omegaDot = 1/engInertia*deltaT;

    stateType temp_u = {omegaDot};
    u = temp_u;
}

stateType PistonEng::dynamics(const stateType x, const stateType u, const double t)
{
    return u; // omegaDot is the single input.
}

void PistonEng::post_propagation()
{
    x[0] = constrain(x[0], omegaMin, omegaMax);
    omega = rotationDir * x[0];
}

void PistonEng::calc_wrench(SimState_t /* states */, Environment_t environment)
{
	rho = environment.density; // Read current air density

	// Calculate current engine power
	double powerHP = engPowerPoly->evaluate(omega/2.0/M_PI*60);
	double engPower = inputMotor * powerHP * 745.7;

	double forceX, forceY, forceZ;
	double torqueX, torqueY, torqueZ;

	double advRatio = normalWind/ (omega/2.0/M_PI) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	advRatio = std::max(advRatio, 0.0); // Force advance ratio above zero, in lack of a better propeller model
	double propPower = propPowerPoly->evaluate(advRatio) * rho * pow(omega/2.0/M_PI,3) * pow(propDiam,5);
	double npCoeff = npPoly->evaluate(advRatio);
	forceX = propPower*std::fabs(npCoeff/(normalWind+1.0e-10)); // Added epsilon for numerical stability

	double fadeFactor = (exp(-normalWind*3/12));
	double staticThrust = 0.9*fadeFactor*pow(M_PI/2.0*propDiam*propDiam*rho*engPower*engPower,1.0/3); //static thrust fades at 5% at 12m/s
	forceX += staticThrust;

	forceX = constrain(forceX, thrustMin, thrustMax);
	torqueX = propPower / omega;
	if (inputMotor < 0.01) {
		forceX = 0;
		torqueX = 0;
	} // To avoid aircraft rolling and turning on the ground while throttle is off
	torqueY = 0.0;
	torqueZ = 0.0;

	wrench_sum.wrenchProp.torque = Vector3d(forceX, forceY, forceZ);
	wrench_sum.wrenchProp.torque = Vector3d(torqueX, torqueY, torqueZ);
	if (!wrench_sum.wrenchProp.force.allFinite()) {throw runtime_error("propulsion.cpp: State NaN in wrenchProp.force");}
	if (!wrench_sum.wrenchProp.torque.allFinite()) {throw runtime_error("propulsion.cpp: State NaN in wrenchProp.torque");}
}

} // namespace propulsion
} // namespace last_letter_lib
