////////////////////////
// Electric engine model
////////////////////////

#include "last_letter_lib/propulsion/propulsion.hpp"

namespace last_letter_lib
{
namespace propulsion
{

using namespace std;

// Constructor
ElectricEng::ElectricEng(string name) : Thruster(name)
{
	// TODO: Make sure to add this to the ROS wrapper
	// sprintf(paramMsg, "propulsion%i", id);
	// ros::NodeHandle n;
	// pub = n.advertise<last_letter_msgs::ElectricEng>(paramMsg, 1000); //propulsion data publisher
	omega = omegaMin; // Initialize engine rotational speed

    stateType x_0 = {0};
    stateType u_0 = {0};
    reset(x_0, u_0);
}

// Destructor
ElectricEng::~ElectricEng()
{
	delete npPoly;
	delete propPowerPoly;
}

void ElectricEng::update_parameters()
{
	Thruster::update_parameters();

	vector<double> doubleVect;
	propDiam = get_param<double>("propDiam");
	engInertia = get_param<double>("engInertia");
	Kv = get_param<double>("Kv");
	Rm = get_param<double>("Rm");
	Rs = get_param<double>("Rs");
	Cells = get_param<double>("Cells");
	I0 = get_param<double>("I0");
	doubleVect = get_param<vector<double>>("RadPSLimits");
	omegaMin = doubleVect[0];
	omegaMax = doubleVect[1];

	// Create propeller efficiency polynomial
	ParameterManager nCoeffPolyConfig = params_.filter("nCoeffPoly/");
	npPoly = buildPolynomial(nCoeffPolyConfig);
	// Create propeller power polynomial
	ParameterManager propPowerPolyConfig = params_.filter("propPowerPoly/");
	propPowerPoly = buildPolynomial(propPowerPolyConfig);
}

void ElectricEng::pre_propagation(SimState_t states, Environment_t environment)
{
	rho = environment.density;

	double Ei = std::fabs(omega) / 2 / M_PI / Kv;
	double Im = (Cells * 4.0 * inputMotor - Ei) / (Rs * inputMotor + Rm);
	double engPower = Ei * (Im - I0);

	double advRatio = normalWind / (std::fabs(omega) / 2.0 / M_PI + 0.001) / propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	double propPower = propPowerPoly->evaluate(advRatio) * rho * pow(std::fabs(omega) / 2.0 / M_PI, 3) * pow(propDiam, 5);
	double deltaT = (engPower - propPower) / (std::fabs(omega) + 0.001);
	double omegaDot = 1 / engInertia * deltaT;
    stateType temp_u = {omegaDot};

    u = temp_u;
}

stateType ElectricEng::dynamics(const stateType x, const stateType u, const double t)
{
    return u; // omegaDot is the single input.
}

// Update motor rotational speed and calculate thrust
void ElectricEng::post_propagation()
{
    x[0] = constrain(x[0], omegaMin, omegaMax);
    omega = rotationDir * x[0];

	// TODO: Decide where this should be output
	// parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message
}

void ElectricEng::calc_wrench(SimState_t /* states */, Environment_t /* environment */)
{
	double Ei = std::fabs(omega) / 2 / M_PI / Kv;
	double Im = (Cells * 4.0 * inputMotor - Ei) / (Rs * inputMotor + Rm);
	double engPower = Ei * (Im - I0);

	double advRatio = normalWind / (std::fabs(omega) / 2.0 / M_PI + 0.001) / propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	double propPower = propPowerPoly->evaluate(advRatio) * rho * pow(std::fabs(omega) / 2.0 / M_PI, 3) * pow(propDiam, 5);
	double npCoeff = npPoly->evaluate(advRatio);

	double forceX, forceY, forceZ;
	double torqueX, torqueY, torqueZ;

	forceX = propPower * std::fabs(npCoeff / (normalWind + 1.0e-10)); // Added epsilon for numerical stability
	forceY = 0.0;
	forceZ = 0.0;
	double fadeFactor = (exp(-normalWind * 3 / 12));
	double staticThrust = 0.9 * fadeFactor * pow(M_PI / 2.0 * propDiam * propDiam * rho * engPower * engPower, 1.0 / 3); // static thrust fades at 5% at 12m/s
	forceX += staticThrust;

	forceX = constrain(forceX, thrustMin, thrustMax);
	torqueX = - torque_sign() * propPower / omega;
	torqueY = 0.0;
	torqueZ = 0.0;

	if (inputMotor < 0.01)
	{
		forceX = 0;
		torqueX = 0;
	} // To avoid aircraft rolling and turning on the ground while throttle is off
	wrench_sum.wrenchProp.force = Vector3d(forceX, forceY, forceZ);
	wrench_sum.wrenchProp.torque = Vector3d(torqueX, torqueY, torqueZ);
	if (!wrench_sum.wrenchProp.force.allFinite())
	{
		throw runtime_error("propulsion.cpp: State NaN in wrenchProp.force");
	}

	if (!wrench_sum.wrenchProp.torque.allFinite())
	{
		throw runtime_error("propulsion.cpp: State NaN in wrenchProp.torque");
	}
}

// Constructor
ElectricEng2::ElectricEng2(string name) : Thruster(name)
{
	// TODO: Make sure to add this to the ROS wrapper
	// sprintf(paramMsg, "propulsion%i", id);
	// ros::NodeHandle n;
	// pub = n.advertise<last_letter_msgs::ElectricEng2>(paramMsg, 1000); //propulsion data publisher
	omega = omegaMin; // Initialize engine rotational speed

    stateType x_0 = {0};
    stateType u_0 = {0};
    reset(x_0, u_0);
}

// Destructor
ElectricEng2::~ElectricEng2()
{
	delete propThrustPoly;
	delete propPowerPoly;
}

void ElectricEng2::update_parameters()
{
	Thruster::update_parameters();

	vector<double> doubleVect;
	propDiam = get_param<double>("propDiam");
	engInertia = get_param<double>("engInertia");
	Kv = get_param<double>("Kv");
	Rm = get_param<double>("Rm");
	Rs = get_param<double>("Rs");
	Cells = get_param<double>("Cells");
	I0 = get_param<double>("I0");
	doubleVect = get_param<vector<double>>("RadPSLimits");
	momentumDragCoeff = get_param<double>("momentumDragCoeff");
	propThrustMultiplier = get_param<double>("propThrustMultiplier");

	omegaMin = doubleVect[0];
	omegaMax = doubleVect[1];

	// Create propeller efficiency polynomial
	ParameterManager thrustCoeffPolyConfig = params_.filter("propThrustPoly/");
	propThrustPoly = buildPolynomial(thrustCoeffPolyConfig);
	// Create propeller power polynomial
	ParameterManager propPowerPolyConfig = params_.filter("propPowerPoly/");
	propPowerPoly = buildPolynomial(propPowerPolyConfig);
}

void ElectricEng2::pre_propagation(SimState_t states, Environment_t environment)
{
	rho = environment.density;

	double Ei = std::fabs(omega) / 2 / M_PI / Kv;
	double Im = (Cells * 4.0 * inputMotor - Ei) / (Rs * inputMotor + Rm);
	double engPower = Ei * (Im - I0);

	double advRatio = normalWind / (std::fabs(omega) / 2.0 / M_PI + 0.001) / propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	double propPower = propPowerPoly->evaluate(advRatio) * rho * pow(std::fabs(omega) / 2.0 / M_PI, 3) * pow(propDiam, 5);

	double deltaT = (engPower - propPower) / (std::fabs(omega) + 0.001);
	double omegaDot = 1 / engInertia * deltaT;

    stateType temp_u = {omegaDot};
    u = temp_u;
}

stateType ElectricEng2::dynamics(const stateType x, const stateType u, const double t)
{
    return u; // omegaDot is the single input.
}

// Update motor rotational speed and calculate thrust
void ElectricEng2::post_propagation()
{
    x[0] = constrain(x[0], omegaMin, omegaMax);
    omega = rotationDir * x[0];

	// TODO: Decide where this should be output
	// parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message
}

void ElectricEng2::calc_wrench(SimState_t /* states */, Environment_t /* environment */)
{
	double advRatio = normalWind / (std::fabs(omega) / 2.0 / M_PI + 0.001) / propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	double propPower = propPowerPoly->evaluate(advRatio) * rho * pow(std::fabs(omega) / 2.0 / M_PI, 3) * pow(propDiam, 5);

	double propThrust = propThrustPoly->evaluate(advRatio) * rho * pow(std::fabs(omega) / 2.0 / M_PI, 2) * pow(propDiam, 4);
	// Constrain propeller force to [0,+5] times the aircraft weight
	propThrust = constrain(propThrust, thrustMin, thrustMax);

	double forceX, forceY, forceZ;
	double torqueX, torqueY, torqueZ;

	forceX = propThrust * propThrustMultiplier;
	forceY = 0.0;
	// Generate momentum drag along the wind direction
	Vector3d sidewind = relativeWind;
	sidewind.x() = 0; // Null out the axial component
	Vector3d momentumDrag = -forceX * sidewind * momentumDragCoeff;
	forceY += momentumDrag.y();
	forceZ = momentumDrag.z();

	torqueX = - torque_sign() * propPower / omega;
	torqueY = 0.0;
	torqueZ = 0.0;

	if (inputMotor < 0.01)
	{
		forceX = 0;
		forceY = 0;
		forceZ = 0;
		torqueX = 0;
	} // To avoid aircraft rolling and turning on the ground while throttle is off

	wrench_sum.wrenchProp.force = Vector3d(forceX, forceY, forceZ);
	wrench_sum.wrenchProp.torque = Vector3d(torqueX, torqueY, torqueZ);
	if (!wrench_sum.wrenchProp.force.allFinite())
	{
		throw runtime_error("propulsion.cpp: State NaN in wrenchProp.force");
	}

	if (!wrench_sum.wrenchProp.torque.allFinite())
	{
		throw runtime_error("propulsion.cpp: State NaN in wrenchProp.torque");
	}
}

} // namespace propulsion
} // namespace last_letter_lib
