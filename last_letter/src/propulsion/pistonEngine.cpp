///////////////////////////////////////////////////
// Piston engine model (based upon Zanzoterra 305i)
///////////////////////////////////////////////////

// Constructor
PistonEng::PistonEng(ModelPlane * parent, int ID) : Propulsion(parent, ID)
{
	XmlRpc::XmlRpcValue list;
	int i, length;
	char s[100];
	char paramMsg[50];
	sprintf(paramMsg, "motor%i/propDiam", id);
	if(!ros::param::getCached(paramMsg, propDiam)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/engInertia", id);
	if(!ros::param::getCached(paramMsg, engInertia)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	// Initialize RadPS limits
	sprintf(paramMsg, "motor%i/RadPSLimits", id);
	if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	omegaMin = list[0];
	omegaMax = list[1];

	Factory factory;
	// Create engine power polynomial
	sprintf(s,"%s%i/%s","motor", id, "engPowerPoly");
	engPowerPoly =  factory.buildPolynomial(s);
	// Create propeller efficiency polynomial
	sprintf(s,"%s%i/%s","motor", id , "nCoeffPoly");
	npPoly =  factory.buildPolynomial(s);
	// Create propeller power polynomial
	sprintf(s,"%s%i/%s","motor", id, "propPowerPoly");
	propPowerPoly =  factory.buildPolynomial(s);

	omega = omegaMin; // Initialize engine rotational speed

	wrenchProp.force.x = 0.0;
	wrenchProp.force.y = 0.0;
	wrenchProp.force.z = 0.0;
	wrenchProp.torque.x = 0.0;
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;

}

// Destructor
PistonEng::~PistonEng()
{
	delete npPoly;
	// delete engPowerPoly; //@TODO examine if I need to uncomment this
	delete propPowerPoly;
}

// Update motor rotational speed and calculate thrust
void PistonEng::updateRadPS()
{
	rho = parentObj->environment.density;

	double powerHP = engPowerPoly->evaluate(omega/2.0/M_PI*60);
	double engPower = inputMotor * powerHP * 745.7; // Calculate current engine power

	double advRatio = normalWind/ (omega/2.0/M_PI) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	advRatio = std::max(advRatio, 0.0); // Force advance ratio above zero, in lack of a better propeller model
	double propPower = propPowerPoly->evaluate(advRatio) * parentObj->environment.density * pow(omega/2.0/M_PI,3) * pow(propDiam,5);
	double npCoeff = npPoly->evaluate(advRatio);
	// wrenchProp.force.x = propPower*std::fabs(npCoeff)/(parentObj->states.velocity.linear.x+1.0e-10); // Added epsilon for numerical stability
	wrenchProp.force.x = propPower*std::fabs(npCoeff/(normalWind+1.0e-10)); // Added epsilon for numerical stability

	double fadeFactor = (exp(-normalWind*3/12));
	double staticThrust = 0.9*fadeFactor*pow(M_PI/2.0*propDiam*propDiam*rho*engPower*engPower,1.0/3); //static thrust fades at 5% at 12m/s
	// ROS_INFO("engPower: %g, staticThrust: %g, regForce: %g", engPower, staticThrust, wrenchProp.force.x);
	wrenchProp.force.x = wrenchProp.force.x + staticThrust;

	// Constrain propeller force to +-2 times the aircraft weight
	wrenchProp.force.x = std::max(std::min(wrenchProp.force.x, 2.0*parentObj->kinematics.mass*9.81), -2.0*parentObj->kinematics.mass*9.81);
	wrenchProp.torque.x = propPower / omega;
	if (inputMotor < 0.01) {
		wrenchProp.force.x = 0;
		wrenchProp.torque.x = 0;
	} // To avoid aircraft rolling and turning on the ground while throttle is off
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;
	// double deltaP = parentObj->kinematics.forceInput.x * parentObj->states.velocity.linear.x / npCoeff;
	double deltaT = (engPower - propPower)/omega;
	double omegaDot = 1/engInertia*deltaT;
	omega += rotationDir * omegaDot * parentObj->dt;
	omega = rotationDir * std::max(std::min(std::fabs(omega), omegaMax), omegaMin); // Constrain omega to working range

	parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message

	// Printouts
	// std::cout << deltat << " ";
	// std::cout << powerHP << " ";
	// std::cout << engPower << " ";
	// std::cout << propPower << " ";
	// std:: cout << advRatio << " ";
	// std::cout << npCoeff << " ";
	// std::cout << wrenchProp.force.x << " ";
	// std::cout << wrenchProp.torque.x << " ";
	// std::cout << parentObj->kinematics.forceInput.x << " ";
	// std::cout << omegaDot << " ";
	// std::cout << omega;
	// std::cout << std::endl;

}

void PistonEng::getForce()
{
	if (!myisfinite(wrenchProp.force)) {ROS_FATAL("propulsion.cpp: State NaN in wrenchProp.force"); ros::shutdown();}
}

void PistonEng::getTorque()
{
	if (!myisfinite(wrenchProp.torque)) {ROS_FATAL("propulsion.cpp: State NaN in wrenchProp.torque"); ros::shutdown();}
}
