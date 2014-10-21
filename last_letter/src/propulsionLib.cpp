//////////////////////////////
// Propulsion interfrace class 
//////////////////////////////

// Constructor
Propulsion::Propulsion(ModelPlane * parent)
{
	parentObj = parent;
	XmlRpc::XmlRpcValue list;
	int i;
	if(!ros::param::getCached("motor/CGOffset", list)) {ROS_FATAL("Invalid parameters for -motor/CGOffset- in param server!"); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	}
	CGOffset.x = list[0];
	CGOffset.y = list[1];
	CGOffset.z = list[2];
}

// Destructor
Propulsion::~Propulsion()
{
	delete parentObj;
}

//////////////////
// No Engine Model
//////////////////

// Constructor
NoEngine::NoEngine(ModelPlane * parent) : Propulsion(parent)
{	
	wrenchProp.force.x = 0.0;
	wrenchProp.force.y = 0.0;
	wrenchProp.force.z = 0.0;
	wrenchProp.torque.x = 0.0;
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;
	omega = 0.0;
}

// Destructor
NoEngine::~NoEngine()
{
}

void NoEngine::updateRadPS()
{
}

// Force calculation function
geometry_msgs::Vector3 NoEngine::getForce()
{
	return wrenchProp.force;
}

// Torque calculation function
geometry_msgs::Vector3 NoEngine::getTorque()
{
	return wrenchProp.torque;
}

////////////////////////////////////////
// Engine model found in R. Beard's book
////////////////////////////////////////

// Constructor
EngBeard::EngBeard(ModelPlane * parent):Propulsion(parent)
{
	std::cout << "reading parameters for new Beard engine" << std::endl;
	omega = 0; // Initialize engine rotational speed
	// Read engine data from parameter server
	if(!ros::param::getCached("motor/s_prop", s_prop)) {ROS_FATAL("Invalid parameters for -s_prop- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/c_prop", c_prop)) {ROS_FATAL("Invalid parameters for -c_prop- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_motor", k_motor)) {ROS_FATAL("Invalid parameters for -k_motor- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_t_p", k_t_p)) {ROS_FATAL("Invalid parameters for -k_t_p- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_omega", k_omega)) {ROS_FATAL("Invalid parameters for -k_omega- in param server!"); ros::shutdown();}
}

// Destructor
EngBeard::~EngBeard()
{
}

// Update motor rotational speed and other states for each timestep
void EngBeard::updateRadPS()
{
	rho = parentObj->environment.density;
	deltat = parentObj->input[2]; // Read thrust command input
	airspeed = parentObj->airdata.airspeed; // Read vehicle airspeed
	// Propagate rotational speed with a first order response
	omega = 1 / (0.5 + parentObj->dt) * (0.5 * omega + parentObj->dt * deltat * k_omega); // Maximum omega set to 300 rad/s
	parentObj->states.rotorspeed[0]=omega; // Write engine speed to states message
}

// Calculate propulsion forces
geometry_msgs::Vector3 EngBeard::getForce()
{
	wrenchProp.force.x = 1.0/2.0*rho*s_prop*c_prop*(pow(omega * k_motor,2)-pow(airspeed,2));
	wrenchProp.force.y = 0;
	wrenchProp.force.z = 0;

	return wrenchProp.force;
}

// Calculate propulsion torques
geometry_msgs::Vector3 EngBeard::getTorque()
{
	wrenchProp.torque.x = -k_t_p*pow(omega,2);
	wrenchProp.torque.y = 0;
	wrenchProp.torque.z = 0;

	// Add torque to to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	// Will potentially add the following code in the future, to support shift of CoG mid-flight
	// XmlRpc::XmlRpcValue list;
	// int i;
	// if(!ros::param::getCached("motor/CGOffset", list)) {ROS_FATAL("Invalid parameters for -/motor/CGOffset- in param server!"); ros::shutdown();}
	// for (i = 0; i < list.size(); ++i) {
	// 	ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	// 	CGOffset[i]=list[i];
	// }
	wrenchProp.torque.x = wrenchProp.torque.x + CGOffset.y*wrenchProp.force.z - CGOffset.z*wrenchProp.force.y;
	wrenchProp.torque.y = wrenchProp.torque.y - CGOffset.x*wrenchProp.force.z + CGOffset.z*wrenchProp.force.x;
	wrenchProp.torque.z = wrenchProp.torque.z - CGOffset.y*wrenchProp.force.x + CGOffset.x*wrenchProp.force.y;

	return wrenchProp.torque;
}

///////////////////////////////////////////////////
// Piston engine model (based upon Zanzoterra 305i)
///////////////////////////////////////////////////

// Constructor
PistonEng::PistonEng(ModelPlane * parent) : Propulsion(parent)
{
	XmlRpc::XmlRpcValue list;
	int i, length;
	char s[100];
	if(!ros::param::getCached("motor/propDiam", propDiam)) {ROS_FATAL("Invalid parameters for -propDiam- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/engInertia", engInertia)) {ROS_FATAL("Invalid parameters for -engInertia- in param server!"); ros::shutdown();}
	// Initialize RadPS limits
	if(!ros::param::getCached("motor/RadPSLimits", list)) {ROS_FATAL("Invalid parameters for -RadPSLimits- in param server!"); ros::shutdown();}
	ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	omegaMin = list[0];
	omegaMax = list[1];

	Factory factory;
	// Create engine power polynomial
	sprintf(s,"%s","motor/engPowerPoly");
	engPowerPoly =  factory.buildPolynomial(s);
	// Create propeller efficiency polynomial
	sprintf(s,"%s","motor/nCoeffPoly");
	npPoly =  factory.buildPolynomial(s);
	// Create propeller power polynomial
	sprintf(s,"%s","motor/propPowerPoly");
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
	// delete engPowerPoly;
	delete propPowerPoly;
}

// Update motor rotational speed and calculate thrust
void PistonEng::updateRadPS()
{
	deltat = parentObj->input[2];

	double powerHP = engPowerPoly->evaluate(omega/2.0/M_PI*60);
	double engPower = deltat * powerHP * 745.7 * 2.0; // Calculate current engine power

	double advRatio = parentObj->states.velocity.linear.x/ (omega/2.0/M_PI) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	double propPower = propPowerPoly->evaluate(advRatio) * parentObj->environment.density * pow(omega/2.0/M_PI,3) * pow(propDiam,5);
	double npCoeff = npPoly->evaluate(advRatio);

	wrenchProp.force.x = propPower*npCoeff/(parentObj->states.velocity.linear.x+1.0e-10); // Added epsilon for numerical stability
	// wrenchProp.force.x = engPower/(parentObj->states.velocity.linear.x+1.0e-10); // Added epsilon for numerical stability - Eff dem propellers!

	// Constrain propeller force to +-2 times the aircraft weight
	wrenchProp.force.x = std::max(std::min(wrenchProp.force.x, 2.0*parentObj->kinematics.mass*9.81), -2.0*parentObj->kinematics.mass*9.81);
	wrenchProp.torque.x = propPower / omega;
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;
	// double deltaP = parentObj->kinematics.forceInput.x * parentObj->states.velocity.linear.x / npCoeff;
	double deltaT = (engPower - propPower)/omega;
	double omegaDot = 1/engInertia*deltaT;
	omega += omegaDot*parentObj->dt;
	omega = std::max(std::min(omega, omegaMax), omegaMin); // Constrain omega to working range

	parentObj->states.rotorspeed[0]=omega; // Write engine speed to states message

	// Printouts
	// std::cout << deltat << " ";
	// std::cout << powerHP << " ";
	// std::cout << engPower << " ";
	// std::cout << propPower << " ";
	// std:: cout << advRatio << " ";
	// std::cout << npCoeff << " ";
	// std::cout << wrenchProp.force.x << " ";
	// std::cout << parentObj->kinematics.forceInput.x << " ";
	// std::cout << omegaDot << " ";				
	// std::cout << omega;
	// std::cout << std::endl;

}

geometry_msgs::Vector3 PistonEng::getForce()
{
	if (isnan(wrenchProp.force.x) || isnan(wrenchProp.force.y) || isnan(wrenchProp.force.z)) {
		ROS_FATAL("State NaN in wrenchProp.force");
		ros::shutdown();
	}
	return wrenchProp.force;
}

geometry_msgs::Vector3 PistonEng::getTorque()
{
	if (isnan(wrenchProp.torque.x) || isnan(wrenchProp.torque.y) || isnan(wrenchProp.torque.z)) {
		ROS_FATAL("State NaN in wrenchProp.torque");
		ros::shutdown();
	}
	
	// Add torque to to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	// Will potentially add the following code in the future, to support shift of CoG mid-flight
	// XmlRpc::XmlRpcValue list;
	// int i;
	// if(!ros::param::getCached("motor/CGOffset", list)) {ROS_FATAL("Invalid parameters for -/motor/CGOffset- in param server!"); ros::shutdown();}
	// for (i = 0; i < list.size(); ++i) {
	// 	ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	// 	CGOffset[i]=list[i];
	// }
	wrenchProp.torque.x = wrenchProp.torque.x + CGOffset.y*wrenchProp.force.z - CGOffset.z*wrenchProp.force.y;
	wrenchProp.torque.y = wrenchProp.torque.y - CGOffset.x*wrenchProp.force.z + CGOffset.z*wrenchProp.force.x;
	wrenchProp.torque.z = wrenchProp.torque.z - CGOffset.y*wrenchProp.force.x + CGOffset.x*wrenchProp.force.y;

	return wrenchProp.torque;
}