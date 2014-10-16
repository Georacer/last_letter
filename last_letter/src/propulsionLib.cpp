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

	std::cout << "reading parameters for new Beard engine" << std::endl;
	if(!ros::param::getCached("motor/propDiam", propDiam)) {ROS_FATAL("Invalid parameters for -propDiam- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/engInertia", engInertia)) {ROS_FATAL("Invalid parameters for -engInertia- in param server!"); ros::shutdown();}
	// Initialize RadPS limits
	if(!ros::param::getCached("motor/RadPSLimits", list)) {ROS_FATAL("Invalid parameters for -RadPSLimits- in param server!"); ros::shutdown();}
	ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	omegaMin = list[0];
	omegaMax = list[1];

	// Create engine power polynomial
	if(!ros::param::getCached("motor/powerPolyNo", list)) {ROS_FATAL("Invalid parameters for -powerPolyNo- in param server!"); ros::shutdown();}
	ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeInt);
	int powerPolyOrder1 = list[0];
	int powerPolyOrder2 = list[1];
	if(!ros::param::getCached("motor/powerPoly", list)) {ROS_FATAL("Invalid parameters for -powerPoly- in param server!"); ros::shutdown();}
	length = list.size();
	if ((2*powerPolyOrder2 + 2*powerPolyOrder1*powerPolyOrder2 + powerPolyOrder1 - powerPolyOrder1*powerPolyOrder1 + 2)/2 != length) {
		ROS_FATAL("Engine power polynomial order and provided coefficient number do not match");
		ros::shutdown();
	}
	double temp[length];
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		temp[i]=list[i];
	}
	powerPoly = new Polynomial2D(powerPolyOrder1, powerPolyOrder2, temp);

	// Create propeller efficiency polynomial
	if(!ros::param::getCached("motor/nCoeffPolyNo", i)) {ROS_FATAL("Invalid parameters for -nCoeffPolyNo- in param server!"); ros::shutdown();}
	int nCoeffPolyNo = i;
	if(!ros::param::getCached("motor/nCoeffPoly", list)) {ROS_FATAL("Invalid parameters for -nCoeffPoly- in param server!"); ros::shutdown();}
	double temp2[nCoeffPolyNo+1];
	if (nCoeffPolyNo+1!=list.size()) {ROS_FATAL("Propeller efficiency polynomial order and provided coefficient number do not match"); ros::shutdown();}
	for (i = 0; i <=nCoeffPolyNo; i++) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		temp2[i]=list[i];
	}
	npPoly = new Polynomial1D(nCoeffPolyNo, temp2);

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
	delete powerPoly;
}

// Update motor rotational speed and calculate thrust
void PistonEng::updateRadPS()
{
	deltat = parentObj->input[2];

	// omega = omegaMin + deltat*(omegaMax-omegaMin); // For use for direct RPM control
	// double power = powerPoly->evaluate(1.0, omega); // Calculate current engine power // For use for direct RPM control

	double power = deltat * powerPoly->evaluate(1.0, omega); // Calculate current engine power // For use for direct RPM control
	// std::cout << deltat << " ";
	// std::cout << power << " ";
	// double advRatio = parentObj->states.velocity.linear.x/ (omega/2.0/M_PI) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	// std:: cout << advRatio << " ";
	// double npCoeff = npPoly->evaluate(advRatio);
	// std::cout << npCoeff << " ";
	// wrenchProp.force.x = power*npCoeff/(parentObj->states.velocity.linear.x+1.0e-10); // Added epsilon for numerical stability
	wrenchProp.force.x = power/(parentObj->states.velocity.linear.x+1.0e-10); // Added epsilon for numerical stability - Eff dem propellers!

	// std::cout << wrenchProp.force.x << " ";
	// std::cout << parentObj->kinematics.forceInput.x << " ";
	// Constrain propeller force to +-2 times the aircraft weight
	// wrenchProp.force.x = std::max(std::min(wrenchProp.force.x, 2.0*parentObj->kinematics.mass*9.81), -2.0*parentObj->kinematics.mass*9.81);
	// std::cout << wrenchProp.force.x << " " << std::endl;
	wrenchProp.torque.x = power / omega;
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;
	// double deltaP = parentObj->kinematics.forceInput.x * parentObj->states.velocity.linear.x / npCoeff;
	double deltaP = parentObj->kinematics.forceInput.x * parentObj->states.velocity.linear.x ;
	double omegaDot = 1/engInertia*deltaP/omega;
	// std::cout << omegaDot << " ";
	omega += omegaDot*parentObj->dt;
	omega = std::max(std::min(omega, omegaMax), omegaMin); // Constrain omega to working range
	// std::cout << omega;
	// std::cout << std::endl;
	parentObj->states.rotorspeed[0]=omega; // Write engine speed to states message

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