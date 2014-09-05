//////////////////////////////
// Propulsion interfrace class 
//////////////////////////////

// Constructor
Propulsion::Propulsion(ModelPlane * parent)
{
	parentObj = parent;
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

	return wrenchProp.torque;
}
