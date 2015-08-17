////////////////////////////////////////
// Engine model found in R. Beard's book
////////////////////////////////////////

// Constructor
EngBeard::EngBeard(ModelPlane * parent, int ID):Propulsion(parent, ID)
{
	std::cout << "reading parameters for new Beard engine" << std::endl;
	omega = 0; // Initialize engine rotational speed
	// Read engine data from parameter server
	char paramMsg[50];
	sprintf(paramMsg, "motor%i/s_prop", id);
	if(!ros::param::getCached(paramMsg, s_prop)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/c_prop", id);
	if(!ros::param::getCached(paramMsg, c_prop)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/k_motor", id);
	if(!ros::param::getCached(paramMsg, k_motor)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/k_t_p", id);
	if(!ros::param::getCached(paramMsg, k_t_p)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/k_omega", id);
	if(!ros::param::getCached(paramMsg, k_omega)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
}

// Destructor
EngBeard::~EngBeard()
{
}

// Update motor rotational speed and other states for each timestep
void EngBeard::updateRadPS()
{
	rho = parentObj->environment.density;
	airspeed = normalWind; // Read vehicle airspeed
	if (!std::isfinite(airspeed)) {ROS_FATAL("propulsion.cpp: EngBeard airspeed is not finite"); ros::shutdown();}
	// Propagate rotational speed with a first order response
	// omega = rotationDir * 1 / (0.5 + parentObj->dt) * (0.5 * omega + parentObj->dt * inputMotor * k_omega);
	omega = rotationDir * inputMotor * k_omega;
	parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message
}

// Calculate propulsion forces
void EngBeard::getForce()
{
	wrenchProp.force.x = 1.0/2.0*rho*s_prop*c_prop*(pow(inputMotor * k_motor,2)-pow(airspeed,2));
	wrenchProp.force.y = 0;
	wrenchProp.force.z = 0;
	if (!myisfinite(wrenchProp.force)) {ROS_FATAL("propulsion.cpp/EngBeard: State NaN in wrenchProp.force"); ros::shutdown();}
}

// Calculate propulsion torques
void EngBeard::getTorque()
{
	wrenchProp.torque.x = -rotationDir * k_t_p*pow(omega,2);
	// wrenchProp.torque.x = 0;
	wrenchProp.torque.y = 0;
	wrenchProp.torque.z = 0;
	if (!myisfinite(wrenchProp.torque)) {ROS_FATAL("propulsion.cpp/EngBeard: State NaN in wrenchProp.torque"); ros::shutdown();}
}