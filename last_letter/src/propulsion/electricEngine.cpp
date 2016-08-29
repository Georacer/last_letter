////////////////////////
// Electric engine model
////////////////////////

// Constructor
ElectricEng::ElectricEng(ModelPlane * parent, int ID) : Propulsion(parent, ID)
{
	XmlRpc::XmlRpcValue list;
	int i, length;
	char s[100];
	char paramMsg[50];
	sprintf(paramMsg, "motor%i/propDiam", id);
	if(!ros::param::getCached(paramMsg, propDiam)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/engInertia", id);
	if(!ros::param::getCached(paramMsg, engInertia)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/Kv", id);
	if(!ros::param::getCached(paramMsg, Kv)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/Rm", id);
	if(!ros::param::getCached(paramMsg, Rm)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "battery%i/Rs", id);
	if(!ros::param::getCached(paramMsg, Rs)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "battery%i/Cells", id);
	if(!ros::param::getCached(paramMsg, Cells)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/I0", id);
	if(!ros::param::getCached(paramMsg, I0)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	// Initialize RadPS limits
	sprintf(paramMsg, "motor%i/RadPSLimits", id);
	if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	omegaMin = list[0];
	omegaMax = list[1];

	Factory factory;
	// Create propeller efficiency polynomial
	sprintf(s,"motor%i/nCoeffPoly", id);
	npPoly =  factory.buildPolynomial(s);
	// Create propeller power polynomial
	sprintf(s,"motor%i/propPowerPoly", id);
	propPowerPoly =  factory.buildPolynomial(s);

	omega = omegaMin; // Initialize engine rotational speed

	wrenchProp.force.x = 0.0;
	wrenchProp.force.y = 0.0;
	wrenchProp.force.z = 0.0;
	wrenchProp.torque.x = 0.0;
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;

	sprintf(paramMsg, "propulsion%i", id);
	ros::NodeHandle n;
	pub = n.advertise<last_letter_msgs::ElectricEng>(paramMsg, 1000); //propulsion data publisher
}

// Destructor
ElectricEng::~ElectricEng()
{
	delete npPoly;
	delete propPowerPoly;
}

// Update motor rotational speed and calculate thrust
void ElectricEng::updateRadPS()
{

	rho = parentObj->environment.density;

	double Ei = std::fabs(omega)/2/M_PI/Kv;
	// double Ei = rotationDir * omega/2/M_PI/Kv;
	double Im = (Cells*4.0*inputMotor - Ei)/(Rs*inputMotor + Rm);
	// Im = std::max(Im,0.0); // Current cannot return to the ESC
	// Im = std::max(Im,-1.0); // Allow limited current back to the ESC
	double engPower = Ei*(Im - I0);

	// // Temporary measure
	// if (engPower<0) engPower=0;

	double advRatio = normalWind / (std::fabs(omega)/2.0/M_PI) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	// advRatio = std::max(advRatio, 0.0); // Force advance ratio above zero, in lack of a better propeller model
	double propPower = propPowerPoly->evaluate(advRatio) * parentObj->environment.density * pow(std::fabs(omega)/2.0/M_PI,3) * pow(propDiam,5);
	double npCoeff = npPoly->evaluate(advRatio);

	wrenchProp.force.x = propPower*std::fabs(npCoeff/(normalWind+1.0e-10)); // Added epsilon for numerical stability
	wrenchProp.force.y = 0.0;
	wrenchProp.force.z = 0.0;

	double fadeFactor = (exp(-normalWind*3/12));
	double staticThrust = 0.9*fadeFactor*pow(M_PI/2.0*propDiam*propDiam*rho*engPower*engPower,1.0/3); //static thrust fades at 5% at 12m/s
	wrenchProp.force.x = wrenchProp.force.x + staticThrust;

	// Constrain propeller force to [0,+5] times the aircraft weight
	wrenchProp.force.x = std::max(std::min(wrenchProp.force.x, 5.0*parentObj->kinematics.mass*9.81), 0.0*parentObj->kinematics.mass*9.81);
	wrenchProp.torque.x = engPower / omega; // I think engPower is the right choice, not propPower

	if (inputMotor < 0.01) {
		wrenchProp.force.x = 0;
		wrenchProp.torque.x = 0;
	} // To avoid aircraft rolling and turning on the ground while throttle is off
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;
	// double deltaP = parentObj->kinematics.forceInput.x * parentObj->states.velocity.linear.x / npCoeff;
	double deltaT = (engPower - propPower)/std::fabs(omega);
	double omegaDot = 1/engInertia*deltaT;
	omega += rotationDir * omegaDot * parentObj->dt;
	omega = rotationDir * std::max(std::min(std::fabs(omega), omegaMax), omegaMin); // Constrain omega to working range

	wrenchProp.torque.y = omega/100.0;
	parentObj->pubMotor.publish(wrenchProp); // Send calculated propeller angular velocity to Gazebo plugin

	parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message

	message.header.stamp = ros::Time::now();
	message.powerEng = propPower;
	message.omega = omega;
	message.throttle = inputMotor*100.0;
	message.powerProp = propPower;
	message.thrust = wrenchProp.force.x;
	message.torque = wrenchProp.torque.x;
	message.advRatio = advRatio;
	message.airspeed = normalWind;
	message.ncoeff = npCoeff;
	pub.publish(message);

	// Printouts
	// std::cout << deltat << " ";
	// std::cout << powerHP << " ";
	// std::cout << engPower << " ";
	// std::cout << propPower << " ";
	// std:: cout << advRatio << " ";
	// std::cout << npCoeff << " ";
	// std::cout << wrenchProp.force.x << " ";
	// std::cout << wrenchProp.force.z << " ";
	// std::cout << wrenchProp.torque.x << " ";
	// std::cout << parentObj->kinematics.forceInput.x << " ";
	// std::cout << omegaDot << " ";
	// std::cout << omega;
	// std::cout << tempVect.getX() << " ";
	// std::cout << tempVect.getZ() << " ";
	// std::cout << parentObj->input[3] << " ";
	// std::cout << std::endl;

}

void ElectricEng::getForce()
{
	if (!myisfinite(wrenchProp.force)) {ROS_FATAL("propulsion.cpp: State NaN in wrenchProp.force"); ros::shutdown();}
}

void ElectricEng::getTorque()
{
	if (!myisfinite(wrenchProp.torque)) {ROS_FATAL("propulsion.cpp: State NaN in wrenchProp.torque"); ros::shutdown();}
}