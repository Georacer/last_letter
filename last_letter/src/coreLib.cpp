// Core class definitions

using namespace std;

//////////////////////////
// Define ModelPlane class
//////////////////////////

///////////////////
//Class Constructor
ModelPlane::ModelPlane (ros::NodeHandle n) :
kinematics(this), dynamics(this), airdata(this)
{
	if(!ros::param::getCached("/world/deltaT", dt)) {ROS_FATAL("Invalid parameters for -deltaT- in param server!"); ros::shutdown();}
	init();
	tprev = ros::Time::now();
	states.header.stamp = tprev;
	//Subscribe and advertize
	subInp = n.subscribe("ctrlPWM",1,&ModelPlane::getInput, this); //model control input subscriber
	subEnv = n.subscribe("environment",1,&ModelPlane::getEnvironment, this); //dynamic environment effects subscriber
	pubState = n.advertise<last_letter_msgs::SimStates>("states",1000); //model states publisher
	pubForce = n.advertise<geometry_msgs::Vector3>("forceInput",1000); // forces publisher
	pubTorque = n.advertise<geometry_msgs::Vector3>("torqueInput",1000); // torques publisher
	pubLinAcc = n.advertise<geometry_msgs::Vector3>("linearAcc",1000); // Body frame linear acceleration - no corriolis effect
}

//Initialize states
void ModelPlane::init()
{
	XmlRpc::XmlRpcValue list;
	double temp[4];
	int i;

	// Set states message frame name
	states.header.frame_id = "bodyFrame";

	// Read initial NED coordinates
	if(!ros::param::getCached("init/position", list)) {ROS_FATAL("Invalid parameters for -init/position- in param server!"); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		temp[i]=list[i];
	}
	states.pose.position.x=temp[0];
	states.pose.position.y=temp[1];
	states.pose.position.z=temp[2];

	// Read initial orientation quaternion
	if(!ros::param::getCached("init/orientation", list)) {ROS_FATAL("Invalid parameters for -init/orientation- in param server!"); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		temp[i]=list[i];
	}
	states.pose.orientation.x=temp[0];
	states.pose.orientation.y=temp[1];
	states.pose.orientation.z=temp[2];
	states.pose.orientation.w=temp[3];

	// Read initial velocity
	if(!ros::param::getCached("init/velLin", list)) {ROS_FATAL("Invalid parameters for -init/velLin- in param server!"); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		temp[i]=list[i];
	}
	states.velocity.linear.x=temp[0];
	states.velocity.linear.y=temp[1];
	states.velocity.linear.z=temp[2];

	// Read initial angular velocity
	if(!ros::param::getCached("init/velAng", list)) {ROS_FATAL("Invalid parameters for -init/velAng- in param server!"); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		temp[i]=list[i];
	}
	states.velocity.angular.x=temp[0];
	states.velocity.angular.y=temp[1];
	states.velocity.angular.z=temp[2];

	// Initialize rotorspeed array
	states.rotorspeed.clear();
	states.rotorspeed.push_back((double) 0);

	// Initialize WGS coordinates
	if(!ros::param::getCached("init/coordinates", list)) {ROS_FATAL("Invalid parameters for -init/coordinates- in param server!"); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		temp[i]=list[i];
	}
	states.geoid.latitude = temp[0];
	states.geoid.longitude = temp[1];
	states.geoid.altitude = temp[2] - states.pose.position.z;

	// Initialize control inputs
	input[0] = 0;
	input[1] = 0;
	input[2] = 0;
	input[3] = 0;
	input[5] = 0;

	// Initialize environment
	environment.wind.x = 0;
	environment.wind.y = 0;
	environment.wind.z = 0;
	// Read initial environmental data from parameter server
	if(!ros::param::getCached("/environment/rho", environment.density)) {ROS_FATAL("Invalid parameters for -/environment/rho- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("/environment/groundPres", environment.pressure)) {ROS_FATAL("Invalid parameters for -/environment/groundPress- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("/environment/groundTemp", environment.temperature)) {ROS_FATAL("Invalid parameters for -/environment/groundTemp- in param server!"); ros::shutdown();}
	// Set initial gravity in case environment node isn't up yet
	environment.gravity = 9.81;
}

///////////////////////////////////////
//Make one step of the plane simulation
void ModelPlane::step(void)
{
	// Perform step actions serially
	dynamics.propulsion->updateRadPS();
	kinematics.forceInput = dynamics.getForce();
	kinematics.torqueInput = dynamics.getTorque();
	kinematics.calcDerivatives();
	kinematics.integrator->propagation();

	tprev = ros::Time::now();
	states.header.stamp = tprev;

	airdata.calcAirData();
	//publish results
	pubState.publish(states);
	pubForce.publish(kinematics.forceInput);
	pubTorque.publish(kinematics.torqueInput);
	pubLinAcc.publish(kinematics.linearAcc);
}

/////////////////////////////////////////////////
//convert uS PWM values to control surface inputs
void ModelPlane::getInput(last_letter_msgs::SimPWM inputMsg)
{
	if(!ros::param::getCached("airframe/deltaa_max", deltaa_max)) {ROS_FATAL("Invalid parameters for -airframe/deltaa_max- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/deltae_max", deltae_max)) {ROS_FATAL("Invalid parameters for -airframe/deltae_max- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/deltar_max", deltar_max)) {ROS_FATAL("Invalid parameters for -airframe/deltar_max- in param server!"); ros::shutdown();}
	//Convert PPM to radians (0/1 for throttle)
	input[0] = deltaa_max * (double)(inputMsg.value[0]-1500)/500;
	input[1] = deltae_max * (double)(inputMsg.value[1]-1500)/500;
	input[2] = (double)(inputMsg.value[2]-1000)/1000;
	input[3] = deltar_max * (double)(inputMsg.value[3]-1500)/500;
	input[5] = (double)(inputMsg.value[5]-1000)/1000;
	// std::cout << input[5] << ',';
	// std::cout << std::endl;
	if (inputMsg.value[9] > 1500) {
		init();
	}
}

/////////////////////////////////////////////////
//Store environmental values
void ModelPlane::getEnvironment(last_letter_msgs::Environment envUpdate)
{
	environment = envUpdate;
}

//////////////////
//Class destructor
ModelPlane::~ModelPlane ()
{
}


//////////////////////////
// Define Airdata class
//////////////////////////

//Class Constructor
Airdata::Airdata(ModelPlane * parent)
{
	parentObj = parent;
	airspeed = 0;
	alpha = 0;
	beta = 0;
}

//Class Destructor
Airdata::~Airdata()
{
	delete parentObj;
}

//Caclulate airspeed and aerodynamics angles
void Airdata::calcAirData()
{
	double u = parentObj->states.velocity.linear.x;
	double v = parentObj->states.velocity.linear.y;
	double w = parentObj->states.velocity.linear.z;

	airspeed = sqrt(pow(u,2)+pow(v,2)+pow(w,2));
	alpha = atan2(w,u);

	if (u==0) {
		if (v==0) {
			beta=0;
		}
		else {
			beta=asin(v/abs(v));
		}

	}
	else {
		beta = atan2(v,u);
	}
}

//////////////////////////
// Define Factory class
//////////////////////////

//Build integrator model
Integrator * Factory::buildIntegrator(ModelPlane * parent)
{
	int i;
	if(!ros::param::getCached("/world/integratorType", i)) {ROS_FATAL("Invalid parameters for -integratorType- in param server!"); ros::shutdown();}
	std::cout<< "building integrator class: ";
	switch (i)
	{
	case 0:
		std::cout << "selecting Forward Euler" << std::endl;
		return new ForwardEuler(parent);
	default:
		ROS_FATAL("Invalid integrator type!");
		ros::shutdown();
		break;
	}
}

// Build aerodynamics model
Aerodynamics * Factory::buildAerodynamics(ModelPlane * parent)
{
	int i;
	if(!ros::param::getCached("airframe/aerodynamicsType", i)) {ROS_FATAL("Invalid parameters for -aerodynamicsType- in param server!"); ros::shutdown();}
	std::cout<< "building aerodynamics model: ";
	switch (i)
	{
	case 0:
		std::cout << "selecting no aerodynamics model" << std::endl;
		return new NoAerodynamics(parent);
	case 1:
		std::cout << "selecting StdLinearAero aerodynamics" << std::endl;
		return new StdLinearAero(parent);
	case 2:
		std::cout << "selecting HCUAVAero aerodynamics" << std::endl;
		return new HCUAVAero(parent);
	default:
		ROS_FATAL("Error while constructing StdLinearAero aerodynamics");
		ros::shutdown();
		break;
	}
}

// Build engine model
Propulsion * Factory::buildPropulsion(ModelPlane * parent)
{
	int i;
	if(!ros::param::getCached("motor/motorType", i)) {ROS_FATAL("Invalid parameters for -motorType- in param server!"); ros::shutdown();}
	std::cout<< "building engine model: ";
	switch (i)
	{
	case 0:
		std::cout << "selecting no engine" << std::endl;
		return new NoEngine(parent);
	case 1:
		std::cout << "selecting Beard engine" << std::endl;
		return new EngBeard(parent);
	case 2:
		std::cout << "selecting piston engine" << std::endl;
		return new PistonEng(parent);
	case 3:
		std::cout << "selecting electric engine" << std::endl;
		return new ElectricEng(parent);
	default:
		ROS_FATAL("Error while constructing Beard motor");
		ros::shutdown();
		break;
	}
}

// Build ground reactions model
GroundReaction * Factory::buildGroundReaction(ModelPlane * parent)
{
	int i;
	if(!ros::param::getCached("airframe/groundReactionType", i)) {ROS_FATAL("Invalid parameters for -groundReactionType- in param server!"); ros::shutdown();}
	std::cout<< "building ground reactions model: ";
	switch (i)
	{
	case 0:
		std::cout << "selecting no ground reactions" << std::endl;
		return new NoGroundReaction(parent);
	case 1:
		std::cout << "selecting Panos ground reactions" << std::endl;
		return new PanosContactPoints(parent);
	default:
		ROS_FATAL("Error while constructing ground reactions");
		ros::shutdown();
		break;
	}
}

// Build a new polynomial, reading from the paramter server
Polynomial * Factory::buildPolynomial(char * baseParam)
{
	int i;
	XmlRpc::XmlRpcValue list;
	char parameter[100];
	sprintf(parameter, "%s/%s",baseParam,"polyType");
	if(!ros::param::getCached(parameter, i)) {ROS_FATAL("Invalid parameters for -*/polyType- in param server!"); ros::shutdown();}
	std::cout<< "building a new polynomial: ";
	switch (i)
	{
	case 0: {
		std::cout << "selecting 1D polynomial" << std::endl;
		sprintf(parameter, "%s/%s",baseParam,"polyNo");
		if(!ros::param::getCached(parameter, i)) {ROS_FATAL("Invalid parameters for -polyNo- in param server!"); ros::shutdown();}
		int polyNo = i;
		sprintf(parameter, "%s/%s",baseParam,"coeffs");
		if(!ros::param::getCached(parameter, list)) {ROS_FATAL("Invalid parameters for -coeffs- in param server!"); ros::shutdown();}
		double coeffs[polyNo+1];
		if (polyNo+1!=list.size()) {ROS_FATAL("Polynomial order and provided coefficient number do not match"); ros::shutdown();}
		for (i = 0; i <=polyNo; i++) {
			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			coeffs[i]=list[i];
		}
		return new Polynomial1D(polyNo, coeffs);
		}
	case 1: {
		std::cout << "selecting 2D polynomial" << std::endl;
		sprintf(parameter, "%s/%s",baseParam,"polyNo");
		if(!ros::param::getCached(parameter, list)) {ROS_FATAL("Invalid parameters for -polyNo- in param server!"); ros::shutdown();}
		ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeInt);
		int polyOrder1 = list[0];
		int polyOrder2 = list[1];
		sprintf(parameter, "%s/%s",baseParam,"coeffs");
		if(!ros::param::getCached(parameter, list)) {ROS_FATAL("Invalid parameters for -coeffs- in param server!"); ros::shutdown();}
		int length = list.size();
		if ((2*polyOrder2 + 2*polyOrder1*polyOrder2 + polyOrder1 - polyOrder1*polyOrder1 + 2)/2 != length) {
			ROS_FATAL("Engine power polynomial order and provided coefficient number do not match");
			ros::shutdown();
		}
		double coeffs[length];
		for (i = 0; i < list.size(); ++i) {
			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			coeffs[i]=list[i];
		}
		return new Polynomial2D(polyOrder1, polyOrder2, coeffs);
		}
	case 2: {
		std::cout << "selecting cubic spline" << std::endl;
		sprintf(parameter, "%s/%s",baseParam,"breaksNo");
		if(!ros::param::getCached(parameter, i)) {ROS_FATAL("Invalid parameters for -breaksNo- in param server!"); ros::shutdown();}
		int breaksNo = i;
		sprintf(parameter, "%s/%s",baseParam,"breaks");
		if(!ros::param::getCached(parameter, list)) {ROS_FATAL("Invalid parameters for -breaks- in param server!"); ros::shutdown();}
		if ((breaksNo+1) != list.size()) {
			ROS_FATAL("breaks order and provided breaks number do not match");
			ros::shutdown();
		}
		double breaks[list.size()];
		for (i = 0; i < list.size(); ++i) {
			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			breaks[i]=list[i];
		}
		sprintf(parameter, "%s/%s",baseParam,"coeffs");
		if(!ros::param::getCached(parameter, list)) {ROS_FATAL("Invalid parameters for -coeffs- in param server!"); ros::shutdown();}
		if (breaksNo*4 != list.size()) {
			ROS_FATAL("breaks order and provided coeffs number do not match");
			ros::shutdown();
		}
		double coeffs[list.size()];
		for (i = 0; i < list.size(); ++i) {
			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			coeffs[i]=list[i];
		}
		return new Spline3(breaksNo, breaks, coeffs);
		}
	default: {
		ROS_FATAL("Error while constructing a polynomial");
		ros::shutdown();
		break;
		}
	}
}