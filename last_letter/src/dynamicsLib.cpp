//////////////////////////
// Define Dynamics class
//////////////////////////

	///////////////////
	//Class Constructor
	Dynamics::Dynamics(ModelPlane * parent)
	{
		parentObj = parent;
		Factory factory;
		aerodynamics = factory.buildAerodynamics(parentObj);
		gravity = new Gravity(parentObj);

		// Create and initialize motor objects array
		if(!ros::param::getCached("motor/nMotors", nMotors)) {ROS_FATAL("Invalid parameters for -motor/nMotors- in param server!"); ros::shutdown();}
		propulsion = new Propulsion*[nMotors];
		for (int i=0; i<nMotors; i++) {
			propulsion[i] = factory.buildPropulsion(parentObj,i+1); // Create a new propulsion object, id's are 1-indexed
		}

		groundReaction = factory.buildGroundReaction(parentObj);
	}

	//Class Destructor
	Dynamics::~Dynamics()
	{
		delete aerodynamics;
		delete gravity;
		for (int i=0; i<nMotors; i++){
			delete propulsion[i]; // delete all propulsion objects
		}
		delete propulsion; // Must also separately delete the array of object pointers
		delete groundReaction;
		delete parentObj;
	}

	// Order subsystems to store control input
	void Dynamics::getInput()
	{
		for (int i=0; i<nMotors; i++) {
			propulsion[i]->getInput();
		}
		groundReaction->getInput();
		aerodynamics->getInput();
	}

	// Calculate the forces and torques for each wrench source
	void Dynamics::calcWrench()
	{

		geometry_msgs::Vector3 accumulator, tempVect;
		accumulator.x = 0;
		accumulator.y = 0;
		accumulator.z = 0;

		// Call gravity calculation routines
		forceGrav = gravity->getForce();
		if (isnan(forceGrav)) {ROS_FATAL("dynamicsLib.cpp: NaN member in gravity force vector"); ros::shutdown();}
		// std::cout << "gravity: " << forceGrav.x << " " << forceGrav.y << " " << forceGrav.z << std::endl;

		torqueGrav = gravity->getTorque();
		if (isnan(torqueGrav)) {ROS_FATAL("dynamicsLib.cpp: NaN member in gravity torque vector"); ros::shutdown();}


		// Call  motors routines
		forceProp.x = 0; forceProp.y = 0; forceProp.z = 0;
		for (int i=0; i<nMotors; i++){
			tempVect = propulsion[i]->wrenchProp.force;
			if (isnan(tempVect)) {ROS_FATAL("dynamicsLib.cpp: NaN member in propulsion%i force vector",i+1); ros::shutdown();}
			forceProp = tempVect + forceProp;
			// std::cout << " ||propulsion " << i << ": " << tempVect.z;
		}
		// std::cout << std::endl;
		// std::cout << "propulsion accumulator: " << forceProp.x << " " << forceProp.y << " " << forceProp.z << std::endl;

		torqueProp.x = 0; torqueProp.y = 0; torqueProp.z = 0;
		for (int i=0; i<nMotors; i++){
			tempVect = propulsion[i]->wrenchProp.torque;
			if (isnan(tempVect)) {ROS_FATAL("dynamicsLib.cpp: NaN member in propulsion%i torque vector",i+1); ros::shutdown();}
			torqueProp = tempVect + torqueProp;
			// std::cout << " ||propulsion " << i << ": " << tempVect.x << " " << tempVect.y << " " << tempVect.z << std::endl;
		}
		// std::cout << "accumulator: " << torqueProp.x << " " << torqueProp.y << " " << torqueProp.z << std::endl;
		// std::cout << std::endl;


		// Call aerodynamics routines
		forceAero = aerodynamics->getForce();
		if (isnan(forceAero)) {ROS_FATAL("dynamicsLib.cpp: NaN member in aerodynamics force vector"); ros::shutdown();}
		// std::cout << "aerodynamics: " << forceAero.x << " " << forceAero.y << " " << forceAero.z << std::endl;

		torqueAero = aerodynamics->getTorque();
		if (isnan(torqueAero)) {ROS_FATAL("dynamicsLib.cpp: NaN member in aerodynamics torque vector"); ros::shutdown();}


		// Call ground reactions routines - MUST BE CALLED LAST!!!
		forceGround = groundReaction->getForce();
		if (isnan(forceGround)) {ROS_FATAL("dynamicsLib.cpp: NaN member in groundReaction force vector"); ros::shutdown();}
		// std::cout << "groundReactions: " << forceGround.x << " " << forceGround.y << " " << forceGround.z << std::endl;

		torqueGround = groundReaction->getTorque();
		if (isnan(torqueGround)) {ROS_FATAL("dynamicsLib.cpp: NaN member in groundReaction torque vector"); ros::shutdown();}

	}

	// Collect forces from underlying models
	geometry_msgs::Vector3 Dynamics::getForce()
	{
		geometry_msgs::Vector3 accumulator;

		accumulator = forceGrav;
		accumulator = accumulator + forceProp;
		accumulator = accumulator + forceAero;
		accumulator = accumulator + forceGround;

		return accumulator;
	}

	// Collect torques from underlying models
	geometry_msgs::Vector3 Dynamics::getTorque()
	{
		geometry_msgs::Vector3 accumulator;

		accumulator = torqueGrav;
		accumulator = accumulator + torqueProp;
		accumulator = accumulator + torqueAero;
		accumulator = accumulator + torqueGround;

		return accumulator;
	}

