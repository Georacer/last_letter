//////////////////////////
// Define Dynamics class
//////////////////////////

	///////////////////
	//Class Constructor
	Dynamics::Dynamics(ModelPlane * parent)
	{
		parentObj = parent;
		Factory factory;
		// Create and initialize aerodynamic objects array
		if(!ros::param::getCached("airfoil/nWings", nWings)) {ROS_FATAL("Invalid parameters for -airfoil/nWings- in param server!"); ros::shutdown();}
		aerodynamics = new Aerodynamics*[nWings];
		for (int i=0; i<nWings; i++) {
			aerodynamics[i] = factory.buildAerodynamics(parentObj,i+1); // Create a new aerodynamics object, id's are 1-indexed
		}

		// Create and initialize gravity object
		gravity = new Gravity(parentObj);

		// Create and initialize motor objects array
		if(!ros::param::getCached("motor/nMotors", nMotors)) {ROS_FATAL("Invalid parameters for -motor/nMotors- in param server!"); ros::shutdown();}
		propulsion = new Propulsion*[nMotors];
		for (int i=0; i<nMotors; i++) {
			propulsion[i] = factory.buildPropulsion(parentObj,i+1); // Create a new propulsion object, id's are 1-indexed
		}

		// Create and initialize ground reactions object
		groundReaction = factory.buildGroundReaction(parentObj);
	}

	//Class Destructor
	Dynamics::~Dynamics()
	{
		for (int i=0; i<nWings; i++){
			delete aerodynamics[i]; // delete all aerodynamics objects
		}
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
		for (int i=0; i<nWings; i++) {
			aerodynamics[i]->getInput();
		}
		groundReaction->getInput();
	}

	// Calculate the forces and torques for each wrench source
	void Dynamics::calcWrench()
	{
		geometry_msgs::Vector3 tempVect;

		// Unneded, Gazebo takes care of them
		//
		// // Call gravity calculation routines
		// forceGrav = gravity->getForce();
		// if (isnan(forceGrav)) {ROS_FATAL("dynamicsLib.cpp: NaN member in gravity force vector"); ros::shutdown();}

		// torqueGrav = gravity->getTorque();
		// if (isnan(torqueGrav)) {ROS_FATAL("dynamicsLib.cpp: NaN member in gravity torque vector"); ros::shutdown();}


		// Call  motors routines

		// Execute one step in the motor dynamics
		for (int i=0; i<nMotors; i++) {
			propulsion[i]->stepEngine();
		}

		forceProp.x = 0; forceProp.y = 0; forceProp.z = 0;
		for (int i=0; i<nMotors; i++){
			tempVect = propulsion[i]->wrenchProp.force;
			if (isnan(tempVect)) {ROS_FATAL("dynamicsLib.cpp: NaN member in propulsion%i force vector",i+1); ros::shutdown();}
			forceProp = tempVect + forceProp;
		}

		torqueProp.x = 0; torqueProp.y = 0; torqueProp.z = 0;
		for (int i=0; i<nMotors; i++){
			tempVect = propulsion[i]->wrenchProp.torque;
			if (isnan(tempVect)) {ROS_FATAL("dynamicsLib.cpp: NaN member in propulsion%i torque vector",i+1); ros::shutdown();}
			torqueProp = tempVect + torqueProp;
			// std::cout << " ||propulsion " << i << ": " << tempVect.x << " " << tempVect.y << " " << tempVect.z << std::endl;
		}
		// std::cout << "accumulator: " << torqueProp.x << " " << torqueProp.y << " " << torqueProp.z << std::endl;
		// std::cout << std::endl;


		// Call  aerodynamics routines

		// Execute one step in the aerodynamics
		for (int i=0; i<nWings; i++) {
			aerodynamics[i]->stepDynamics();
		}

		forceAero.x = 0; forceAero.y = 0; forceAero.z = 0;
		for (int i=0; i<nWings; i++){
			tempVect = aerodynamics[i]->wrenchAero.force;
			if (isnan(tempVect)) {ROS_FATAL("dynamicsLib.cpp: NaN member in airfoil%i force vector",i+1); ros::shutdown();}
			forceAero = tempVect + forceAero;
			// std::cout << " ||airfoil " << i << ": " << tempVect.z;
		}
		// std::cout << std::endl;
		// std::cout << "aerodynamics accumulator: " << forceAero.x << " " << forceAero.y << " " << forceAero.z << std::endl;

		torqueAero.x = 0; torqueAero.y = 0; torqueAero.z = 0;
		for (int i=0; i<nWings; i++){
			tempVect = aerodynamics[i]->wrenchAero.torque;
			if (isnan(tempVect)) {ROS_FATAL("dynamicsLib.cpp: NaN member in airfoil%i torque vector",i+1); ros::shutdown();}
			torqueAero = tempVect + torqueAero;
			// std::cout << " ||propulsion " << i << ": " << tempVect.x << " " << tempVect.y << " " << tempVect.z << std::endl;
		}
		// std::cout << "accumulator: " << torqueProp.x << " " << torqueProp.y << " " << torqueProp.z << std::endl;
		// std::cout << std::endl;

		geometry_msgs::Wrench GazeboWrenchAero;
		GazeboWrenchAero.force.x = forceAero.x;
		GazeboWrenchAero.force.y = -forceAero.y;
		GazeboWrenchAero.force.z = -forceAero.z;
		GazeboWrenchAero.torque.x = torqueAero.x;
		GazeboWrenchAero.torque.y = -torqueAero.y;
		GazeboWrenchAero.torque.z = -torqueAero.z;
		parentObj->pubAero.publish(GazeboWrenchAero);

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

