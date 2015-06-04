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

	// Collect forces from underlying models
	geometry_msgs::Vector3 Dynamics::getForce()
	{
		geometry_msgs::Vector3 tempVect;
		tempVect = gravity->getForce();
		for (int i=0; i<nMotors; i++){
			tempVect = propulsion[i]->getForce() + tempVect;
		}
		tempVect = groundReaction->getForce() + tempVect;
		tempVect = aerodynamics->getForce() + tempVect;

		return tempVect;
	}

	// Collect torques from underlying models
	geometry_msgs::Vector3 Dynamics::getTorque()
	{
		geometry_msgs::Vector3 tempVect;
		tempVect = aerodynamics->getTorque();
		tempVect = gravity->getTorque() + tempVect;
		for (int i=0; i<nMotors; i++){
			tempVect = propulsion[i]->getTorque() + tempVect;
		}
		tempVect = groundReaction->getTorque() + tempVect;

		return tempVect;
	}

