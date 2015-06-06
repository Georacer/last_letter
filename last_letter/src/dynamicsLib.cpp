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
		geometry_msgs::Vector3 accumulator;
		accumulator = gravity->getForce();
		for (int i=0; i<nMotors; i++){
			accumulator = propulsion[i]->getForce() + accumulator;
		}
		accumulator = groundReaction->getForce() + accumulator;
		accumulator = aerodynamics->getForce() + accumulator;

		return accumulator;
	}

	// Collect torques from underlying models
	geometry_msgs::Vector3 Dynamics::getTorque()
	{
		geometry_msgs::Vector3 tempVect, accumulator;
		accumulator = aerodynamics->getTorque();
		accumulator = gravity->getTorque() + accumulator;
		for (int i=0; i<nMotors; i++){
			accumulator = propulsion[i]->getTorque() + accumulator;
		}
		accumulator = groundReaction->getTorque() + accumulator;

		return accumulator;
	}

