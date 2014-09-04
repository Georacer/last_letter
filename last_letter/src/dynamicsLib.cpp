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
		propulsion = factory.buildPropulsion(parentObj);
		groundReaction = factory.buildGroundReaction(parentObj);
	}
	
	//Class Destructor
	Dynamics::~Dynamics()
	{
		delete aerodynamics;
		delete gravity;
		delete propulsion;
		delete groundReaction;
		delete parentObj;
	}
	
	// Collect forces from underlying models
	geometry_msgs::Vector3 Dynamics::getForce()
	{
		geometry_msgs::Vector3 tempVect;
		tempVect = propulsion->getForce();
		tempVect = groundReaction->getForce() + tempVect;
		tempVect = gravity->getForce() + tempVect;
		tempVect = aerodynamics->getForce() + tempVect;
		
		return tempVect;
	}
	
	// Collect torques from underlying models
	geometry_msgs::Vector3 Dynamics::getTorque()
	{
		geometry_msgs::Vector3 tempVect;
		tempVect = aerodynamics->getTorque();
		tempVect = gravity->getTorque() + tempVect;
		tempVect = propulsion->getTorque() + tempVect;
		tempVect = groundReaction->getTorque() + tempVect;
		
		return tempVect;
	}

