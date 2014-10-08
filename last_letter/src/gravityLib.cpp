//////////////////////////
// Define Gravity class
//////////////////////////

// Class constructor
Gravity::Gravity(ModelPlane * parent)
{
	parentObj = parent;
	wrenchGrav.torque.x = 0;
	wrenchGrav.torque.y = 0;
	wrenchGrav.torque.z = 0;
}

// Class destructor
Gravity::~Gravity()
{
	delete parentObj;
}

// Force calculation function
geometry_msgs::Vector3 Gravity::getForce()
{
	double Reb[9];
	quat2rotmtx(parentObj->states.pose.orientation,Reb);
	g = parentObj->environment.gravity;
	geometry_msgs::Vector3 gravVect;
	gravVect.z = parentObj->kinematics.mass*g;
	wrenchGrav.force = Reb/gravVect;
	return wrenchGrav.force;
}

// Torque calculation function
geometry_msgs::Vector3 Gravity::getTorque()
{
	// Gravity does not generate torque around the CG
	return wrenchGrav.torque;
}