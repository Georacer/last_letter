/////////////////////////////////
// Define NoGroundReactions class
/////////////////////////////////

// Constructor
NoGroundReaction::NoGroundReaction(ModelPlane * parent) : GroundReaction(parent)
{
	wrenchGround.force.x = 0.0;
	wrenchGround.force.y = 0.0;
	wrenchGround.force.z = 0.0;
	wrenchGround.torque.x = 0.0;
	wrenchGround.torque.y = 0.0;
	wrenchGround.torque.z = 0.0;
}

// Destructor
NoGroundReaction::~NoGroundReaction()
{
}

// Force calculation function
geometry_msgs::Vector3 NoGroundReaction::getForce()
{
	return wrenchGround.force;
}

// Torque calculation function
geometry_msgs::Vector3 NoGroundReaction::getTorque()
{
	return wrenchGround.torque;
}
