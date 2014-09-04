//////////////////////////
// Define Gravity class
//////////////////////////

// Class constructor
Gravity::Gravity(ModelPlane * parent)
{
	parentObj = parent;
	// Read Center of Lift coordinates
	if(!ros::param::getCached("airframe/col_x", col_x)) {ROS_FATAL("Invalid parameters for -col_x- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/col_y", col_y)) {ROS_FATAL("Invalid parameters for -col_y- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/col_z", col_z)) {ROS_FATAL("Invalid parameters for -col_z- in param server!"); ros::shutdown();}
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
	//calculate cog torque, r x F, where r is the distance of CoL from CoG
	// TODO: Split each force contribution at its point of application
	geometry_msgs::Vector3 gravTorque;
	geometry_msgs::Vector3 otherForces = parentObj->kinematics.forceInput - wrenchGrav.force;
	wrenchGrav.torque.x = col_y*otherForces.z - col_z*otherForces.y;
	wrenchGrav.torque.y = -col_x*otherForces.z + col_z*otherForces.x;
	wrenchGrav.torque.z = -col_y*otherForces.x + col_x*otherForces.y;
	return wrenchGrav.torque;
}