//////////////////////////
// Define Gravity class
//////////////////////////

	Gravity::Gravity(ModelPlane * parent)
	{
		parentObj = parent;
		if(!ros::param::getCached("airframe/col_x", col_x)) {ROS_FATAL("Invalid parameters for -col_x- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/col_y", col_y)) {ROS_FATAL("Invalid parameters for -col_y- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/col_z", col_z)) {ROS_FATAL("Invalid parameters for -col_z- in param server!"); ros::shutdown();}
	}
	
	Gravity::~Gravity()
	{
		delete parentObj;
	}
	
	geometry_msgs::Vector3 Gravity::getForce()
	{
		//calculate gravity force
		double Reb[9];
		quat2rotmtx(parentObj->states.pose.orientation,Reb);
		g = parentObj->environment.gravity;
		geometry_msgs::Vector3 gravVect;
		gravVect.z = parentObj->kinematics.mass*g;
		wrenchGrav.force = Reb/gravVect;
		return wrenchGrav.force;
	}
	
	geometry_msgs::Vector3 Gravity::getTorque()
	{
		//calculate cog torque, r x F, where r is the distance of CoL from CoG
		geometry_msgs::Vector3 gravTorque;
		geometry_msgs::Vector3 otherForces = parentObj->kinematics.forceInput - wrenchGrav.force;
		wrenchGrav.torque.x = col_y*otherForces.z - col_z*otherForces.y;
		wrenchGrav.torque.y = -col_x*otherForces.z + col_z*otherForces.x;
		wrenchGrav.torque.z = -col_y*otherForces.x + col_x*otherForces.y;
		return wrenchGrav.torque;
	}