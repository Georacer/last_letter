////////////////////////////////////////
// Define GroundReaction interface class
////////////////////////////////////////

// Class constructor
GroundReaction::GroundReaction(ModelPlane * parent)
{
	parentObj = parent;
	if(!ros::param::getCached("airframe/chanSteer", chanSteer)) {ROS_INFO("No STEER channel selected"); chanSteer=-1;}
	if(!ros::param::getCached("airframe/chanBrake", chanBrake)) {ROS_INFO("No BRAKE channel selected"); chanBrake-1;}
	if(!ros::param::getCached("airframe/steerAngle_max", steerAngle_max)) {ROS_INFO("No STEERANGLE_MAX value selected"); steerAngle_max=0.0;}

	inputSteer = 0.0;
	inputBrake = 0.0;
}

// Class destructor
GroundReaction::~GroundReaction()
{
	delete parentObj;
}

void GroundReaction::getInput()
{
	ros::param::getCached("airframe/steerAngle_max", steerAngle_max);
	if (chanSteer>-1) {inputSteer = steerAngle_max * (double)(parentObj->input.value[chanSteer]-1500)/500;}
	if (chanBrake>-1) {inputBrake = (double)(parentObj->input.value[chanBrake]-1000)/1000;}
}

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

//////////////////////////////////
// Define PanosContactPoints class
//////////////////////////////////

// Class constructor
PanosContactPoints::PanosContactPoints(ModelPlane * parent) : GroundReaction(parent)
{
	XmlRpc::XmlRpcValue list;
	int i, j, points;
	char paramMsg[50];
	double temp[6];
	// Read contact points number from parameter server
	if(!ros::param::getCached("airframe/contactPtsNo", contactPtsNo)) {ROS_FATAL("Invalid parameters for -/airframe/contactPtsNo- in param server!"); ros::shutdown();}
	// Create an appropriately sized matrix to contain contact point information
	pointCoords = (double*)malloc(sizeof(double) * contactPtsNo*3); // contact points coordinates in the body frame
	materialIndex = (double*)malloc(sizeof(double) * contactPtsNo*1); // contact points material type index
	springIndex = (double*)malloc(sizeof(double) * contactPtsNo*2); // contact points spring characteristics
	// Set spring characteristics
	// kspring = 4570.0;
	// mspring = 1300.0;
	len=0.2;

	// Set coefficient of friction for each material
	frictForw[0] = 0.7; frictSide[0] = 0.7;
	frictForw[1] = 0.4; frictSide[1] = 0.4;
	frictForw[2] = 0.1; frictSide[2] = 1.0;
	frictForw[3] = 0.4; frictSide[3] = 0.4; //To update composite to ground firction coefficients!

	// Read contact points location and material from parameter server
	for (j = 0; j<contactPtsNo; j++) { //Distribute the data
		sprintf(paramMsg, "airframe/contactPoint%i", j+1);
		if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -/airframe/contactPoint- in param server!"); ros::shutdown();}
		for (i = 0; i < list.size(); ++i) {
			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			temp[i]=list[i];
		}
		pointCoords[j] = temp[0]; // Save body frame contact point coordinates
		pointCoords[j + contactPtsNo] = temp[1];
		pointCoords[j + contactPtsNo*2] = temp[2];
		materialIndex[j] = temp[3]; // A separate contact point material index array
		springIndex[j] = temp[4]; // And the spring constants
		springIndex[j + contactPtsNo] = temp[5];
	}

	// Create and initialize spring contraction container
	spp = (double*)malloc(sizeof(double) * contactPtsNo);
	memset(spp, 0, sizeof(spp));

	// Create and initialize previous spring contraction container
	sppprev = (double*)malloc(sizeof(double) * contactPtsNo);
	memset(sppprev, 0, sizeof(sppprev));

	contact = false;

	// Create other matrices needed for calculations
	cpi_up = (double*)malloc(sizeof(double) * contactPtsNo*3); // upper spring end matrix
	cpi_down = (double*)malloc(sizeof(double) * contactPtsNo*3); // lower spring end matrix
	spd = (double*)malloc(sizeof(double) * contactPtsNo); // spring contraction speed
}

// Class destructor
PanosContactPoints::~PanosContactPoints()
{
	free(cpi_up);
	free(cpi_down);
	free(spd);
	free(spp);
	free(sppprev);
	free(pointCoords);
	free(materialIndex);
	free(springIndex);
}

// Wrench calculation function
geometry_msgs::Vector3 PanosContactPoints::getForce()
{
	double Reb[9];
	double kFLong, kFLat, kFrictionE, kFrictionX, kFrictionY;
	int i, j;
	contact = false;
	safe = true; // NaN protection flag
	geometry_msgs::Vector3 Euler;

	geometry_msgs::Quaternion quat = parentObj->states.pose.orientation; // Read vehicle orientation quaternion
	quat2rotmtx(quat, Reb); // Construct the rotation matrix
	Euler = quat2euler(quat);

	geometry_msgs::Wrench tempE, totalE; /** @todo This is a TODO list example */
	geometry_msgs::Vector3 dx,we,vpoint,Ve;

	totalE.force.x = 0;
	totalE.force.y = 0;
	totalE.force.z = 0;
	totalE.torque.x = 0;
	totalE.torque.y = 0;
	totalE.torque.z = 0;

	// Get velocity in the inertial frame
	Ve = parentObj->kinematics.posDot;

	// Read vehicle coordinates
	uavpos[0]=parentObj->states.pose.position.x;
	uavpos[1]=parentObj->states.pose.position.y;
	uavpos[2]=parentObj->states.pose.position.z;

	// Rotate contact points coordinates from body frame to earth frame
	multi_mtx_mtx_3Xn(Reb,pointCoords,cpi_up,contactPtsNo);

	// Place the spring ends to the contact points positions
	for (i=0;i<3;i++)
	{
		for (j=0;j<contactPtsNo;j++) {
			cpi_up[contactPtsNo*i+j] += uavpos[i]; // Place upper spring end to contact point
			if (i==2)
				cpi_up[contactPtsNo*i+j] -= len; //Raise the upper spring end to obtain a visually pleasing result in RViz
			cpi_down[contactPtsNo*i+j]=cpi_up[contactPtsNo*i+j]; // Place lower spring end to contact point
		}
	}

	// Rotate body angular speeds to earth frame
	we = Reb*parentObj->states.velocity.angular;

	// Main calculations for each contact point
	for (i=0;i<contactPtsNo;i++)
	{
		cpi_down[i+2*contactPtsNo]+=len; // Place lower spring end "len" below upper spring end
		// Calculate force arm
		dx.x = (cpi_up[i]-uavpos[0]);
		dx.y = (cpi_up[i+contactPtsNo]-uavpos[1]);
		dx.z = (cpi_up[i+2*contactPtsNo]-uavpos[2]);

		if (cpi_down[i+2*contactPtsNo]>0) // if the lower spring end touches the ground
		{
			cpi_down[i+2*contactPtsNo]=0; // Force lower spring end to stay at ground level
			contact=true;
			spp[i]=len+cpi_up[i+2*contactPtsNo]; // Calculate current spring contraction
			spd[i]=(spp[i]-sppprev[i])/parentObj->dt; // Calculate current spring contraction spreed
			vector3_cross(we,dx, &vpoint); // Contact point Earth-frame speed due to vehicle rotation
			vpoint = Ve+vpoint; // Add vehicle velocity to find total contact point velocity

			// Calculate spring force. Acts only upon Earth z-axis
			// tempE.force.z = -(kspring*spp[i] + mspring*spd[i]);
			tempE.force.z = -(springIndex[i]*spp[i] + springIndex[i + contactPtsNo]*spd[i]);
			// Make spring force negative or zero, as the spring lower end isn't fixed on the ground
			if (tempE.force.z > 0)
				tempE.force.z = 0;
			/** @todo model full spring contraction (possibly with exponential force curve after full contraction?) */

			int tempIndex = materialIndex[i];
			// Get longitudinal and lateral friction coefficients for this surface
			kFLong = frictForw[tempIndex];
			kFLat = frictSide[tempIndex];

			if ((inputBrake>0.9) && (i<3)) { // Apply breaks
				kFLong = 1.0;
			}

			// Convert friction coefficients to the Earth frame
			double trackAngle = Euler.z - atan2(vpoint.y, vpoint.x);
			if (i==2) { // Apply steeering
				trackAngle = trackAngle + inputSteer;
			}

			// Calculate the magnitude of the friction force in the Earth frame
			double frictionLong = fabs(sqrt(kFLong*(kFLong*cos(trackAngle)*cos(trackAngle)+kFLat*sin(trackAngle)*sin(trackAngle)))*tempE.force.z);
			frictionLong = std::max(frictionLong - 0.05*frictionLong/sqrt(vpoint.x*vpoint.x + vpoint.y*vpoint.y + 0.001), 0.0); //Aply static friction
			frictionLong = frictionLong*cos(trackAngle);

			double frictionLat = fabs(sqrt(kFLat*(kFLat*sin(trackAngle)*sin(trackAngle)+kFLong*cos(trackAngle)*cos(trackAngle)))*tempE.force.z);
			frictionLat = std::max(frictionLat - 0.05*frictionLat/sqrt(vpoint.x*vpoint.x + vpoint.y*vpoint.y + 0.001), 0.0); //Aply static friction
			frictionLat = frictionLat*sin(trackAngle);

			tempE.force.x = -frictionLong*cos(Euler.z)-frictionLat*sin(Euler.z);
			tempE.force.y = -frictionLong*sin(Euler.z)+frictionLat*cos(Euler.z);

			// Add current contact point force contribution to the total
			totalE.force = totalE.force + tempE.force;
			// Calculate current contact point torque
			vector3_cross(dx,tempE.force, &tempE.torque);
			// Add current contact point torque contribution to the total
			totalE.torque = totalE.torque + tempE.torque;

			if (i==2) {
				// std::cout << kFrictionLong << ',';
				// std::cout << trackAngle << ',';
				// std::cout << kFrictionE << ',';
				// std::cout << tempE.force.x << ',';
				// std::cout << tempE.force.y << ',';
				// // std::cout << tempB.force.z << ',';
				// std::cout << std::endl;
			}
		}

		// If there is no ground contact
		else {
			// Set spring contraction to zero
			spp[i] = 0;
			spd[i] = 0;
		}
		sppprev[i] = spp[i];
	}

	// Chech for rogue NaN results
	if (isnan(totalE.force.x) or isnan(totalE.force.y) or isnan(totalE.force.z)) {
		safe = false; /** @todo remove the safe safety, it is not used*/
		ROS_FATAL("State NAN in PanosGroundReactions force calculation!");
		ros::shutdown();
	}

	if (isnan(totalE.torque.x) or isnan(totalE.torque.y) or isnan(totalE.torque.z)) {
		safe = false;
		ROS_FATAL("State NAN in PanosGroundReactions torque calculation!");
		ros::shutdown();
	}

	// If there is a ground contact write the resulting wrench
	if (contact and safe) {
		wrenchGround.force= Reb/totalE.force;
		wrenchGround.torque= Reb/totalE.torque;
	}
	// Otherwise there is no wrench created
	else {
		wrenchGround.force.x = 0;
		wrenchGround.force.y = 0;
		wrenchGround.force.z = 0;
		wrenchGround.torque.x = 0;
		wrenchGround.torque.y = 0;
		wrenchGround.torque.z = 0;
	}

	return wrenchGround.force;

}

// Dummy torque calculation function
geometry_msgs::Vector3 PanosContactPoints::getTorque()
{
	return wrenchGround.torque;
}