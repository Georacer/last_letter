////////////////////////////////////////
// Define GroundReaction interface class
////////////////////////////////////////

// Class constructor
GroundReaction::GroundReaction(ModelPlane * parent)
{
	parentObj = parent;
}

// Class destructor
GroundReaction::~GroundReaction()
{
	delete parentObj;
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
	char paramMsg[25];
	double temp[4];
	// Read contact points number from parameter server
	if(!ros::param::getCached("airframe/contactPtsNo", contactPtsNo)) {ROS_FATAL("Invalid parameters for -/airframe/contactPtsNo- in param server!"); ros::shutdown();}
	// Create an appropriately sized matrix to contain contact point information
	contactPoints = (double*)malloc(sizeof(double) * (contactPtsNo*4));
	
	// Set spring characteristics
	kspring = 30000.0;
	mspring = 100.0;
	len=0.1;

	// Set coefficient of friction for each material
	frictForw[0] = 0.7; frictSide[0] = 0.7;
	frictForw[1] = 0.4; frictSide[1] = 0.4;
	frictForw[2] = 0.01; frictSide[2] = 1.0;

	// Read contact points location and material from parameter server
	for (j = 0; j<contactPtsNo; j++) { //Distribute the data
		sprintf(paramMsg, "airframe/contactPoint%i", j+1);
		if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -/airframe/contactPoint- in param server!"); ros::shutdown();}
		for (i = 0; i < list.size(); ++i) {
			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			temp[i]=list[i];
		}
		contactPoints[j] = temp[0];
		contactPoints[j + contactPtsNo] = temp[1];
		contactPoints[j + contactPtsNo*2] = temp[2];
		contactPoints[j + contactPtsNo*3] = temp[3];
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
	pointCoords = (double*)malloc(sizeof(double) * contactPtsNo*3); // contact points coordinates in the body frame
}

// Class destructor
PanosContactPoints::~PanosContactPoints()
{
	free(contactPoints);
	free(cpi_up);
	free(cpi_down);
	free(spd);
	free(spp);
	free(sppprev);
	free(pointCoords);
}

// Wrench calculation function
geometry_msgs::Vector3 PanosContactPoints::getForce()
{
	double Reb[9];
	double kFrictionLong, kFrictionLat, kFrictionX, kFrictionY;
	int i, j;
	contact = false;
	safe = true; // NaN protection flag

	// Copy contact point coordinates to a separate matrix
	for (i=0; i<contactPtsNo*3; i++) {
		pointCoords[i] = contactPoints[i];
	}

	//Read vehicle orientation quaternion
	geometry_msgs::Quaternion quat = parentObj->states.pose.orientation;
	quat2rotmtx(quat, Reb);

	geometry_msgs::Wrench tempE, totalE;
	geometry_msgs::Vector3 dx,we,vpoint,Ve;

	totalE.force.x = 0;
	totalE.force.y = 0;
	totalE.force.z = 0;
	totalE.torque.x = 0;
	totalE.torque.y = 0;
	totalE.torque.z = 0;

	// Rotate body velocity to earth velocity
	Ve=Reb*parentObj->states.velocity.linear;

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

		// if the lower spring end touches the ground
		if (cpi_down[i+2*contactPtsNo]>0)
		{
			cpi_down[i+2*contactPtsNo]=0; // Force lower spring end to stay at ground level
			contact=true;
			spp[i]=len+cpi_up[i+2*contactPtsNo]; // Calculate current spring contraction
			spd[i]=(spp[i]-sppprev[i])/parentObj->dt; // Calculate current spring contraction spreed
			vector3_cross(we,dx, &vpoint); // Contact point Earth-frame speed due to vehicle rotation
			vpoint = Ve+vpoint; // Add vehicle velocity to find total contact point velocity
			// Unused calculations
			// normVe = sqrt(vpoint.x*vpoint.x+vpoint.y*vpoint.y+vpoint.z*vpoint.z); // Absolute contact point velocity
			// if (normVe<=0.001) // Take at static friction???
			// 	normVe=0.001;

			// Calculate spring force. Acts only upon Earth z-axis
			tempE.force.z = -(kspring*spp[i] + mspring*spd[i]*abs(spd[i])); // -mspring*vpoint.z*abs(vpoint.z));
			// Make spring force negative or zero, as the spring lower end isn't fixed on the ground
			if (tempE.force.z > 0)
				tempE.force.z = 0;

			//Select Contact Point material
			int materialIndex = int(contactPoints[i + contactPtsNo*3]);

			// To implement steering
			// if (i==2) {
			// 	double tempVx = vpoint.x*cos(parentObj->input[3]) + vpoint.y*sin(parentObj->input[3]);
			// 	double tempVy = vpoint.x*sin(parentObj->input[3]) + vpoint.y*cos(parentObj->input[3]);
			// 	vpoint.x = tempVx;
			// 	vpoint.y = tempVy;
			// }

			// Select contact point friction coefficient
			kFrictionLong = frictForw[materialIndex];
			kFrictionLat = frictSide[materialIndex];
			// Convert frictions coefficients to the Earth frame
			kFrictionX = abs(Reb[0] *kFrictionLong + Reb[1]*kFrictionLat);
			kFrictionY = abs(Reb[3]*kFrictionLong + Reb[4]*kFrictionLat);
			// Calculate friction forces
			tempE.force.x = -kFrictionX*abs(tempE.force.z)*vpoint.x; // Point friction along earth x-axis
			tempE.force.y = -kFrictionY*abs(tempE.force.z)*vpoint.y; // Point friction along earth y-axis

		// Add current contact point force contribution to the total
		totalE.force = totalE.force + tempE.force;
		// Calculate current contact point torque
		vector3_cross(dx,tempE.force, &tempE.torque);
		// Add current contact point torque contribution to the total
		totalE.torque = totalE.torque + tempE.torque;
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
		safe = false;
		ROS_FATAL("State NAN in PanosGroundReactions force calculation!");
	}

	if (isnan(totalE.torque.x) or isnan(totalE.torque.y) or isnan(totalE.torque.z)) {
		safe = false;
		ROS_FATAL("State NAN in PanosGroundReactions torque calculation!");
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