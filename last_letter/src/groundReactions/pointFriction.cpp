
//////////////////////////////////
// Define PointFriction class
//////////////////////////////////

// Class constructor
PointFriction::PointFriction(ModelPlane * parent) : GroundReaction(parent)
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
	len=0.2;

	// Set coefficient of friction for each material
	frict[0] = 0.7;
	frict[1] = 0.4;
	frict[2] = 1.0;
	frict[3] = 0.4; //To update composite to ground firction coefficients!

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
PointFriction::~PointFriction()
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
geometry_msgs::Vector3 PointFriction::getForce()
{
	double Reb[9];
	double mu;
	int i, j;
	contact = false;
	safe = true; // NaN protection flag
	geometry_msgs::Vector3 Euler;

	geometry_msgs::Quaternion quat = parentObj->states.pose.orientation; // Read vehicle orientation quaternion
	quat2rotmtx(quat, Reb); // Construct the rotation matrix
	Euler = quat2euler(quat);

	double normalF[contactPtsNo];
	for (i=0;i<contactPtsNo;i++){
		normalF[i]=0.0;
	}
	int contactList[contactPtsNo];
	for (i=0;i<contactPtsNo;i++){
		contactList[i]=0;
	}

	geometry_msgs::Wrench tempE, totalE; /** @todo This is a TODO list example */
	geometry_msgs::Vector3 dx,we,vpoint,Ve;
	geometry_msgs::Vector3 vpointList[contactPtsNo];

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
			contact=true; // update contact flag
			contactList[i]=1; // update list of point which contact the ground

			cpi_down[i+2*contactPtsNo]=0; // Force lower spring end to stay at ground level
			spp[i]=len+cpi_up[i+2*contactPtsNo]; // Calculate current spring contraction
			spd[i]=(spp[i]-sppprev[i])/parentObj->dt; // Calculate current spring contraction spreed
			vector3_cross(we,dx, &vpoint); // Contact point Earth-frame speed due to vehicle rotation
			vpoint = Ve+vpoint; // Add vehicle velocity to find total contact point velocity
			vpointList[i] = vpoint; // Store the contact point velocity

			// Calculate spring force. Acts only upon Earth z-axis
			// tempE.force.z = -(kspring*spp[i] + mspring*spd[i]);
			tempE.force.z = -(springIndex[i]*spp[i] + springIndex[i + contactPtsNo]*spd[i]);

			// Make spring force negative or zero, as the spring lower end isn't fixed on the ground
			if (tempE.force.z > 0)
				tempE.force.z = 0;
			/** @todo model full spring contraction (possibly with exponential force curve after full contraction?) */
			normalF[i] = tempE.force.z; // store the normal force on each contact point

		}

		// If there is no ground contact
		else {
			// Set spring contraction to zero
			spp[i] = 0;
			spd[i] = 0;
		}
		sppprev[i] = spp[i];
	}

	// Measure the sum of the normal force and count the touching points
	double totalN = 0;
	int totalPoints = 0;
	for (i=0;i<contactPtsNo;i++)
	{
		totalN += normalF[i];
		totalPoints += contactList[i];
	}

	// Distribute the friction forces
	for (i=0;i<contactPtsNo;i++)
	{
		if (contactList[i]) // If this point is contacting the ground
		{

			int tempIndex = materialIndex[i];
			// Get longitudinal and lateral friction coefficients for this surface
			mu = frict[tempIndex];
			double Fmax = mu*normalF[i];

			// Calculate force arm
			dx.x = (cpi_up[i]-uavpos[0]);
			dx.y = (cpi_up[i+contactPtsNo]-uavpos[1]);
			dx.z = (cpi_up[i+2*contactPtsNo]-uavpos[2]);

			// Calculate point friction in the Earth frame
			if (sqrt(vpointList[i].x*vpointList[i].x + vpointList[i].y*vpointList[i].y) > 0.01) // If the point is moving
			{
				double forceAngle = atan2(vpointList[i].y, vpointList[i].x);
				tempE.force.x = Fmax*cos(forceAngle);
				tempE.force.y = Fmax*sin(forceAngle);
			}
			else // If static friction is applied on the point
			{
				// Read the external forces and torques
				extForce = parentObj->dynamics.forceGrav;
				extForce = extForce + parentObj->dynamics.forceProp;
				extForce = extForce + parentObj->dynamics.forceAero;
				// Rotate forces to the inertial frame
				extForce = Reb*extForce;

				extTorque = parentObj->dynamics.torqueGrav;
				extTorque = extTorque + parentObj->dynamics.torqueProp;
				extTorque = extTorque + parentObj->dynamics.torqueAero;
				// Rotate torque to the inertial frame
				extTorque = Reb*extTorque;

				// Calculate the force arm in the Earth coordinates
				geometry_msgs::Vector3 dx_inertial;
				dx_inertial = Reb*dx;

				geometry_msgs::Vector3 reaction2Force, reaction2Torque;
				reaction2Force.x = 0; reaction2Force.y = 0; reaction2Force.z = 0;
				if (std::fabs(totalN) > 1e-6)
				{
					reaction2Force.x = -extForce.x*(normalF[i]/totalN); // Calculate the load share of the point
					reaction2Force.y = -extForce.y*(normalF[i]/totalN);
				}
				else
				{
					// No action required
				}

				geometry_msgs::Vector3 dx_inertial_planar;
				dx_inertial_planar = dx_inertial;
				dx_inertial_planar.z = 0;
				double armLength = vector3_norm(dx_inertial_planar); // Calculate arm length...
				dx_inertial_planar = vector3_normalize(dx_inertial_planar); // ... and the unit arm vector

				geometry_msgs::Vector3 extTorque_planar;
				extTorque_planar.x = 0;
				extTorque_planar.y = 0;
				extTorque_planar.z = -extTorque.z/totalPoints;
				double torqueNorm = vector3_norm(extTorque_planar); // Calculate torque norm...
				extTorque_planar = vector3_normalize(extTorque_planar); // ... and the unit torque vector

				vector3_cross(extTorque_planar, dx_inertial_planar, &reaction2Torque); // Calculate unit force vector
				reaction2Torque = (torqueNorm/armLength) * reaction2Torque; // Calculate the torque share of the point

				geometry_msgs::Vector3 reaction, unit_reaction;
				reaction = reaction2Force + reaction2Torque; // Sum the force components
				// reaction = reaction2Force; // use only the external force reaction
				// reaction = reaction2Torque; // use only the external torque reaction
				// reaction.x =0; reaction.y = 0; reaction.z = 0; // return zero lateral ground reaction
				double reactionNorm = vector3_norm(reaction); // Calculate the friction force magnitude
				reactionNorm = std::min(Fmax,reactionNorm); // and compare with the static friction limit
				unit_reaction = vector3_normalize(reaction);
				reaction = reactionNorm * unit_reaction;

				tempE.force.x = reaction.x;
				tempE.force.y = reaction.y;
			}

			tempE.force.z = normalF[i];

			// Add current contact point force contribution to the total
			totalE.force = totalE.force + tempE.force;
			// Calculate current contact point torque
			vector3_cross(dx,tempE.force, &tempE.torque);
			// Add current contact point torque contribution to the total
			totalE.torque = totalE.torque + tempE.torque;
		}
	}

	// Chech for rogue NaN results
	if (isnan(totalE.force.x) or isnan(totalE.force.y) or isnan(totalE.force.z)) {
		safe = false; /** @todo remove the safe safety, it is not used*/
		ROS_FATAL("State NAN in PointFriction force calculation!");
		ros::shutdown();
	}

	if (isnan(totalE.torque.x) or isnan(totalE.torque.y) or isnan(totalE.torque.z)) {
		safe = false;
		ROS_FATAL("State NAN in PointFriction torque calculation!");
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
geometry_msgs::Vector3 PointFriction::getTorque()
{

	return wrenchGround.torque;
}