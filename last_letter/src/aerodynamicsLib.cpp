//////////////////////////
// Define Aerodynamics class
//////////////////////////

Aerodynamics::Aerodynamics(ModelPlane * parent, int ID)
{
	parentObj = parent;
	id = ID;
	XmlRpc::XmlRpcValue list;
	int i;
	char paramMsg[50];
	sprintf(paramMsg, "airfoil%i/CGOffset", id);
	if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	}
	CGOffset.x = list[0];
	CGOffset.y = list[1];
	CGOffset.z = list[2];

	sprintf(paramMsg, "airfoil%i/mountOrientation", id);
	if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	}
	// !!! Order mixed because tf::Quaternion::setEuler seems to work with PRY, instead of YPR
	mountOrientation.y = list[0];
	mountOrientation.z = list[1];
	mountOrientation.x = list[2];

	sprintf(paramMsg, "airfoil%i/chanAileron", id);
	if(!ros::param::getCached(paramMsg, chanAileron)) {ROS_INFO("No AILERON%i channel selected", id); chanAileron=-1;}
	sprintf(paramMsg, "airfoil%i/chanElevator", id);
	if(!ros::param::getCached(paramMsg, chanElevator)) {ROS_INFO("No ELEVATOR%i channel selected", id); chanElevator=-1;}
	sprintf(paramMsg, "airfoil%i/chanRudder", id);
	if(!ros::param::getCached(paramMsg, chanRudder)) {ROS_INFO("No RUDDER%i channel selected", id); chanRudder=-1;}
	sprintf(paramMsg, "airfoil%i/chanGimbal", id);
	if(!ros::param::getCached(paramMsg, chanGimbal)) {ROS_INFO("No AIRFOIL GIMBAL%i channel set, selecting default", id); chanGimbal=-1;}
	sprintf(paramMsg, "airfoil%i/gimbalAngle_max", id);
	if(!ros::param::getCached(paramMsg, gimbalAngle_max)) {ROS_INFO("No AIRFOIL GIMBALANGLE_MAX%i value set, selecting default", id); gimbalAngle_max=0.0;}

	inputAileron = 0.0;
	inputElevator = 0.0;
	inputRudder = 0.0;
	inputGimbal = 0.0;
}

Aerodynamics::~Aerodynamics()
{
	delete parentObj;
}

void Aerodynamics::getInput()
{
	char paramMsg[50];
	sprintf(paramMsg, "airfoil%i/gimbalAngle_max", id);
	ros::param::getCached(paramMsg, gimbalAngle_max);
	sprintf(paramMsg, "airfoil%i/deltaa_max", id);
	ros::param::getCached(paramMsg, deltaa_max);
	sprintf(paramMsg, "airfoil%i/deltae_max", id);
	ros::param::getCached(paramMsg, deltae_max);
	sprintf(paramMsg, "airfoil%i/deltar_max", id);
	ros::param::getCached(paramMsg, deltar_max);
	//Convert PPM to radians
	if (chanAileron>-1) {inputAileron = deltaa_max * (double)(parentObj->input.value[chanAileron]-1500)/500;}
	if (chanElevator>-1) {inputElevator = deltae_max * (double)(parentObj->input.value[chanElevator]-1500)/500;}
	if (chanRudder>-1) {inputRudder = deltar_max * (double)(parentObj->input.value[chanRudder]-1500)/500;}
	if (chanGimbal>-1) {inputGimbal = gimbalAngle_max * (double)(parentObj->input.value[chanGimbal]-1500)/500; }
}

// One step in the physics engine
void Aerodynamics::stepDynamics()
{
	rotateFrame();
	getForce();
	getTorque();
	rotateForce();
	rotateTorque();
}

// Convert the relative wind from body axes to airfoil axes
void Aerodynamics::rotateFrame()
{

	tf::Quaternion tempQuat;
	// Construct transformation from body axes to mount frame
	tempQuat.setEuler(mountOrientation.z, mountOrientation.y, mountOrientation.x);

	body_to_mount.setOrigin(tf::Vector3(CGOffset.x, CGOffset.y, CGOffset.z));
	body_to_mount.setRotation(tempQuat);
	body_to_mount_rot.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	body_to_mount_rot.setRotation(tempQuat);

	// Construct transformation to apply gimbal movement. Gimbal rotation MUST be aligned with (be applied on) the resulting z-axis!
	// !!! Order mixed because tf::Quaternion::setEuler seems to work with PRY, instead of YPR
	tempQuat.setEuler(0.0, 0.0, inputGimbal);

	mount_to_gimbal.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	mount_to_gimbal.setRotation(tempQuat);
	mount_to_gimbal_rot.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	mount_to_gimbal_rot.setRotation(tempQuat);

	body_to_gimbal = body_to_mount * mount_to_gimbal;
	body_to_gimbal_rot = body_to_mount_rot * mount_to_gimbal_rot;

	// Publish the airfoil transformation to the transform tree
	char foil_frame[50];
	sprintf(foil_frame, "airfoil_%i", id);
	broadcaster.sendTransform(tf::StampedTransform(body_to_gimbal, ros::Time::now(), "base_link", foil_frame));

	// Transform the relative wind from body axes to airfoil axes
	tf::Vector3 bodyWind(parentObj->airdata.u_r, parentObj->airdata.v_r, parentObj->airdata.w_r);
	tf::Vector3 tempVect;
	tempVect = mount_to_gimbal_rot * (body_to_mount_rot * bodyWind);

	relativeWind.x = tempVect.getX();
	relativeWind.y = tempVect.getY();
	relativeWind.z = tempVect.getZ();

	airspeed = sqrt(pow(relativeWind.x,2)+pow(relativeWind.y,2)+pow(relativeWind.z,2));
	alpha = atan2(relativeWind.z,relativeWind.x);

	if (relativeWind.x==0) {
		if (relativeWind.y==0) {
			beta=0;
		}
		else {
			beta=asin(relativeWind.y/abs(relativeWind.y));
		}

	}
	else {
		beta = atan2(relativeWind.y,relativeWind.x);
		// beta = asin(v_r/airspeed);
	}

	// ROS_DEBUG_STREAM("aerodynamicsLib.cpp/rotateFrame: airspeed: " << airspeed << " alpha: " << alpha << " beta: " << beta);

	if (!std::isfinite(airspeed)) {ROS_FATAL("aerodynamicsLib.cpp/rotateWind: NaN value in airspeed"); ros::shutdown();}
	if (std::fabs(airspeed)>1e+160) {ROS_FATAL("aerodynamicsLib.cpp/rotateWind: normalWind over 1e+160"); ros::shutdown();}

	// Rotate angular rates from the body frame to the airfoil frame
	tf::Vector3 bodyRates(parentObj->states.velocity.angular.x, parentObj->states.velocity.angular.y, parentObj->states.velocity.angular.z);
	tempVect = mount_to_gimbal_rot * (body_to_mount_rot * bodyRates);
	relativeRates.x = tempVect.getX();
	p = relativeRates.x;
	relativeRates.y = tempVect.getY();
	q = relativeRates.y;
	relativeRates.z = tempVect.getZ();
	r = relativeRates.z;

	// ROS_DEBUG_STREAM("aerodynamicsLib.cpp/rotateFrame: p:" << p << "\t q: " << q << "\t r: " << r);
}

 // Convert the resulting force to the body axes
void Aerodynamics::rotateForce()
{


	tf::Vector3 tempVect(wrenchAero.force.x, wrenchAero.force.y, wrenchAero.force.z);
	tempVect = body_to_gimbal_rot * tempVect; // I'm not sure why this works and not inverted

	wrenchAero.force.x = tempVect.getX();
	wrenchAero.force.y = tempVect.getY();
	wrenchAero.force.z = tempVect.getZ();


}

// Convert the resulting torque to the body axes
void Aerodynamics::rotateTorque()
{

	tf::Vector3 tempVect(wrenchAero.torque.x, wrenchAero.torque.y, wrenchAero.torque.z);
	tempVect = body_to_gimbal_rot * tempVect;

	wrenchAero.torque.x = tempVect.getX();
	wrenchAero.torque.y = tempVect.getY();
	wrenchAero.torque.z = tempVect.getZ();

	// Convert the torque from the motor frame to the body frame
	double ratio = parentObj->kinematics.J[0] / (parentObj->kinematics.J[0] + parentObj->kinematics.mass * CGOffset.x*CGOffset.x);
	wrenchAero.torque.x =  ratio * wrenchAero.torque.x;
	ratio = parentObj->kinematics.J[4] / (parentObj->kinematics.J[4] + parentObj->kinematics.mass * CGOffset.y*CGOffset.y);
	wrenchAero.torque.y =  ratio * wrenchAero.torque.y;
	ratio = parentObj->kinematics.J[8] / (parentObj->kinematics.J[8] + parentObj->kinematics.mass * CGOffset.z*CGOffset.z);
	wrenchAero.torque.z =  ratio * wrenchAero.torque.z;

	// Add torque to to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	// Will potentially add the following code in the future, to support shift of CoG mid-flight
	// XmlRpc::XmlRpcValue list;
	// int i;
	// if(!ros::param::getCached("motor/CGOffset", list)) {ROS_FATAL("Invalid parameters for -/motor/CGOffset- in param server!"); ros::shutdown();}
	// for (i = 0; i < list.size(); ++i) {
	// 	ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	// 	CGOffset[i]=list[i];
	// }
	wrenchAero.torque.x = wrenchAero.torque.x + CGOffset.y*wrenchAero.force.z - CGOffset.z*wrenchAero.force.y;
	wrenchAero.torque.y = wrenchAero.torque.y - CGOffset.x*wrenchAero.force.z + CGOffset.z*wrenchAero.force.x;
	wrenchAero.torque.z = wrenchAero.torque.z - CGOffset.y*wrenchAero.force.x + CGOffset.x*wrenchAero.force.y;

}

/////////////////////////////
// Define NoAerodynamics class
/////////////////////////////

// Class constructor
NoAerodynamics::NoAerodynamics(ModelPlane * parent, int id) : Aerodynamics(parent, id)
{
	wrenchAero.force.x = 0.0;
	wrenchAero.force.y = 0.0;
	wrenchAero.force.z = 0.0;
	wrenchAero.torque.x = 0.0;
	wrenchAero.torque.y = 0.0;
	wrenchAero.torque.z = 0.0;
}

// Class destructor
NoAerodynamics::~NoAerodynamics()
{
}

// Force calculation function
void NoAerodynamics::getForce()
{
}

// Torque calculation function
void NoAerodynamics::getTorque()
{
}

/////////////////////////////
// Define StdLinearAero class
/////////////////////////////

// Class constructor
StdLinearAero::StdLinearAero(ModelPlane * parent, int id) : Aerodynamics(parent, id)
{
	// Read aerodynamic coefficients from parameter server
	char paramMsg[50];
	sprintf(paramMsg, "airfoil%i/c_lift_q", id);
	if(!ros::param::getCached(paramMsg, c_lift_q)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_lift_deltae", id);
	if(!ros::param::getCached(paramMsg, c_lift_deltae)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_drag_q", id);
	if(!ros::param::getCached(paramMsg, c_drag_q)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_drag_deltae", id);
	if(!ros::param::getCached(paramMsg, c_drag_deltae)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c", id);
	if(!ros::param::getCached(paramMsg, c)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/b", id);
	if(!ros::param::getCached(paramMsg, b)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/s", id);
	if(!ros::param::getCached(paramMsg, s)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_y_0", id);
	if(!ros::param::getCached(paramMsg, c_y_0)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_y_b", id);
	if(!ros::param::getCached(paramMsg, c_y_b)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_y_p", id);
	if(!ros::param::getCached(paramMsg, c_y_p)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_y_r", id);
	if(!ros::param::getCached(paramMsg, c_y_r)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_y_deltaa", id);
	if(!ros::param::getCached(paramMsg, c_y_deltaa)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_y_deltar", id);
	if(!ros::param::getCached(paramMsg, c_y_deltar)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_l_0", id);
	if(!ros::param::getCached(paramMsg, c_l_0)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_l_b", id);
	if(!ros::param::getCached(paramMsg, c_l_b)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_l_p", id);
	if(!ros::param::getCached(paramMsg, c_l_p)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_l_r", id);
	if(!ros::param::getCached(paramMsg, c_l_r)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_l_deltaa", id);
	if(!ros::param::getCached(paramMsg, c_l_deltaa)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_l_deltar", id);
	if(!ros::param::getCached(paramMsg, c_l_deltar)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_m_0", id);
	if(!ros::param::getCached(paramMsg, c_m_0)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_m_a", id);
	if(!ros::param::getCached(paramMsg, c_m_a)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_m_q", id);
	if(!ros::param::getCached(paramMsg, c_m_q)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_m_deltae", id);
	if(!ros::param::getCached(paramMsg, c_m_deltae)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_n_0", id);
	if(!ros::param::getCached(paramMsg, c_n_0)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_n_b", id);
	if(!ros::param::getCached(paramMsg, c_n_b)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_n_p", id);
	if(!ros::param::getCached(paramMsg, c_n_p)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_n_r", id);
	if(!ros::param::getCached(paramMsg, c_n_r)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_n_deltaa", id);
	if(!ros::param::getCached(paramMsg, c_n_deltaa)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_n_deltar", id);
	if(!ros::param::getCached(paramMsg, c_n_deltar)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_drag_p", id);
	if(!ros::param::getCached(paramMsg, c_drag_p)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_lift_0", id);
	if(!ros::param::getCached(paramMsg, c_lift_0)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/c_lift_a", id);
	if(!ros::param::getCached(paramMsg, c_lift_a0)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/oswald", id);
	if(!ros::param::getCached(paramMsg, oswald)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/mcoeff", id);
	if(!ros::param::getCached(paramMsg, M)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/alpha_stall", id);
	if(!ros::param::getCached(paramMsg, alpha0)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "airfoil%i/deltaa_max", id);
	if(!ros::param::getCached(paramMsg, deltaa_max)) {ROS_INFO("No -%s- value selected", paramMsg); deltaa_max=0;}
	sprintf(paramMsg, "airfoil%i/deltae_max", id);
	if(!ros::param::getCached(paramMsg, deltae_max)) {ROS_INFO("No -%s- value selected", paramMsg); deltae_max=0;}
	sprintf(paramMsg, "airfoil%i/deltar_max", id);
	if(!ros::param::getCached(paramMsg, deltar_max)) {ROS_INFO("No -%s- value selected", paramMsg); deltar_max=0;}

}

// Class destructor
StdLinearAero::~StdLinearAero()
{
}


// Force calculation function
void StdLinearAero::getForce()
{

	// Read air density
	rho = parentObj->environment.density;

	//request lift and drag alpha-coefficients from the corresponding functions
	double c_lift_a = liftCoeff(alpha);
	double c_drag_a = dragCoeff(alpha);

	//convert coefficients to the body frame
	double c_x_a = -c_drag_a*cos(alpha)+c_lift_a*sin(alpha);
	double c_x_q = -c_drag_q*cos(alpha)+c_lift_q*sin(alpha);
	double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
	double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);

	//read orientation
	geometry_msgs::Quaternion quat = parentObj->states.pose.orientation;

	//calculate aerodynamic force
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure


	// ROS_DEBUG_STREAM("aerodynamicsLib.cpp/getForce: qbar: " << qbar << "\t c_lift_a: " << c_lift_a << "\t c_drag_a: " << c_drag_a);
	double ax, ay, az;
	if (airspeed==0)
	{
		ax = 0;
		ay = 0;
		az = 0;
	}
	else
	{
		ax = qbar*(c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(inputElevator) + c_lift_deltae*sin(alpha)*inputElevator);
		// split c_x_deltae to include "abs" term
		ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder);
		az = qbar*(c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(inputElevator) - c_lift_deltae*cos(alpha)*inputElevator);
		// split c_z_deltae to include "abs" term
	}

	wrenchAero.force.x = ax;
	wrenchAero.force.y = ay;
	wrenchAero.force.z = az;


	// ROS_DEBUG_STREAM("aerodynamicsLib.cpp/getForce: forceAero.x:" << ax << "\tforceAero.y: " << ay << "\tforceAero.z: " << az);
}

// Torque calculation function
void StdLinearAero::getTorque()
{
	// Read air density
	rho = parentObj->environment.density;

	//calculate aerodynamic torque
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
	double la, na, ma;
	if (airspeed==0)
	{
		la = 0;
		ma = 0;
		na = 0;
	}
	else
	{
		la = qbar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*airspeed) + c_l_r*b*r/(2*airspeed) + c_l_deltaa*inputAileron + c_l_deltar*inputRudder);
		ma = qbar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*airspeed) + c_m_deltae*inputElevator);
		na = qbar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*airspeed) + c_n_r*b*r/(2*airspeed) + c_n_deltaa*inputAileron + c_n_deltar*inputRudder);
	}

	wrenchAero.torque.x = la;
	wrenchAero.torque.y = ma;
	wrenchAero.torque.z = na;


	// ROS_DEBUG_STREAM("aerodynamicsLib.cpp/getTorque: torqueAero.x:" << la << "\ttorqueAero.y: " << ma << "\ttorqueAero.z: " << na);

	// Add torque to to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	// Will potentially add the following code in the future, to support shift of CoG mid-flight
	// XmlRpc::XmlRpcValue list;
	// int i;
	// if(!ros::param::getCached("airframe/CGOffset", list)) {ROS_FATAL("Invalid parameters for -/airframe/CGOffset- in param server!"); ros::shutdown();}
	// for (i = 0; i < list.size(); ++i) {
	// 	ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	// 	CGOffset[i]=list[i];
	// }

	// Removing torque calculation, because the CG effect is now being taken into account in the 'rotateTorque' function
	// wrenchAero.torque.x = wrenchAero.torque.x + CGOffset.y*wrenchAero.force.z - CGOffset.z*wrenchAero.force.y;
	// wrenchAero.torque.y = wrenchAero.torque.y - CGOffset.x*wrenchAero.force.z + CGOffset.z*wrenchAero.force.x;
	// wrenchAero.torque.z = wrenchAero.torque.z - CGOffset.y*wrenchAero.force.x + CGOffset.x*wrenchAero.force.y;
}

//////////////////////////
//C_lift_alpha calculation
double StdLinearAero::liftCoeff (double alpha)
{
	double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
	double linear = (1.0-sigmoid) * (c_lift_0 + c_lift_a0*alpha); //Lift at small AoA
	double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall

	double result  = linear+flatPlate;
	return result;
}

//////////////////////////
//C_drag_alpha calculation
double StdLinearAero::dragCoeff (double alpha)
{
	AR = pow(b,2)/s;
	double c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a0*alpha,2)/(M_PI*oswald*AR);

	return c_drag_a;
}

/////////////////////////
// Define HCUAVAero class
/////////////////////////

// Class constructor
HCUAVAero::HCUAVAero (ModelPlane * parent, int ID) : StdLinearAero(parent, ID)
{
	int id = ID;
	char s[100];
	Factory factory;
	// Create CLift polynomial
	sprintf(s,"airfoil%i/cLiftPoly",id);
	liftCoeffPoly =  factory.buildPolynomial(s);
	// Create CDrag polynomial
	sprintf(s,"airfoil%i/cDragPoly",id);
	dragCoeffPoly =  factory.buildPolynomial(s);
}

// Class destructor
HCUAVAero::~HCUAVAero()
{
}

//////////////////////////
//C_lift_alpha calculation
double HCUAVAero::liftCoeff (double alpha)
{
	return liftCoeffPoly->evaluate(alpha);
}

//////////////////////////
//C_drag_alpha calculation
double HCUAVAero::dragCoeff (double alpha)
{
	return dragCoeffPoly->evaluate(alpha);
}