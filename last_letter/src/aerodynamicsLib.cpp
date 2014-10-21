//////////////////////////
// Define Aerodynamics class
//////////////////////////

Aerodynamics::Aerodynamics(ModelPlane * parent)
{
	parentObj = parent;
	XmlRpc::XmlRpcValue list;
	int i;
	if(!ros::param::getCached("airframe/CGOffset", list)) {ROS_FATAL("Invalid parameters for -/airframe/CGOffset- in param server!"); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	}
	CGOffset.x = list[0];
	CGOffset.y = list[1];
	CGOffset.z = list[2];
}

Aerodynamics::~Aerodynamics()
{
	delete parentObj;
}

/////////////////////////////
// Define NoAerodynamics class
/////////////////////////////

// Class constructor
NoAerodynamics::NoAerodynamics(ModelPlane * parent) : Aerodynamics(parent)
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
geometry_msgs::Vector3 NoAerodynamics::getForce()
{
	return wrenchAero.force;
}

// Torque calculation function
geometry_msgs::Vector3 NoAerodynamics::getTorque()
{
	return wrenchAero.torque;
}

/////////////////////////////
// Define StdLinearAero class
/////////////////////////////

// Class constructor
StdLinearAero::StdLinearAero(ModelPlane * parent) : Aerodynamics(parent)
{
	// Read aerodynamic coefficients from parameter server
	if(!ros::param::getCached("airframe/c_lift_q", c_lift_q)) {ROS_FATAL("Invalid parameters for -c_lift_q- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_lift_deltae", c_lift_deltae)) {ROS_FATAL("Invalid parameters for -c_lift_deltae- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_drag_q", c_lift_q)) {ROS_FATAL("Invalid parameters for -c_drag_q- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_drag_deltae", c_drag_deltae)) {ROS_FATAL("Invalid parameters for -c_drag_deltae- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c", c)) {ROS_FATAL("Invalid parameters for -c- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/b", b)) {ROS_FATAL("Invalid parameters for -b- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/s", s)) {ROS_FATAL("Invalid parameters for -s- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_y_0", c_y_0)) {ROS_FATAL("Invalid parameters for -c_y_0- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_y_b", c_y_b)) {ROS_FATAL("Invalid parameters for -c_y_b- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_y_p", c_y_p)) {ROS_FATAL("Invalid parameters for -c_y_p- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_y_r", c_y_r)) {ROS_FATAL("Invalid parameters for -c_y_r- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_y_deltaa", c_y_deltaa)) {ROS_FATAL("Invalid parameters for -c_y_deltaa- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_y_deltar", c_y_deltar)) {ROS_FATAL("Invalid parameters for -c_y_deltar- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_l_0", c_l_0)) {ROS_FATAL("Invalid parameters for -c_l_0- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_l_b", c_l_b)) {ROS_FATAL("Invalid parameters for -c_l_b- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_l_p", c_l_p)) {ROS_FATAL("Invalid parameters for -c_l_p- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_l_r", c_l_r)) {ROS_FATAL("Invalid parameters for -c_l_r- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_l_deltaa", c_l_deltaa)) {ROS_FATAL("Invalid parameters for -c_l_deltaa- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_l_deltar", c_l_deltar)) {ROS_FATAL("Invalid parameters for -c_l_deltar- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_m_0", c_m_0)) {ROS_FATAL("Invalid parameters for -c_m_0- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_m_a", c_m_a)) {ROS_FATAL("Invalid parameters for -c_m_a- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_m_q", c_m_q)) {ROS_FATAL("Invalid parameters for -c_m_q- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_m_deltae", c_m_deltae)) {ROS_FATAL("Invalid parameters for -c_m_deltae- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_n_0", c_n_0)) {ROS_FATAL("Invalid parameters for -c_n_0- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_n_b", c_n_b)) {ROS_FATAL("Invalid parameters for -c_n_b- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_n_p", c_n_p)) {ROS_FATAL("Invalid parameters for -c_n_p- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_n_r", c_n_r)) {ROS_FATAL("Invalid parameters for -c_n_r- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_n_deltaa", c_n_deltaa)) {ROS_FATAL("Invalid parameters for -c_n_deltaa- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_n_deltar", c_n_deltar)) {ROS_FATAL("Invalid parameters for -c_n_deltar- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_drag_p", c_drag_p)) {ROS_FATAL("Invalid parameters for -c_drag_p- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_lift_0", c_lift_0)) {ROS_FATAL("Invalid parameters for -c_lift_0- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/c_lift_a", c_lift_a0)) {ROS_FATAL("Invalid parameters for -c_lift_a- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("/environment/oswald", oswald)) {ROS_FATAL("Invalid parameters for -oswald- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/mcoeff", M)) {ROS_FATAL("Invalid parameters for -mcoeff- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("airframe/alpha_stall", alpha0)) {ROS_FATAL("Invalid parameters for -alpha_stall- in param server!"); ros::shutdown();}
}

// Class destructor
StdLinearAero::~StdLinearAero()
{
}

// Force calculation function
geometry_msgs::Vector3 StdLinearAero::getForce()
{
	//split control input values
	double deltaa = parentObj->input[0];
	double deltae = parentObj->input[1];
	double deltat = parentObj->input[2];
	double deltar = parentObj->input[3];

	// Read airdata
	double airspeed = parentObj->airdata.airspeed;
	double alpha = parentObj->airdata.alpha;
	double beta = parentObj->airdata.beta;

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

	//read angular rates
	double p = parentObj->states.velocity.angular.x;
	double q = parentObj->states.velocity.angular.y;
	double r = parentObj->states.velocity.angular.z;

	//calculate aerodynamic force
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
	double ax, ay, az;
	if (airspeed==0)
	{
		ax = 0;
		ay = 0;
		az = 0;
	}
	else
	{
		ax = qbar*(c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(deltae) + c_lift_deltae*sin(alpha)*deltae);
		// split c_x_deltae to include "abs" term
		ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*deltaa + c_y_deltar*deltar);
		az = qbar*(c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(deltae) - c_lift_deltae*cos(alpha)*deltae);
		// split c_z_deltae to include "abs" term
	}
	
	wrenchAero.force.x = ax;
	wrenchAero.force.y = ay;
	wrenchAero.force.z = az;

	// Printouts
	// std::cout << rho << " ";
	// std::cout << airspeed << " ";
	// std::cout << pow(airspeed,2) << " ";
	// std::cout << s<< " ";
	// std::cout << alpha << " ";
	// std::cout << c_lift_a << " ";
	// std::cout << c_drag_a << " ";
	// std::cout << c_x_a << " ";
	// std::cout<< ax << std::endl;

	return wrenchAero.force;
}

// Torque calculation function
geometry_msgs::Vector3 StdLinearAero::getTorque()
{
	//split control input values
	double deltaa = parentObj->input[0];
	double deltae = parentObj->input[1];
	double deltat = parentObj->input[2];
	double deltar = parentObj->input[3];

	// Read airdata
	double airspeed = parentObj->airdata.airspeed;
	double alpha = parentObj->airdata.alpha;
	double beta = parentObj->airdata.beta;

	// Read air density
	rho = parentObj->environment.density;

	//read angular rates
	double p = parentObj->states.velocity.angular.x;
	double q = parentObj->states.velocity.angular.y;
	double r = parentObj->states.velocity.angular.z;

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
		la = qbar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*airspeed) + c_l_r*b*r/(2*airspeed) + c_l_deltaa*deltaa + c_l_deltar*deltar);
		ma = qbar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*airspeed) + c_m_deltae*deltae);
		na = qbar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*airspeed) + c_n_r*b*r/(2*airspeed) + c_n_deltaa*deltaa + c_n_deltar*deltar);
	}
	
	wrenchAero.torque.x = la;
	wrenchAero.torque.y = ma;
	wrenchAero.torque.z = na;

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
	wrenchAero.torque.x = wrenchAero.torque.x + CGOffset.y*wrenchAero.force.z - CGOffset.z*wrenchAero.force.y;
	wrenchAero.torque.y = wrenchAero.torque.y - CGOffset.x*wrenchAero.force.z + CGOffset.z*wrenchAero.force.x;
	wrenchAero.torque.z = wrenchAero.torque.z - CGOffset.y*wrenchAero.force.x + CGOffset.x*wrenchAero.force.y;

	return wrenchAero.torque;
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
HCUAVAero::HCUAVAero (ModelPlane * parent) : StdLinearAero(parent)
{
	char s[100];
	Factory factory;
	// Create CLift polynomial
	sprintf(s,"%s","airframe/cLiftPoly");
	liftCoeffPoly =  factory.buildPolynomial(s);
	// Create CDrag polynomial
	sprintf(s,"%s","airframe/cDragPoly");
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