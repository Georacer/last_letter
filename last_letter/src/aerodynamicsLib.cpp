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
}

// Class destructor
HCUAVAero::~HCUAVAero()
{
}

//////////////////////////
//C_lift_alpha calculation
double HCUAVAero::liftCoeff (double alpha)
{
	int i;
	static int breaksNo = 28;
	static double breaks[] = {
	  -1.570796326794897,
	  -1.308996938995747,
	  -0.942477796076938,
	  -0.698131700797732,
	  -0.366519142918809,
	  -0.261799387799149,
	  -0.139626340159546,
	  -0.069813170079773,
	  -0.052359877559830,
	  -0.034906585039887,
	  -0.017453292519943,
	                   0,
	   0.017453292519943,
	   0.034906585039887,
	   0.052359877559830,
	   0.069813170079773,
	   0.139626340159546,
	   0.209439510239320,
	   0.261799387799149,
	   0.296705972839036,
	   0.331612557878923,
	   0.366519142918809,
	   0.401425727958696,
	   0.436332312998582,
	   0.471238898038469,
	   0.959931088596881,
	   1.308996938995747,
	   1.570796326794897
	};
	static double coeffs[] = {
	  -0000.794688686371,   0002.400009384401,  -0002.418777984005,                   0,
	  -0000.794688686371,   0001.775862349652,  -0001.325537320502,  -0000.483000000000,
	   0011.420136444807,   0000.902056501004,  -0000.344028798553,  -0000.769400000000,
	  -0037.549419781824,   0009.273453744538,   0002.142317397419,  -0000.633000000000,
	   0286.985967215252,  -0028.082123677623,  -0004.094873749392,  -0000.272100000000,
	  -0194.082915430201,   0062.077176951057,  -0000.534920095318,  -0000.679300000000,
	   0059.587400985233,  -0009.057926867604,   0005.942603270943,  -0000.172000000000,
	   0020.081997112670,   0003.422029211177,   0005.549143389303,   0000.219000000000,
	  -0304.454699090793,   0004.473520121153,   0005.686946721406,   0000.317000000000,
	   1009.645850436120,  -0011.467690645756,   0005.564875417306,   0000.416000000000,
	  -2229.401112138328,   0041.397242461870,   0006.087244640143,   0000.515000000000,
	   2453.321082499050,  -0075.333926801442,   0005.494937761207,   0000.622000000000,
	  -1376.881906982022,   0053.121664693158,   0005.107260653102,   0000.708000000000,
	   0420.933262027101,  -0018.971703370766,   0005.703289917606,   0000.806000000000,
	  -0118.760192311919,   0003.068310689833,   0005.425723353087,   0000.902000000000,
	   0010.771302806784,  -0003.149958438601,   0005.424298331044,   0000.997000000000,
	  -0122.318590009356,  -0000.894022054109,   0005.141975233107,   0001.364000000000,
	   0199.470675179972,  -0026.512367638832,   0003.228648288201,   0001.677000000000,
	  -0985.787691914520,   0004.820412748767,   0002.092860186124,   0001.802000000000,
	   0561.771016843568,  -0098.411032948496,  -0001.174068756814,   0001.839000000000,
	   0337.476689462810,  -0039.582509631315,  -0005.990952085831,   0001.702000000000,
	   1662.050252779152,  -0004.242033352176,  -0007.520717222319,   0001.459000000000,
	  -2518.517666236952,   0169.807462115425,  -0001.741393503529,   0001.262000000000,
	   0935.405196795478,  -0093.931091157445,   0000.907191491834,   0001.301000000000,
	  -0003.075518658603,   0004.024311988634,  -0002.231147140885,   0001.258000000000,
	  -0001.240229986117,  -0000.484633862493,  -0000.501334083549,   0000.769800000000,
	  -0001.240229986117,  -0001.783399666876,  -0001.293027136211,   0000.483000000000
	};

	for (i=0;i<breaksNo;i++) {
		if (alpha<=breaks[i])
			break;
	}

	double delta = alpha-breaks[i];
	double CLift = coeffs[4*i]*pow(delta,3) + coeffs[4*i+1]*pow(delta,2) + coeffs[4*i+2]*delta + coeffs[4*i+3];
	return CLift;
}

//////////////////////////
//C_drag_alpha calculation
double HCUAVAero::dragCoeff (double alpha)
{
	int i;
	static int breaksNo = 28;
	static double breaks[] = {
	  -1.570796326794897,
	  -1.308996938995747,
	  -1.047197551196598,
	  -0.785398163397448,
	  -0.523598775598299,
	  -0.261799387799149,
	  -0.139626340159546,
	  -0.069813170079773,
	  -0.052359877559830,
	  -0.034906585039887,
	  -0.017453292519943,
	                   0,
	   0.017453292519943,
	   0.034906585039887,
	   0.052359877559830,
	   0.069813170079773,
	   0.139626340159546,
	   0.209439510239320,
	   0.261799387799149,
	   0.296705972839036,
	   0.331612557878923,
	   0.366519142918809,
	   0.401425727958696,
	   0.436332312998582,
	   0.471238898038469,
	   0.872664625997165,
	   1.221730476396031,
	   1.570796326794897
	};
	static double coeffs[] = {
	   000.4464253410478,  -001.3110430986748,   000.0523252575824,   002.0000000000000,
	   000.4464253410478,  -000.9604214557218,  -000.5423427721661,   001.9318516525781,
	  -001.4732927261931,  -000.6097998127688,  -000.9534257389662,   001.7320508075689,
	   006.6538761013844,  -001.7669212140677,  -001.5756498487613,   001.4142135623731,
	  -023.5690485343350,   003.4590208554338,  -001.1326591985565,   001.0000000000000,
	   141.5973253572642,  -015.0520665764582,  -004.1677114710483,   000.5176380902050,
	  -237.4923587299377,   036.8460637530820,  -001.5050724157312,   000.0420000000000,
	   340.7600639436899,  -012.8942195448969,   000.1670817576990,   000.0357000000000,
	  -075.7588405915324,   004.9479356804745,   000.0283929409668,   000.0365000000000,
	  -037.7247015775627,   000.9812120630272,   000.1318760909281,   000.0381000000000,
	   132.6121724945746,  -000.9940486925551,   000.1316520494779,   000.0405000000000,
	  -154.1602805347813,   005.9495084222039,   000.2181411377103,   000.0432000000000,
	   070.2288622528277,  -002.1223049911863,   000.2849384387252,   000.0480000000000,
	   004.9084956935661,   001.5548696375379,   000.2750348235118,   000.0527000000000,
	  -014.6264655013311,   001.8118778710560,   000.3337956526201,   000.0580000000000,
	  -002.5976488889844,   001.0460379282733,   000.3836756930631,   000.0643000000000,
	   014.4679206994997,   000.5019876171907,   000.4917482637564,   000.0953000000000,
	  -050.0032094940294,   003.5321418426752,   000.7733836298619,   000.1370000000000,
	   110.8495086227584,  -004.3223439374425,   000.7320087449323,   000.1800000000000,
	   304.7001057524865,   007.2857894606674,   000.8354525080999,   000.2050000000000,
	  -412.7065561624121,   039.1939099200022,   002.4579000871596,   000.2560000000000,
	  -205.6242088218000,  -004.0246195776037,   003.6855399112890,   000.3720000000000,
	   365.2827531829215,  -025.5575363720964,   002.6529278689676,   000.4870000000000,
	  -126.9611110233629,   012.6947840906542,   002.2039331126084,   000.5640000000000,
	  -001.1798578325812,  -000.6005523654322,   002.6261014408170,   000.6510000000000,
	   001.0420560093838,  -002.0214282334273,   001.5735709702262,   001.5320888862380,
	   001.0420560093838,  -000.9301897321909,   000.5432619350051,   001.8793852415718
	};

	for (i=0;i<breaksNo;i++) {
		if (alpha<=breaks[i])
			break;
	}

	double delta = alpha-breaks[i];
	double CDrag = coeffs[4*i]*pow(delta,3) + coeffs[4*i+1]*pow(delta,2) + coeffs[4*i+2]*delta + coeffs[4*i+3];
	return CDrag;
}