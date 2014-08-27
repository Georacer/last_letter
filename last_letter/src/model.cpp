#include "model.hpp"

//////////////////////////
// Define ModelPlane class
//////////////////////////

	///////////////////
	//Class Constructor
	ModelPlane::ModelPlane (ros::NodeHandle n) :
	kinematics(this), dynamics(this), airdata(this)
	{
		initTime = -1;
		init();
		tprev = ros::Time::now();
		states.header.stamp = tprev;
		//Subscribe and advertize
		subInp = n.subscribe("ctrlPWM",1,&ModelPlane::getInput, this); //model control input
		subEnv = n.subscribe("environment",1,&ModelPlane::getEnvironment, this); //dynamic environment effects
		pubState = n.advertise<last_letter::SimStates>("states",1000); //model states
		pubForce = n.advertise<geometry_msgs::Vector3>("forceInput",1000); //forces & torques
		pubTorque = n.advertise<geometry_msgs::Vector3>("torqueInput",1000); //forces & torques
	}
	
	//Initialize states
	void ModelPlane::init()
	{
		XmlRpc::XmlRpcValue list;
		double temp[4];
		int i;
		
		//Initialize states
		states.header.frame_id = "bodyFrame";
		
		if(!ros::param::getCached("init/position", list)) {ROS_FATAL("Invalid parameters for -init/position- in param server!"); ros::shutdown();}
		for (i = 0; i < list.size(); ++i) {
			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			temp[i]=list[i];
		}
		states.pose.position.x=temp[0];
		states.pose.position.y=temp[1];
		states.pose.position.z=temp[2];
		
		if(!ros::param::getCached("init/orientation", list)) {ROS_FATAL("Invalid parameters for -init/orientation- in param server!"); ros::shutdown();}
		for (i = 0; i < list.size(); ++i) {
			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			temp[i]=list[i];
		}
		states.pose.orientation.x=temp[0];
		states.pose.orientation.y=temp[1];
		states.pose.orientation.z=temp[2];
		states.pose.orientation.w=temp[3];
		
		if(!ros::param::getCached("init/velLin", list)) {ROS_FATAL("Invalid parameters for -init/velLin- in param server!"); ros::shutdown();}
		for (i = 0; i < list.size(); ++i) {
			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			temp[i]=list[i];
		}
		states.velocity.linear.x=temp[0];
		states.velocity.linear.y=temp[1];
		states.velocity.linear.z=temp[2];
		
		if(!ros::param::getCached("init/velAng", list)) {ROS_FATAL("Invalid parameters for -init/velAng- in param server!"); ros::shutdown();}
		for (i = 0; i < list.size(); ++i) {
			ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			temp[i]=list[i];
		}
		states.velocity.angular.x=temp[0];
		states.velocity.angular.y=temp[1];
		states.velocity.angular.z=temp[2];
		
		states.rotorspeed.clear();
		states.rotorspeed.push_back((double) 0);
		
		states.geoid.latitude = 45.542;
		states.geoid.longitude = 0.0;
		states.geoid.altitude = 0.0;
		
		input[0] = 0;
		input[1] = 0;
		input[2] = 0;
		input[3] = 0;
		
		//Initialize environment
		environment.wind.x = 0;
		environment.wind.y = 0;
		environment.wind.z = 0;
		if(!ros::param::getCached("/environment/rho", environment.density)) {ROS_FATAL("Invalid parameters for -/environment/rho- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/environment/groundPres", environment.pressure)) {ROS_FATAL("Invalid parameters for -/environment/groundPress- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/environment/groundTemp", environment.temperature)) {ROS_FATAL("Invalid parameters for -/environment/groundTemp- in param server!"); ros::shutdown();}
		//if(!ros::param::getCached("/environment/g", environment.gravity)) {ROS_FATAL("Invalid parameters for -/environment/g- in param server!"); ros::shutdown();}
		environment.gravity = 9.81;
	}
	
	///////////////////////////////////////
	//Make one step of the plane simulation
	void ModelPlane::step(void)
	{
		if (initTime > 0) {
			dt = (ros::Time::now() - tprev).toSec();
			}
		else {
			if(!ros::param::getCached("simRate", dt)) {ROS_FATAL("Invalid parameters for -simRate- in param server!"); ros::shutdown();}
			dt = 1/dt;
			initTime = 1;
		}
		tprev = ros::Time::now();
		states.header.stamp = tprev;
		
		//make a simulation step
		dynamics.propulsion->updateRPS();
		kinematics.forceInput = dynamics.getForce();
		kinematics.torqueInput = dynamics.getTorque();
		kinematics.calcDerivatives();
		kinematics.integrator->propagation();
		airdata.calcAirData();
		//publish results
		pubState.publish(states);
		pubForce.publish(kinematics.forceInput);
		pubTorque.publish(kinematics.torqueInput);
	}
	
	/////////////////////////////////////////////////
	//convert uS PPM values to control surface inputs
	void ModelPlane::getInput(last_letter::SimPWM inputMsg)
	{
		if(!ros::param::getCached("airframe/deltaa_max", deltaa_max)) {ROS_FATAL("Invalid parameters for -airframe/deltaa_max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/deltae_max", deltae_max)) {ROS_FATAL("Invalid parameters for -airframe/deltae_max- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/deltar_max", deltar_max)) {ROS_FATAL("Invalid parameters for -airframe/deltar_max- in param server!"); ros::shutdown();}
		//Convert PPM to radians (0/1 for throttle)
		input[0] = deltaa_max * (double)(inputMsg.value[0]-1500)/500;
		input[1] = deltae_max * (double)(inputMsg.value[1]-1500)/500;
		input[2] = (double)(inputMsg.value[2]-1000)/1000;
		input[3] = deltar_max * (double)(inputMsg.value[3]-1500)/500;
//		std::cout << "aetr :" << input[0] << " " << input[1] << " " << input[2] << " " << input[3] << std::endl;
	}
	
	/////////////////////////////////////////////////
	//Store environmental values
	void ModelPlane::getEnvironment(last_letter::Environment envUpdate)
	{
		environment = envUpdate;
	}
	
	//////////////////
	//Class destructor
	ModelPlane::~ModelPlane ()
	{
	}
	

//////////////////////////
// Define Kinematics class
//////////////////////////
	
	///////////////////
	//Class Constructor
	Kinematics::Kinematics(ModelPlane * parent)
	{
		parentObj = parent;
		forceInput.x = 0;
		forceInput.y = 0;
		forceInput.z = 0;
		torqueInput.x = 0;
		torqueInput.y = 0;
		torqueInput.z = 0;
		if(!ros::param::getCached("airframe/m", mass)) {ROS_FATAL("Invalid parameters for -m- in param server!"); ros::shutdown();}
		double j_x, j_y, j_z, j_xz;
		if(!ros::param::getCached("airframe/j_x", j_x)) {ROS_FATAL("Invalid parameters for -j_x- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/j_y", j_y)) {ROS_FATAL("Invalid parameters for -j_y- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/j_z", j_z)) {ROS_FATAL("Invalid parameters for -j_z- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/j_xz", j_xz)) {ROS_FATAL("Invalid parameters for -j_xz- in param server!"); ros::shutdown();}
		double G = j_x*j_z-pow(j_xz,2);
//		J[9] = {j_x, 0, -j_xz, 0, j_y, 0, -j_xz, 0, j_z};
		J[0] = j_x; J[1] = 0; J[2] = -j_xz;
		J[3] = 0; J[4] = j_y; J[5] = 0;
		J[6] = -j_xz; J[7] = 0; J[8] = j_z;
//		Jinv[9] = {j_z/G, 0, j_xz/G, 0, 1/j_y, 0, j_xz/G, 0, j_x/G};
		Jinv[0] = j_z/G; Jinv[1] = 0; Jinv[2] = j_xz/G;
		Jinv[3] = 0; Jinv[4] = 1/j_y; Jinv[5] = 0;
		Jinv[6] = j_xz/G; Jinv[7] = 0; Jinv[8] = j_x/G;
		Factory factory;
		integrator = factory.buildIntegrator(parentObj);
	}
	
	//////////////////
	//Class Destructor
	Kinematics::~Kinematics()
	{
		delete integrator;
		delete parentObj;
	}
	
	///////////////////////////////
	//State derivatives calculation
	void Kinematics::calcDerivatives()
	{
		//variable declaration
		geometry_msgs::Vector3 tempVect;
		double Reb[9];
		
		//create transformation matrix
		quat2rotmtx (parentObj->states.pose.orientation, Reb);
		
		//create position derivatives
		posDot = Reb*parentObj->states.velocity.linear;
		
		//create speed derivatives
		geometry_msgs::Vector3 linearAcc = (1.0/mass)*forceInput;
		geometry_msgs::Vector3 corriolisAcc;
		vector3_cross(-parentObj->states.velocity.angular, parentObj->states.velocity.linear, &corriolisAcc);
		speedDot = linearAcc + corriolisAcc;
		
		//create angular derivatives
		quatDot.w = 1.0;
		quatDot.x = parentObj->states.velocity.angular.x*0.5*parentObj->dt;
		quatDot.y = parentObj->states.velocity.angular.y*0.5*parentObj->dt;
		quatDot.z = parentObj->states.velocity.angular.z*0.5*parentObj->dt;
		
		//create angular rate derivatives
		vector3_cross(parentObj->states.velocity.angular, J*parentObj->states.velocity.angular, &tempVect);
		tempVect = -tempVect+torqueInput;
		rateDot = Jinv*tempVect;
	}

//////////////////////////
// Define Integrator class
//////////////////////////

	Integrator::Integrator(ModelPlane * parent)
	{
		parentObj = parent;
	}
	
	Integrator::~Integrator()
	{
		delete parentObj;
	}

//////////////////////////
// Define ForwardEuler class
//////////////////////////

	//Class Constructor
	ForwardEuler::ForwardEuler(ModelPlane * parent) : Integrator(parent)
	{
	}
	//Class Destructor
	
	//Propagation of the states
	void ForwardEuler::propagation()
	{
		geometry_msgs::Vector3 tempVect;
		
		parentObj->states.pose.position.x = parentObj->states.pose.position.x + parentObj->kinematics.posDot.x * parentObj->dt;
		parentObj->states.pose.position.y = parentObj->states.pose.position.y + parentObj->kinematics.posDot.y * parentObj->dt;
		parentObj->states.pose.position.z = parentObj->states.pose.position.z + parentObj->kinematics.posDot.z * parentObj->dt;
		
//		tempVect = parentObj->dt*parentObj->kinematics.posDot;
//		tempVect = parentObj->states.pose.position + tempVect;
//		parentObj->states.pose.position = tempVect;
//		parentObj->states.pose.position = parentObj->states.pose.position + tempVect;
		
		geometry_msgs::Quaternion quat = parentObj->states.pose.orientation;
		quat_product(quat,parentObj->kinematics.quatDot,&(parentObj->states.pose.orientation));
		quat_normalize(&(parentObj->states.pose.orientation));
		
		tempVect = parentObj->dt*parentObj->kinematics.speedDot;
		parentObj->states.velocity.linear = parentObj->states.velocity.linear + tempVect;
		
		tempVect = parentObj->dt*parentObj->kinematics.rateDot;
		parentObj->states.velocity.angular = parentObj->states.velocity.angular + tempVect;
		
		parentObj->states.acceleration.linear = parentObj->kinematics.speedDot;
		
		parentObj->states.acceleration.angular = parentObj->kinematics.rateDot;
		
		//Update Geoid stuff -- To update!
		parentObj->states.geoid.altitude = -parentObj->states.pose.position.z;
		
//		std::cout << "u v w : " << parentObj->states.velocity.linear.x << " " << parentObj->states.velocity.linear.y << " " << parentObj->states.velocity.linear.z << std::endl;
		
//		std::cout << "Updated ModelPlane states" << std::endl;
	}

//////////////////////////
// Define Dynamics class
//////////////////////////

	///////////////////
	//Class Constructor
	Dynamics::Dynamics(ModelPlane * parent)
	{
		parentObj = parent;
		Factory factory;
		aerodynamics = factory.buildAerodynamics(parentObj);
		gravity = new Gravity(parentObj);
		propulsion = factory.buildPropulsion(parentObj);
		groundReaction = factory.buildGroundReaction(parentObj);
	}
	
	//Class Destructor
	Dynamics::~Dynamics()
	{
		delete aerodynamics;
		delete gravity;
		delete propulsion;
		delete groundReaction;
		delete parentObj;
	}
	
	geometry_msgs::Vector3 Dynamics::getForce()
	{
		geometry_msgs::Vector3 tempVect;
		tempVect = propulsion->getForce();
		tempVect = groundReaction->getForce() + tempVect;
		tempVect = gravity->getForce() + tempVect;
		tempVect = aerodynamics->getForce() + tempVect;
		
//		std::cout << "AeroForceY: " << aerodynamics->wrenchAero.force.z;
//		std::cout << " GroundForceY: " << groundReaction->wrenchGround.force.z;
//		std::cout << " GravForceY: " << gravity->wrenchGrav.force.z;
//		std::cout << " PropForceX: " << propulsion->wrenchProp.force.x << std::endl;
		return tempVect;
	}
	
	geometry_msgs::Vector3 Dynamics::getTorque()
	{
		geometry_msgs::Vector3 tempVect;
		tempVect = aerodynamics->getTorque();
		tempVect = gravity->getTorque() + tempVect;
		tempVect = propulsion->getTorque() + tempVect;
		tempVect = groundReaction->getTorque() + tempVect;
		
//		std::cout << "AeroTorqueY: " << aerodynamics->wrenchAero.torque.y;
//		std::cout << " GroundTorqueY: " << groundReaction->wrenchGround.torque.y;
//		std::cout << " GravTorqueY: " << gravity->wrenchGrav.torque.y;
//		std::cout << " PropTorqueY: " << propulsion->wrenchProp.torque.y << std::endl;
		return tempVect;
	}
	

//////////////////////////
// Define Airdata class
//////////////////////////

	//Class Constructor
	Airdata::Airdata(ModelPlane * parent)
	{
		parentObj = parent;
		airspeed = 0;
		alpha = 0;
		beta = 0;
	}
	
	//Class Destructor
	Airdata::~Airdata()
	{
		delete parentObj;
	}
	
	//Caclulate airspeed and aerodynamics angles
	void Airdata::calcAirData()
	{
		double u = parentObj->states.velocity.linear.x;
		double v = parentObj->states.velocity.linear.y;
		double w = parentObj->states.velocity.linear.z;
		
		airspeed = sqrt(pow(u,2)+pow(v,2)+pow(w,2));
		alpha = atan2(w,u);

		if (u==0) {
			if (v==0) {
				beta=0;
			}
			else {
				beta=asin(v/abs(v));
			}

		}
		else {
			beta = atan2(v,u);
		}
	}


//////////////////////////
// Define Aerodynamics class
//////////////////////////

	Aerodynamics::Aerodynamics(ModelPlane * parent)
	{
		parentObj = parent;
	}
	
	Aerodynamics::~Aerodynamics()
	{
		delete parentObj;
	}

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


//////////////////////////
// Define GroundReaction class
//////////////////////////

	GroundReaction::GroundReaction(ModelPlane * parent)
	{
		parentObj = parent;
	}
	
	GroundReaction::~GroundReaction()
	{
		delete parentObj;
	}

//////////////////////////
// Define StdLinearAero class
//////////////////////////

	StdLinearAero::StdLinearAero(ModelPlane * parent) : Aerodynamics(parent)
	{
		if(!ros::param::getCached("airframe/m", m)) {ROS_FATAL("Invalid parameters for -m- in param server!"); ros::shutdown();}
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
	
	StdLinearAero::~StdLinearAero()
	{
	}
	
	geometry_msgs::Vector3 StdLinearAero::getForce()
	{
		//split control input values
		double deltaa = parentObj->input[0];
		double deltae = parentObj->input[1];
		double deltat = parentObj->input[2];
		double deltar = parentObj->input[3];
		double airspeed = parentObj->airdata.airspeed;
//		std::cout << airspeed << std::endl;
		double alpha = parentObj->airdata.alpha;
		double beta = parentObj->airdata.beta;
		
		rho = parentObj->environment.density;

		//request lift and drag alpha-coefficients from the corresponding functions
		double c_lift_a = liftCoeff(alpha);
		double c_drag_a = dragCoeff(alpha);
		
		//convert coefficients to the body frame
		double c_x_a = -c_drag_a*cos(alpha)+c_lift_a*sin(alpha);
		double c_x_q = -c_drag_q*cos(alpha)+c_lift_q*sin(alpha);
//		double c_x_deltae = -c_drag_deltae*cos(alpha)+c_lift_deltae*sin(alpha);
		double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
		double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);
//		double c_z_deltae = -c_drag_deltae*sin(alpha)-c_lift_deltae*cos(alpha);
	
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
			ax = qbar*(c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*abs(deltae) + c_lift_deltae*sin(alpha)*deltae);
			//split c_x_deltae to include "abs" term
			ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*deltaa + c_y_deltar*deltar);
			az = qbar*(c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*abs(deltae) - c_lift_deltae*cos(alpha)*deltae);
			//split c_z_deltae to include "abs" term
		}
		
		double temp = c_lift_a + c_lift_deltae*deltae;
//		std::cout << "c_lift: " << temp << std::endl;
		
		wrenchAero.force.x = ax;
		wrenchAero.force.y = ay;
		wrenchAero.force.z = az;
//		std::cout << "body aerodynamic forces: " << ax << " " << ay << " " << az << std::endl;
		return wrenchAero.force;
	}
	
	geometry_msgs::Vector3 StdLinearAero::getTorque()
	{
		//split control input values
		double deltaa = parentObj->input[0];
		double deltae = parentObj->input[1];
		double deltat = parentObj->input[2];
		double deltar = parentObj->input[3];
		double airspeed = parentObj->airdata.airspeed;
		double alpha = parentObj->airdata.alpha;
		double beta = parentObj->airdata.beta;
		
		rho = parentObj->environment.density;

		//request lift and drag alpha-coefficients from the corresponding functions
//		double c_lift_a = liftCoeff(alpha);
//		double c_drag_a = dragCoeff(alpha);

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
//		std::cout << "sigma, alpha, c_lift_a :" << sigmoid << " " << alpha << " " << result << std::endl;
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

//////////////////////////
// Define PanosContactPoints class
//////////////////////////


	PanosContactPoints::PanosContactPoints(ModelPlane * parent) : GroundReaction(parent)
	{
		XmlRpc::XmlRpcValue list;
		int i, j, points;
		char paramMsg[25];
		double temp[4];
		//Read contact points location and type from airframe parameters
		if(!ros::param::getCached("airframe/contactPtsNo", contactPtsNo)) {ROS_FATAL("Invalid parameters for -/airframe/contactPtsNo- in param server!"); ros::shutdown();}
		contactPoints = (double*)malloc(sizeof(double) * (contactPtsNo*4)); //Allocate space for the table
		
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
			contactPoints[j + contactPtsNo*3] = temp[2];
		}
		
		spp = (double*)malloc(sizeof(double) * contactPtsNo); 
		memset(spp, 0, sizeof(spp)); // initialize sprint contraction
		
		kspring = 8000.0;
		mspring = 250.0;
		kfriction = 0.01;
		len=-0.02;

		contact = false;
		cpi_up = (double*)malloc(sizeof(double) * contactPtsNo*3); // upper spring end matrix
		cpi_down = (double*)malloc(sizeof(double) * contactPtsNo*3); // lower spring end matrix
		spd = (double*)malloc(sizeof(double) * contactPtsNo); // spring contraction speed
		pointCoords = (double*)malloc(sizeof(double) * contactPtsNo*3); // contact points coordinates in the body frame
	}
	
	PanosContactPoints::~PanosContactPoints()
	{
		free(contactPoints);
		free(cpi_up);
		free(cpi_down);
		free(spd);
		free(pointCoords);
	}
	
	geometry_msgs::Vector3 PanosContactPoints::getForce()
	{
		double Reb[9];
		int i, j;
		for (i=0; i<contactPtsNo*3; i++) {
			pointCoords[i] = contactPoints[i];
		}
		geometry_msgs::Quaternion quat = parentObj->states.pose.orientation;
		quat2rotmtx(quat, Reb);
		geometry_msgs::Wrench totalE;
		geometry_msgs::Vector3 dx,we,vpoint,Ve;

		Ve=Reb*parentObj->states.velocity.linear; // Rotate body velocity to earth velocity

		uavpos[0]=parentObj->states.pose.position.x;
		uavpos[1]=parentObj->states.pose.position.y;
		uavpos[2]=parentObj->states.pose.position.z;

		multi_mtx_mtx_3Xn(Reb,pointCoords,cpi_up,contactPtsNo); // Rotate contact points coordinates from body frame to earth frame

		for (i=0;i<3;i++) 
		{
			for (j=0;j<contactPtsNo;j++) {
				cpi_up[contactPtsNo*i+j] +=uavpos[i]; // Place upper spring end to contact point
				cpi_down[contactPtsNo*i+j]=cpi_up[contactPtsNo*i+j]; // Place lower spring end to contact point
			}
		}
		
		we = Reb*parentObj->states.velocity.angular; // Rotate body angular speeds to earth frame
		
		for (i=0;i<contactPtsNo;i++) // For each contact point
		{
			cpi_down[i+2*contactPtsNo]-=len; // Place lower spring end "len" below upper spring end
			dx.x = (cpi_up[i]-uavpos[0]); // Calculate force arm
			dx.y = (cpi_up[i+contactPtsNo]-uavpos[1]);
			dx.z = (cpi_up[i+2*contactPtsNo]-uavpos[2]);

			spd[i]=(len-(cpi_up[i+2*contactPtsNo]-cpi_down[i+2*contactPtsNo])-spp[i])/parentObj->dt; // Update spring contraction speed
			
			if (cpi_down[i+2*contactPtsNo]>0) // Handle ground contact
			{
				cpi_down[i+2*contactPtsNo]=0;
				contact=true;
				vector3_cross(we,dx, &vpoint); // Contact point inertial speed
				vpoint = Ve+vpoint;
				normVe = sqrt(vpoint.x*vpoint.x+vpoint.y*vpoint.y+vpoint.z*vpoint.z); // Absolute contact point velocity
				if (normVe<=0.001) // Take at static friction???
					normVe=0.001;

				totalE.force.z = kspring*(len-cpi_up[i+2*contactPtsNo])-mspring*vpoint.z*abs(vpoint.z)+10.0*spd[i]; // Spring force along body z-axis
				totalE.force.x = -kfriction*abs(totalE.force.z)*vpoint.x; // Spring friction along body x-axis
				totalE.force.y = -kfriction*abs(totalE.force.z)*vpoint.y; // Spring friction along body y-axis
				totalE.force.x = max(-1000.0,min(totalE.force.x,1000.0)); // Cap forces
				totalE.force.y = max(-1000.0,min(totalE.force.y,1000.0));
				totalE.force.z = max(-1000.0,min(totalE.force.z,1000.0));
			}
			spp[i]=len-(cpi_up[i+2*contactPtsNo]-cpi_down[i+2*contactPtsNo]); // Update spring contraction
		}

		if (contact) {
			wrenchGround.force= Reb/totalE.force;
			vector3_cross(dx,wrenchGround.force, &totalE.torque);
			wrenchGround.torque= Reb/totalE.torque;
		}
		return wrenchGround.force;
		
	}
	
	geometry_msgs::Vector3 PanosContactPoints::getTorque()
	{
		return wrenchGround.torque;
	}


/////////////////////////////////////////////////////
Propulsion::Propulsion(ModelPlane * parent)
{
	parentObj = parent;
}

Propulsion::~Propulsion()
{
	delete parentObj;
}

EngBeard::EngBeard(ModelPlane * parent):Propulsion(parent)
{
	std::cout << "reading parameters for new Beard engine" << std::endl;
	omega = 0;
	if(!ros::param::getCached("motor/s_prop", s_prop)) {ROS_FATAL("Invalid parameters for -s_prop- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/c_prop", c_prop)) {ROS_FATAL("Invalid parameters for -c_prop- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_motor", k_motor)) {ROS_FATAL("Invalid parameters for -k_motor- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_t_p", k_t_p)) {ROS_FATAL("Invalid parameters for -k_t_p- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_omega", k_omega)) {ROS_FATAL("Invalid parameters for -k_omega- in param server!"); ros::shutdown();}
}

EngBeard::~EngBeard()
{
}

void EngBeard::updateRPS()
{
	rho = parentObj->environment.density;
	deltat = parentObj->input[2];
//	std::cout << "throttle input: " << deltat << std::endl;
	airspeed = parentObj->airdata.airspeed;
	omega = 1 / (0.5 + parentObj->dt) * (0.5 * omega + parentObj->dt * deltat * k_motor);
}

geometry_msgs::Vector3 EngBeard::getForce()
{
	wrenchProp.force.x = 1.0/2.0*rho*s_prop*c_prop*(pow(omega,2)-pow(airspeed,2));
//		double tx = 1.0/2.0*rho*s_prop*c_prop*(pow(k_motor*deltat,2)-pow(airspeed,2));
	wrenchProp.force.y = 0;
	wrenchProp.force.z = 0;

	return wrenchProp.force;
}

geometry_msgs::Vector3 EngBeard::getTorque()
{
	wrenchProp.torque.x = -k_t_p*pow(k_omega*omega/k_motor,2);
//		double lm = -k_t_p*pow(k_omega*deltat,2);
	wrenchProp.torque.y = 0;
	wrenchProp.torque.z = 0;

	return wrenchProp.torque;
}
/////////////////////////////////////////////////



//////////////////////////
// Define Factory class
//////////////////////////

	//Build integrator model
	Integrator * Factory::buildIntegrator(ModelPlane * parent)
	{
		int i;
		if(!ros::param::getCached("/world/integratorType", i)) {ROS_FATAL("Invalid parameters for -integratorType- in param server!"); ros::shutdown();}
//		std::cout<< "building engine model" << std::endl;
		switch (i)
		{
		case 0:
//			std::cout << "selecting Forward Euler" << std::endl;
			return new ForwardEuler(parent);
		default:
			ROS_FATAL("Error while constructing integrator");
			ros::shutdown();
			break;
		}
	}
	
	Aerodynamics * Factory::buildAerodynamics(ModelPlane * parent)
	{
		int i;
		if(!ros::param::getCached("airframe/aerodynamicsType", i)) {ROS_FATAL("Invalid parameters for -aerodynamicsType- in param server!"); ros::shutdown();}
//		std::cout<< "building engine model" << std::endl;
		switch (i)
		{
		case 0:
//			std::cout << "selecting StdLinearAero aerodynamics" << std::endl;
			return new StdLinearAero(parent);
		default:
			ROS_FATAL("Error while constructing StdLinearAero aerodynamics");
			ros::shutdown();
			break;
		}
	}
	
	//Build engine model
	Propulsion * Factory::buildPropulsion(ModelPlane * parent)
	{
		int i;
		if(!ros::param::getCached("motor/motorType", i)) {ROS_FATAL("Invalid parameters for -motorType- in param server!"); ros::shutdown();}
//		std::cout<< "building engine model" << std::endl;
		switch (i)
		{
		case 0:
//			std::cout << "selecting Beard engine" << std::endl;
			return new EngBeard(parent);
		default:
			ROS_FATAL("Error while constructing Beard motor");
			ros::shutdown();
			break;
		}
	}
	
	GroundReaction * Factory::buildGroundReaction(ModelPlane * parent)
	{
		int i;
		if(!ros::param::getCached("airframe/groundReactionType", i)) {ROS_FATAL("Invalid parameters for -groundReactionType- in param server!"); ros::shutdown();}
//		std::cout<< "building engine model" << std::endl;
		switch (i)
		{
		case 0:
//			std::cout << "selecting Panos ground reactions" << std::endl;
			return new PanosContactPoints(parent);
		default:
			ROS_FATAL("Error while constructing Panos ground reactions");
			ros::shutdown();
			break;
		}
	}


		
		
///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

	ros::init(argc, argv, "simNode");
	ros::NodeHandle n;

	ros::Duration(3).sleep(); //wait for other nodes to get raised
	double simRate;
	ros::param::get("simRate",simRate); //frame rate in Hz
	ros::Rate spinner(simRate);
	
	ModelPlane uav(n);
	spinner.sleep();
	ROS_INFO("simNode up");
	
	while (ros::ok())
	{
		uav.step();
//		std::cout << "Just stepped" << std::endl;
		ros::spinOnce();
		spinner.sleep();

		if (isnan(uav.states.velocity.linear.x))
		{		
			ROS_FATAL("State NAN!");
			break;
		}
	}
	
	return 0;
	
}
