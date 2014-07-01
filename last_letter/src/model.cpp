#include "model.hpp"

static double _spp[contactN]={0.0};

//////////////////////////
// Define ModelPlane class
//////////////////////////

	///////////////////
	//Class Constructor
	ModelPlane::ModelPlane (ros::NodeHandle n)
	{
		XmlRpc::XmlRpcValue list;
		double temp[4];
		int i;
		initTime = -1;
		
		//Initialize states
		states.header.frame_id = "bodyFrame";
		tprev = ros::Time::now();
		states.header.stamp = tprev;
		
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
		dynamics.header.frame_id = "bodyFrame";
		
		//Initialize environment
		environment.wind.x = 0;
		environment.wind.y = 0;
		environment.wind.z = 0;
		if(!ros::param::getCached("/environment/rho", environment.density)) {ROS_FATAL("Invalid parameters for -/environment/rho- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/environment/groundPres", environment.pressure)) {ROS_FATAL("Invalid parameters for -/environment/groundPress- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/environment/groundTemp", environment.temperature)) {ROS_FATAL("Invalid parameters for -/environment/groundTemp- in param server!"); ros::shutdown();}
		//if(!ros::param::getCached("/environment/g", environment.gravity)) {ROS_FATAL("Invalid parameters for -/environment/g- in param server!"); ros::shutdown();}
		environment.gravity = 9.81;
		
		//Subscribe and advertize
		subInp = n.subscribe("input",1,&ModelPlane::getInput, this);
		subEnv = n.subscribe("environment",1,&ModelPlane::getEnvironment, this);
		pubState = n.advertise<last_letter::SimStates>("states",1000);
		pubWrench = n.advertise<geometry_msgs::WrenchStamped>("wrenchStamped",1000);
		
		//Define contact points
		contactpoints[0]=0.162; //x1
		contactpoints[1]=0.162; //x2
		contactpoints[2]=-0.8639; //x3 
		contactpoints[3]=-0.0832; //x4
		contactpoints[4]=-0.0832; //x5
		contactpoints[5]=0.3785; //x6
		contactpoints[6]=-0.9328; //x7

		contactpoints[7]=-0.2324; //y1
		contactpoints[8]=0.2324; //y2
		contactpoints[9]=0.0; //y3
		contactpoints[10]=0.9671; //y4
		contactpoints[11]=-0.9671; //y5
		contactpoints[12]=0.0; //y6
		contactpoints[13]=0.0; //y7

		contactpoints[14]=0.2214; //z1
		contactpoints[15]=0.2214; //z2
		contactpoints[16]=0.0522; //z3
		contactpoints[17]=-0.1683; //z4
		contactpoints[18]=-0.1683; //z5
		contactpoints[19]=0.017; //z6
		contactpoints[20]=-0.2196; //z7
		
	}
	
	////////////////////////////////////
	//Force calculation in the body axes
	geometry_msgs::Vector3 ModelPlane::getForce(last_letter::SimStates states, double inputs[4])
	{
	
		//request air data from getAirData
		geometry_msgs::Vector3 airData = getAirData(states.velocity.linear);
		double airspeed = airData.x;
		double alpha = airData.y;
		double beta = airData.z;
	
		//split control input values
		double deltaa = inputs[0];
		double deltae = inputs[1];
		double deltat = inputs[2];
		double deltar = inputs[3];
	
		//request lift and drag alpha-coefficients from the corresponding functions
		double c_lift_a = liftCoeff(alpha);
		double c_drag_a = dragCoeff(alpha);

		//read from parameter server
		double rho,g,m;
		double c_lift_q,c_lift_deltae,c_drag_q,c_drag_deltae;
		double c,b,s;
		double c_y_0,c_y_b,c_y_p,c_y_r,c_y_deltaa,c_y_deltar;
		double s_prop,c_prop,k_motor;
	
		//if(!ros::param::getCached("/environment/rho", rho)) {ROS_FATAL("Invalid parameters for -rho- in param server!"); ros::shutdown();}	
		//if(!ros::param::getCached("/environment/g", g)) {ROS_FATAL("Invalid parameters for -g- in param server!"); ros::shutdown();}	
		rho = environment.density;
		g = environment.gravity;
		
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
		if(!ros::param::getCached("motor/s_prop", s_prop)) {ROS_FATAL("Invalid parameters for -s_prop- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("motor/c_prop", c_prop)) {ROS_FATAL("Invalid parameters for -c_prop- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("motor/k_motor", k_motor)) {ROS_FATAL("Invalid parameters for -k_motor- in param server!"); ros::shutdown();}
		
		//convert coefficients to the body frame
		double c_x_a = -c_drag_a*cos(alpha)-c_lift_a*sin(alpha);
		double c_x_q = -c_drag_q*cos(alpha)-c_lift_q*sin(alpha);
		double c_x_deltae = -c_drag_deltae*cos(alpha)-c_lift_deltae*sin(alpha);
		double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
		double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);
		double c_z_deltae = -c_drag_deltae*sin(alpha)-c_lift_deltae*cos(alpha);
	
		//read orientation
		geometry_msgs::Quaternion quat = states.pose.orientation;
	
		//read angular rates
		double p = states.velocity.angular.x;
		double q = states.velocity.angular.y;
		double r = states.velocity.angular.z;
	
		//calculate gravity force
		double Reb[9];
		quat2rotmtx(quat,Reb);
		geometry_msgs::Vector3 gravVect;
		gravVect.z = m*g;
		geometry_msgs::Vector3 gravForce = Reb/gravVect;
		//quat_vector3_rotate(quat, gravVect, &gravForce);
		double gx = gravForce.x;
		double gy = gravForce.y;
		double gz = gravForce.z;
	
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
			ax = qbar*(c_x_a + c_x_q*c*q/(2*airspeed) + c_x_deltae*deltae);
			ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*deltaa + c_y_deltar*deltar);
			az = qbar*(c_z_a + c_z_q*c*q/(2*airspeed) + c_z_deltae*deltae);
		}
	
		//Calculate Thrust force
		double tx = 1.0/2.0*rho*s_prop*c_prop*(pow(k_motor*deltat,2)-pow(airspeed,2));
		double ty = 0;
		double tz = 0;
		states.rotorspeed[0] = k_motor*deltat;

		//Sum forces
		double fx = gx+ax+tx;
		double fy = gy+ay+ty;
		double fz = gz+az+tz;
		
		geometry_msgs::Vector3 result;
		result.x = fx;
		result.y = fy;
		result.z = fz;
		
		return result;
	}

	/////////////////////////////////////
	//Torque calculation in the body axes
	geometry_msgs::Vector3 ModelPlane::getTorque(last_letter::SimStates states, geometry_msgs::Vector3 forces, double inputs[4])
	{

		//request air data from getAirData
		geometry_msgs::Vector3 airData = getAirData(states.velocity.linear);
		double airspeed = airData.x;
		double alpha = airData.y;
		double beta = airData.z;
	
		//split control input values
		double deltaa = inputs[0];
		double deltae = inputs[1];
		double deltat = inputs[2];
		double deltar = inputs[3];
	
		//request lift and drag alpha-coefficients from the corresponding functions
		double c_lift_a = liftCoeff(alpha);
		double c_drag_a = dragCoeff(alpha);

		//read from parameter server
		double rho, g, mass;
		double c,b,s;
		double c_l_0, c_l_b, c_l_p, c_l_r, c_l_deltaa, c_l_deltar;
		double c_m_0, c_m_a, c_m_q, c_m_deltae;
		double c_n_0, c_n_b, c_n_p, c_n_r, c_n_deltaa, c_n_deltar;
		double k_t_p, k_omega;
		double col_x, col_y, col_z;
	
		//if(!ros::param::getCached("/environment/rho", rho)) {ROS_FATAL("Invalid parameters for -rho- in param server!"); ros::shutdown();}
		rho = environment.density;
		g = environment.gravity;

		if(!ros::param::getCached("airframe/col_x", col_x)) {ROS_FATAL("Invalid parameters for -col_x- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/col_y", col_y)) {ROS_FATAL("Invalid parameters for -col_y- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/col_z", col_z)) {ROS_FATAL("Invalid parameters for -col_z- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/c", c)) {ROS_FATAL("Invalid parameters for -c- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/b", b)) {ROS_FATAL("Invalid parameters for -b- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/s", s)) {ROS_FATAL("Invalid parameters for -s- in param server!"); ros::shutdown();}
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
		if(!ros::param::getCached("motor/k_t_p", k_t_p)) {ROS_FATAL("Invalid parameters for -k_t_p- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("motor/k_omega", k_omega)) {ROS_FATAL("Invalid parameters for -k_omega- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/m", mass)) {ROS_FATAL("Invalid parameters for -m- in param server!"); ros::shutdown();}			
			
		//read angular rates
		double p = states.velocity.angular.x;
		double q = states.velocity.angular.y;
		double r = states.velocity.angular.z;
	
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
	
		//calculate thrust torque
		double lm = -k_t_p*pow(k_omega*deltat,2);
		double mm = 0;
		double nm = 0;
		
		//calculate cog torque, r x F, where r is the distance of CoL from CoG
		geometry_msgs::Quaternion quat = states.pose.orientation;
		double Reb[9];
		quat2rotmtx(quat,Reb);
		geometry_msgs::Vector3 gravVect;
		gravVect.z = mass*g;
		geometry_msgs::Vector3 gravForce = Reb/gravVect;
		forces.x = forces.x - gravForce.x;
		forces.y = forces.y - gravForce.y;
		forces.z = forces.z - gravForce.z;
		double lg = col_y*forces.z - col_z*forces.y;
		double mg = -col_x*forces.z + col_z*forces.x;
		double ng = -col_y*forces.x + col_x*forces.y;
		
		//Sum torques
		double l = la + lm + lg;
		double m = ma + mm + mg;
		double n = na + nm + ng;
	
		geometry_msgs::Vector3 result;
		result.x = l;
		result.y = m;
		result.z = n;
		
		return result;
	}
	
	//////////////////
	//Ground dynamics
	geometry_msgs::Wrench ModelPlane::groundDynamics(geometry_msgs::Quaternion quat)
	{
	
		double Reb[9];
		quat2rotmtx(quat, Reb);
		
		double kspring = 8000.0;
		double mspring = 250.0;
		double kfriction = 0.01;
		double len=-0.02;

		int i,j;	
		double cpi_up[contactN*3];
		double cpi_down[contactN*3];
		double helipos[3], normVe;

		bool contact = false;
		double spd[contactN];

		geometry_msgs::Wrench temp, totalE, totalB;


		geometry_msgs::Vector3 dx[contactN],we,vpoint,Ve;

		Ve=Reb*states.velocity.linear;

		temp.force = 0.0*temp.force;
		temp.torque = 0.0*temp.torque;
		totalE.force = 0.0*temp.force;
		totalE.torque = 0.0*temp.torque;
		totalB.force = 0.0*temp.force;
		totalB.torque = 0.0*temp.torque;

		helipos[0]=states.pose.position.x;
		helipos[1]=states.pose.position.y;
		helipos[2]=states.pose.position.z;

		multi_mtx_mtx_3Xn(Reb,contactpoints,cpi_up,contactN);

		for (i=0;i<3;i++) {
			for (j=0;j<contactN;j++) {
				cpi_up[contactN*i+j] +=helipos[i];
				cpi_down[contactN*i+j]=cpi_up[contactN*i+j];
			}
		}

		we = Reb*states.velocity.angular;
		for (i=0;i<contactN;i++) {
			cpi_down[i+2*contactN]-=len;
			dx[i].x = (cpi_up[i]-helipos[0]);
			dx[i].y = (cpi_up[i+contactN]-helipos[1]);
			dx[i].z = (cpi_up[i+2*contactN]-helipos[2]);

			spd[i]=(len-(cpi_up[i+2*contactN]-cpi_down[i+2*contactN])-_spp[i])/dt;
			_spp[i]=len-(cpi_up[i+2*contactN]-cpi_down[i+2*contactN]);

			if (cpi_down[i+2*contactN]>0) {
				cpi_down[i+2*contactN]=0;
				contact=true;
				vector3_cross(we,dx[i], &vpoint);
				vpoint = Ve+vpoint;			
				normVe = sqrt(vpoint.x*vpoint.x+vpoint.y*vpoint.y+vpoint.z*vpoint.z);
				if (normVe<=0.001)
					normVe=0.001;

				temp.force.z = kspring*(len-cpi_up[i+2*contactN])-mspring*vpoint.z*abs(vpoint.z)+10.0*spd[i];
				temp.force.x = -kfriction*abs(temp.force.z)*vpoint.x;
				temp.force.y = -kfriction*abs(temp.force.z)*vpoint.y;
				temp.force.x = max(-1000.0,min(temp.force.x,1000.0));
				temp.force.y = max(-1000.0,min(temp.force.y,1000.0));
				temp.force.z = max(-1000.0,min(temp.force.z,1000.0));

				totalE.force = totalE.force + temp.force;

				vector3_cross(dx[i],temp.force, &temp.torque);
				totalE.torque = totalE.torque + temp.torque;

			}
		}

		if (contact) {
			totalB.force= Reb/totalE.force;
			totalB.torque= Reb/totalE.torque;
		}

		return totalB;			
	}	
	
	/////////////////////////////////////////
	//Aerodynamc angles/ airspeed calculation
	geometry_msgs::Vector3 ModelPlane::getAirData (geometry_msgs::Vector3 speeds)
	{
		double u = speeds.x-environment.wind.x;
		double v = speeds.y-environment.wind.y;
		double w = speeds.z-environment.wind.z;

		double airspeed = sqrt(pow(u,2)+pow(v,2)+pow(w,2));
		double alpha = atan2(w,u);
		double beta = 0;
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
	
		geometry_msgs::Vector3 result;
		result.x = airspeed;
		result.y = alpha;
		result.z = beta;
		
		return result;
	}
	
	//////////////////////////
	//C_lift_alpha calculation
	double ModelPlane::liftCoeff (double alpha)
	{
		double M, alpha0, c_lift_0, c_lift_a;
		if(!ros::param::getCached("airframe/mcoeff", M)) {ROS_FATAL("Invalid parameters for -mcoeff- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/alpha_stall", alpha0)) {ROS_FATAL("Invalid parameters for -alpha_stall- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/c_lift_0", c_lift_0)) {ROS_FATAL("Invalid parameters for -c_lift_0- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/c_lift_a", c_lift_a)) {ROS_FATAL("Invalid parameters for -c_lift_a- in param server!"); ros::shutdown();}
		
		double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
		double linear = (1-sigmoid) * (c_lift_0 + c_lift_a*alpha); //Lift at small AoA
		double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall
	
		c_lift_a = linear+flatPlate;	
		return c_lift_a;
	}
	
	//////////////////////////
	//C_drag_alpha calculation
	double ModelPlane::dragCoeff (double alpha)
	{
		double c_drag_p, c_lift_0, c_lift_a, oswald, b, S, AR;
		if(!ros::param::getCached("airframe/c_drag_p", c_drag_p)) {ROS_FATAL("Invalid parameters for -c_drag_p- in param server!"); ros::shutdown();}	
		if(!ros::param::getCached("airframe/c_lift_0", c_lift_0)) {ROS_FATAL("Invalid parameters for -c_lift_0- in param server!"); ros::shutdown();}	
		if(!ros::param::getCached("airframe/c_lift_a", c_lift_a)) {ROS_FATAL("Invalid parameters for -c_lift_a- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/environment/oswald", oswald)) {ROS_FATAL("Invalid parameters for -oswald- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/b", b)) {ROS_FATAL("Invalid parameters for -b- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/s", S)) {ROS_FATAL("Invalid parameters for -s- in param server!"); ros::shutdown();}
		
		AR = pow(b,2)/S;
		double c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a*alpha,2)/(M_PI*oswald*AR);

		return c_drag_a;
	}
	
	///////////////////////////////////////////////////
	//Differential Equation calculation and propagation	
	void ModelPlane::diffEq(void)
	{
		//variables declaration
		geometry_msgs::Vector3 tempVect;
		geometry_msgs::Quaternion quat = states.pose.orientation;		
		double Reb[9];
		
		//create transformation matrix
		quat2rotmtx (states.pose.orientation, Reb);			
		
		//create position derivatives
		geometry_msgs::Vector3 posDot = Reb*states.velocity.linear;
		
		//create speed derivatives
		double mass;
		if(!ros::param::getCached("airframe/m", mass)) {ROS_FATAL("Invalid parameters for -m- in param server!"); ros::shutdown();}
		geometry_msgs::Vector3 linearAcc = (1.0/mass)*dynamics.wrench.force;
		geometry_msgs::Vector3 corriolisAcc;
		vector3_cross(-states.velocity.angular, states.velocity.linear, &corriolisAcc);
		geometry_msgs::Vector3 speedDot = linearAcc + corriolisAcc;
		states.acceleration.linear = linearAcc;
		
		//create angular derivatives
		geometry_msgs::Quaternion wquat;
		wquat.w = 1.0;
		wquat.x = states.velocity.angular.x*0.5*dt;
		wquat.y = states.velocity.angular.y*0.5*dt;
		wquat.z = states.velocity.angular.z*0.5*dt;
		
		//create angular rate derivatives
		double j_x, j_y, j_z, j_xz;
		if(!ros::param::getCached("airframe/j_x", j_x)) {ROS_FATAL("Invalid parameters for -j_x- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/j_y", j_y)) {ROS_FATAL("Invalid parameters for -j_y- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/j_z", j_z)) {ROS_FATAL("Invalid parameters for -j_z- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("airframe/j_xz", j_xz)) {ROS_FATAL("Invalid parameters for -j_xz- in param server!"); ros::shutdown();}

		double G = j_x*j_z-pow(j_xz,2);	
		double J[9] = {j_x, 0, -j_xz, 0, j_y, 0, -j_xz, 0, j_z};
		double Jinv[9] = {j_z/G, 0, j_xz/G, 0, 1/j_y, 0, j_xz/G, 0, j_x/G};

		vector3_cross(states.velocity.angular, J*states.velocity.angular, &tempVect);
		tempVect = -tempVect+dynamics.wrench.torque;
		geometry_msgs::Vector3 rateDot = Jinv*tempVect;
		states.acceleration.angular = rateDot;
		
		//integrate quantities using forward Euler
		states.pose.position.x = states.pose.position.x + posDot.x*dt;
		states.pose.position.y = states.pose.position.y + posDot.y*dt;
		states.pose.position.z = states.pose.position.z + posDot.z*dt;
		
		tempVect = dt*speedDot;
		states.velocity.linear = states.velocity.linear + tempVect;
		
		quat_product(quat,wquat,&states.pose.orientation);
		quat_normalize(&states.pose.orientation);		
		
		tempVect = dt*rateDot;
		states.velocity.angular = states.velocity.angular + tempVect;
		
		//Update Geoid stuff -- To update!
		states.geoid.altitude = -states.pose.position.z;

	}	
	
	///////////////////////////////////////
	//Make one step of the plane simulation
	void ModelPlane::step(void)
	{
		if (initTime > 0) {
			durTemp = (ros::Time::now() - tprev);
			dt = durTemp.toSec();
			}
		else {
			if(!ros::param::getCached("simRate", dt)) {ROS_FATAL("Invalid parameters for -simRate- in param server!"); ros::shutdown();}
			dt = 1/dt;
		}
		tprev = ros::Time::now();
		states.header.stamp = tprev;
		
		//calclulate body force/torques
		dynamics.wrench.force = getForce(states, input);
		dynamics.wrench.torque = getTorque(states, dynamics.wrench.force, input);
		//add ground dynamics
		groundDynamicsVect = groundDynamics(states.pose.orientation);
		dynamics.wrench.force = dynamics.wrench.force + groundDynamicsVect.force;
		dynamics.wrench.torque = dynamics.wrench.torque + groundDynamicsVect.torque;

		//make a simulation step
		diffEq();
		//publish results
		pubState.publish(states);
		pubWrench.publish(dynamics);
	}
	
	/////////////////////////////////////////////////
	//convert uS PPM values to control surface inputs
	void ModelPlane::getInput(last_letter::SimPWM inputMsg)
	{
		//Convert PPM to -1/1 ranges (0/1 for throttle)
		input[0] = (double)(inputMsg.value[0]-1500)/500;
		input[1] = (double)(inputMsg.value[1]-1500)/500;
		input[2] = (double)(inputMsg.value[2]-1000)/1000;
		input[3] = (double)(inputMsg.value[3]-1500)/500;
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
	//ros::Duration(3).sleep(); 
//	uav.tprev = ros::Time::now();
	spinner.sleep();
	ROS_INFO("simNode up");
	
	while (ros::ok())
	{
		uav.step();
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
