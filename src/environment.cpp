#include "environment.hpp"

ros::Publisher pub;

///////////////////
//Environment Class
//Based on Aerocalc: http://www.kilohotel.com/python/aerocalc/
//Valid up to 11km
//////////////////

	/////////////
	//Constructor
	environmentModel::environmentModel()
	{
		int i;
		ros::param::get("simRate",simRate); //frame rate in Hz
		dt = 1.0/simRate;
		
		//initialize atmosphere stuff
		if(!ros::param::getCached("/world/groundTemp", T0)) {ROS_FATAL("Invalid parameters for -/world/groundTemp- in param server!"); ros::shutdown();}
		T0 += 274.15;
		if(!ros::param::getCached("/world/groundPres", P0)) {ROS_FATAL("Invalid parameters for -/world/groundPres- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/world/g", g)) {ROS_FATAL("Invalid parameters for -/world/g- in param server!"); ros::shutdown();}
		Rd = 287.05307;
		L0 = -6.5;
		if(!ros::param::getCached("/world/rho", Rho0)) {ROS_FATAL("Invalid parameters for -/world/rho- in param server!"); ros::shutdown();}
		//initialize gravity calculations
		R_earth = 6378137.0;
		f_earth = 1.0/298.257223563;
		e_earth = sqrt(2.0*f_earth - f_earth*f_earth);
		RP_earth = R_earth*(1.0-e_earth*e_earth);
		omega_earth = 7.292115e-5;
		grav_const = 3.986004418e14;
		grav_earth =  9.7803267714;
		Re = 6378137.0;
		grav_temp = (2.0/R_earth)*(1.0+f_earth+(omega_earth*omega_earth)*(R_earth*R_earth)*RP_earth/grav_const);
		//Initialize bias wind engine
		if(!ros::param::getCached("/world/windRef", windRef)) {ROS_FATAL("Invalid parameters for -/world/windRef- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/world/windRefAlt", windRefAlt)) {ROS_FATAL("Invalid parameters for -/world/windRefAlt- in param server!"); ros::shutdown();}			
		if(!ros::param::getCached("/world/windDir", windDir)) {ROS_FATAL("Invalid parameters for -/world/windDir- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/world/surfSmooth", surfSmooth)) {ROS_FATAL("Invalid parameters for -/world/surfSmooth- in param server!"); ros::shutdown();}
		windDir = windDir*M_PI/180; //convert to rad
		kwind = windRef/pow(windRefAlt,surfSmooth);
		//Initialize turbulence engine
		if(!ros::param::getCached("/world/Dryden/Lu", Lu)) {ROS_FATAL("Invalid parameters for -/world/Dryden/Lu- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/world/Dryden/Lw", Lw)) {ROS_FATAL("Invalid parameters for -/world/Dryden/Lw- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/world/Dryden/sigmau", sigmau)) {ROS_FATAL("Invalid parameters for -/world/Dryden/sigmau- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/world/Dryden/sigmaw", sigmaw)) {ROS_FATAL("Invalid parameters for -/world/Dryden/sigmaw- in param server!"); ros::shutdown();}
		windDistU = 0;
		for (i=0;i<2;i++)
		{
			windDistV[i] = 0;
			windDistW[i] = 0;
		}
		
			
	}
	
	////////////////////////////////////////////////
	//Read input states and publish environment data
	void environmentModel::callback(const last_letter::SimStates::ConstPtr& InpStates)
	{
		states = *InpStates;
		calcTemp(); //Run 1st		
		calcWind();
		calcDens();
		calcPres();
		calcGrav();
		environment.header.stamp = ros::Time::now();
		pub.publish(environment);
		dt = (ros::Time::now() - tprev).toSec();
		tprev = ros::Time::now();

	}

	/////////////////////////////////////
	//Calculate wind element in body axes
	void environmentModel::calcWind()
	{
		double Reb[9];
		geometry_msgs::Vector3 airspeed;
		double Va, input, temp[2];
		
		if (dt > 2/simRate)
		{
			dt = 2/simRate;
		} //Compensate for possible transient frame drops
		
		//calculate bias wind
		quat2rotmtx(states.pose.orientation, Reb);			
		wind.x = -cos(windDir)*kwind*pow(abs(states.geoid.altitude)+0.001,surfSmooth);
		wind.y = -sin(windDir)*kwind*pow(abs(states.geoid.altitude)+0.001,surfSmooth); //abs is used to avoid exp(x,0) which may return nan
		wind.z = 0;

		if (isnan(wind.x) || isnan(wind.y) || isnan(wind.z))
		{		
			ROS_FATAL("earth wind NAN in environmentNode!");
			cout << wind.x << " " << wind.y << " " << wind.z << endl;
			ros::shutdown();
		}	
		
		//calculate turbulent wind
		airspeed = Reb*states.velocity.linear;
		airspeed = airspeed - wind;
		Va = sqrt(pow(airspeed.x,2)+pow(airspeed.y,2)+pow(airspeed.z,2));
		
		input = (((double)rand()) / (RAND_MAX) - 0.5); //turbulence u-component
		
		windDistU = windDistU*(1-Va/Lu*dt) + sigmau*sqrt(2*Va/Lu)*dt*input;
		
		input = (((double)rand()) / (RAND_MAX) - 0.5); //turbulence v-component
		temp[0] = windDistV[0];
		temp[1] = windDistV[1];
		windDistV[1] = -pow(Va/Lu,2)*dt*temp[0] + temp[1] + sigmau*sqrt(3*Va/Lu)*Va/(sqrt(3)*Lu)*dt*input;
		windDistV[0] = (1.0-2.0*Va/Lu*dt)*temp[0] + dt*temp[1] + sigmau*sqrt(3*Va/Lu)*dt*input;
		
		input = ((double)rand() / (RAND_MAX) - 0.5); //turbulence w-component
		temp[0] = windDistW[0];
		temp[1] = windDistW[1];
		windDistW[1] = -pow(Va/Lw,2)*dt*temp[0] + temp[1] + sigmaw*sqrt(3*Va/Lw)*Va/(sqrt(3)*Lw)*dt*input;
		windDistW[0] = (1.0-2.0*Va/Lw*dt)*temp[0] + dt*temp[1] + sigmaw*sqrt(3*Va/Lw)*dt*input;
		
		environment.wind = Reb/wind; //Rotate bias wind in body axes
		
		environment.wind.x +=windDistU; //Add turbulence
		environment.wind.y +=windDistV[0];
		environment.wind.z +=windDistW[0];
		
		if (isnan(environment.wind.x) || isnan(environment.wind.y) || isnan(environment.wind.z))
		{		
			ROS_FATAL("body wind NAN in environmentNode!");
			cout << environment.wind.x << " " << environment.wind.y << " " << environment.wind.z << endl;
			ros::shutdown();
		}	
	}

	///////////////////////
	//Calculate air density
	void environmentModel::calcDens()
	{
		double Hb = 0, Tb = T0, Pb = P0, L = L0;
		double alt2pressRatio = (Pb / P0) * pow(1 - (L / Tb) * (states.geoid.altitude/1000.0 - Hb), ((1000 * g) / (Rd * L))); //Corrected to 1 - (L/...)
		double alt2tempRatio =  environment.temperature / T0;
		double density = Rho0 * alt2pressRatio  / alt2tempRatio;	
		environment.density = density;
	}

	///////////////////////////////
	//Calculate barometric pressure
	void environmentModel::calcPres()
	{
		double pressure;
		double Hb = 0, Tb = T0, Pb = P0, L = L0;
		pressure = Pb * pow(1 - (L / Tb) * (states.geoid.altitude/1000.0 - Hb), ((1000 * g) / (Rd * L))); //Corrected to 1 - (L/...)
		environment.pressure = pressure;
	}

	///////////////////////
	//Calculate temperature
	void environmentModel::calcTemp()
	{		
		environment.temperature = T0 + states.geoid.altitude/1000.0 * L0;
	}

	///////////////////
	//Calculate gravity
	void environmentModel::calcGrav()
	{
		double slat2 = pow(sin(M_PI/180*states.geoid.latitude),2);
		double Re2 = pow(R_earth,2);

		double grav0 = g * (1.0+0.00193185138639 * slat2) / sqrt(1.0-0.00669437999013 *slat2);
		double gravity = grav0 * (1.0 - grav_temp * states.geoid.altitude + 3.0 *(pow(states.geoid.altitude,2)/Re2) );
		environment.gravity = gravity;
	}



///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

	ros::init(argc, argv, "environmentNode");
	ros::NodeHandle n;
	
	last_letter::SimStates states;
	environmentModel env;
	
	ros::Subscriber sub = n.subscribe("states",1,&environmentModel::callback, &env);	
	pub = n.advertise<last_letter::Environment>("environment",1000);
	
	ros::Duration(3).sleep(); //wait for other nodes to get raised
	double simRate;	
	ros::param::get("simRate",simRate); //frame rate in Hz	
	ros::Rate spinner(simRate);
	
	ROS_INFO("environmentNode up");	
	env.tprev = ros::Time::now();
	spinner.sleep();
	
	while (ros::ok())
	{
		ros::spinOnce();
		spinner.sleep();

		if (isnan(states.velocity.linear.x))
		{		
			ROS_FATAL("State NAN! in environmentNode");
			break;
		}
	}
	
	return 0;
	
}
