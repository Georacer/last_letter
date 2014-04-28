#include "environment.hpp"

ros::Publisher pub;

///////////////////
//Environment Class
///////////////////

	/////////////
	//Constructor
	environmentModel::environmentModel()
	{
		if(!ros::param::getCached("/world/groundTemp", T0)) {ROS_FATAL("Invalid parameters for -/world/groundTemp- in param server!"); ros::shutdown();}
		T0 += 274.15;
		if(!ros::param::getCached("/world/groundPres", P0)) {ROS_FATAL("Invalid parameters for -/world/groundPres- in param server!"); ros::shutdown();}
		g = 9.80665;
		Rd = 287.05307;
		L0 = -6.5;
		Rho0 = 1.2250;
		R_earth = 6378137.0;
		f_earth = 1.0/298.257223563;
		e_earth = sqrt(2.0*f_earth - f_earth*f_earth);
		RP_earth = R_earth*(1.0-e_earth*e_earth);
		omega_earth = 7.292115e-5;
		grav_const = 3.986004418e14;
		grav_earth =  9.7803267714;
		Re = 6378137.0;
		grav_temp = (2.0/R_earth)*(1.0+f_earth+(omega_earth*omega_earth)*(R_earth*R_earth)*RP_earth/grav_const);
		if(!ros::param::getCached("/world/windRef", windRef)) {ROS_FATAL("Invalid parameters for -/world/windRef- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/world/windRefAlt", windRefAlt)) {ROS_FATAL("Invalid parameters for -/world/windRefAlt- in param server!"); ros::shutdown();}			
		if(!ros::param::getCached("/world/windDir", windDir)) {ROS_FATAL("Invalid parameters for -/world/windDir- in param server!"); ros::shutdown();}
		if(!ros::param::getCached("/world/surfSmooth", surfSmooth)) {ROS_FATAL("Invalid parameters for -/world/surfSmooth- in param server!"); ros::shutdown();}
		windDir = windDir*M_PI/180; //convert to rad
		kwind = windRef/pow(windRefAlt,surfSmooth);
			
	}
	
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
	}
	
	void environmentModel::calcWind()
	{
		double Reb[9];
		quat2rotmtx(states.pose.orientation, Reb);			
		wind.x = -cos(windDir)*kwind*pow(states.geoid.altitude,surfSmooth);
		wind.y = -sin(windDir)*kwind*pow(states.geoid.altitude,surfSmooth);
		wind.z = 0;
		
		environment.wind = Reb/wind;
	}
	
	void environmentModel::calcDens()
	{
		double Hb = 0, Tb = T0, Pb = P0, L = L0;
		double alt2pressRatio = (Pb / P0) * pow(1 - (L / Tb) * (states.geoid.altitude/1000.0 - Hb), ((1000 * g) / (Rd * L)));
		double alt2tempRatio =  environment.temperature / T0;
		double density = Rho0 * alt2pressRatio  / alt2tempRatio;	
		environment.density = density;
	}
	
	void environmentModel::calcPres()
	{
		double pressure;
		double Hb = 0, Tb = T0, Pb = P0, L = L0;
		pressure = Pb * pow(1 - (L / Tb) * (states.geoid.altitude/1000.0 - Hb), ((1000 * g) / (Rd * L))); //Corrected to 1 - (L/...)
		environment.pressure = pressure;
	}
	
	void environmentModel::calcTemp()
	{		
		environment.temperature = T0 + states.geoid.altitude/1000.0 * L0;
	}
	
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
	
	while (ros::ok())
	{
		ros::spinOnce();
		spinner.sleep();

		if (isnan(states.velocity.linear.x))
		{		
			ROS_FATAL("State NAN!");
			break;
		}
	}
	
	return 0;
	
}
