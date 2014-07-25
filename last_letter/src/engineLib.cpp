#include "engineLib.hpp"

engine::engine()
{
}

engine::~engine()
{
}

engBeard::engBeard():engine()
{
	std::cout << "reading parameters for new Beard engine" << std::endl;
	omega = 0;
	if(!ros::param::getCached("motor/s_prop", s_prop)) {ROS_FATAL("Invalid parameters for -s_prop- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/c_prop", c_prop)) {ROS_FATAL("Invalid parameters for -c_prop- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_motor", k_motor)) {ROS_FATAL("Invalid parameters for -k_motor- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_t_p", k_t_p)) {ROS_FATAL("Invalid parameters for -k_t_p- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_omega", k_omega)) {ROS_FATAL("Invalid parameters for -k_omega- in param server!"); ros::shutdown();}
}

engBeard::~engBeard()
{
}

void engBeard::step(last_letter::SimStates states, last_letter::Environment environment, double input[4], double dtIn)
{
	geometry_msgs::Vector3 temp;
	temp.x = states.velocity.linear.x - environment.wind.x;
	temp.y = states.velocity.linear.y - environment.wind.y;
	temp.z = states.velocity.linear.z - environment.wind.z;
	geometry_msgs::Vector3 airData = getAirData(temp);
	airspeed = airData.x;
	rho = environment.density;
	deltat = input[2];
	dt = dtIn;
	omega = 1 / (0.5 + dt) * (0.5 * omega + dt * deltat * k_motor);
}

geometry_msgs::Vector3 engBeard::getForce()
{
	double tx = 1.0/2.0*rho*s_prop*c_prop*(pow(omega,2)-pow(airspeed,2));
//		double tx = 1.0/2.0*rho*s_prop*c_prop*(pow(k_motor*deltat,2)-pow(airspeed,2));
	double ty = 0;
	double tz = 0;
	
	geometry_msgs::Vector3 output;
	output.x = tx;
	output.y = ty;
	output.z = tz;
	return output;
}

geometry_msgs::Vector3 engBeard::getTorque()
{
	double lm = -k_t_p*pow(k_omega*omega/k_motor,2);
//		double lm = -k_t_p*pow(k_omega*deltat,2);
	double mm = 0;
	double nm = 0;
	
	geometry_msgs::Vector3 output;
	output.x = lm;
	output.y = mm;
	output.z = nm;
	return output;
}
