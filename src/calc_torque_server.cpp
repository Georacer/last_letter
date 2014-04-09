#include "ros/ros.h"
#include "last_letter/calc_torque.h"
#include "last_letter/calc_air_data.h"
#include "last_letter/c_lift_a.h"
#include "last_letter/c_drag_a.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
//#include "LinearMath/btMatrix3x3.h"
#include <cstdlib>
#include <math.h>

ros::ServiceClient airDataClient, liftClient, dragClient;

bool calc_torque_callback(last_letter::calc_torque::Request &req, last_letter::calc_torque::Response &res)
{

	//initializing inputs		
	nav_msgs::Odometry states = req.states;
	double inputs[4];
	inputs[0] = req.inputs[0];
	inputs[1] = req.inputs[1];
	inputs[2] = req.inputs[2];
	inputs[3] = req.inputs[3];
	
	//request air data from getAirData server
	last_letter::calc_air_data airDataService;
	airDataService.request.speeds = req.states.twist.twist.linear;
	if (!airDataClient.call(airDataService)){
		ROS_ERROR("Failed to call service calc_air_data in %s","torque_server");
	}
	double airspeed = airDataService.response.airspeed;
	double alpha = airDataService.response.alpha;
	double beta = airDataService.response.beta;
	
	//split control input values
	double deltaa = inputs[0];
	double deltae = inputs[1];
	double deltat = inputs[2];
	double deltar = inputs[3];
	
	//request lift and drag alpha-coefficients from the corresponding server
	last_letter::c_lift_a liftService;
	liftService.request.alpha = alpha;
	if (!liftClient.call(liftService)){
		ROS_ERROR("Failed to call service c_lift_a in %s","torque_server");
	}
	double c_lift_a = liftService.response.c_lift_a;
	
	last_letter::c_drag_a dragService;
	dragService.request.alpha = alpha;
	if (!dragClient.call(dragService)){
		ROS_ERROR("Failed to call service c_drag_a in %s","torque_server");
	}	
	double c_drag_a = dragService.response.c_drag_a;

	//read from parameter server
	double rho;
	double c,b,s;
	double c_l_0, c_l_b, c_l_p, c_l_r, c_l_deltaa, c_l_deltar;
	double c_m_0, c_m_a, c_m_q, c_m_deltae;
	double c_n_0, c_n_b, c_n_p, c_n_r, c_n_deltaa, c_n_deltar;
	double k_t_p, k_omega;
	
	ros::param::getCached("/world/rho",rho);
	ros::param::getCached("/airframe/c",c);
	ros::param::getCached("/airframe/b",b);
	ros::param::getCached("/airframe/s",s);	
	ros::param::getCached("/airframe/c_l_0",c_l_0);
	ros::param::getCached("/airframe/c_l_b",c_l_b);
	ros::param::getCached("/airframe/c_l_p",c_l_p);
	ros::param::getCached("/airframe/c_l_r",c_l_r);
	ros::param::getCached("/airframe/c_l_deltaa",c_l_deltaa);
	ros::param::getCached("/airframe/c_l_deltar",c_l_deltar);
	ros::param::getCached("/airframe/c_m_0",c_m_0);
	ros::param::getCached("/airframe/c_m_a",c_m_a);
	ros::param::getCached("/airframe/c_m_q",c_m_q);
	ros::param::getCached("/airframe/c_m_deltae",c_m_deltae);
	ros::param::getCached("/airframe/c_n_0",c_n_0);
	ros::param::getCached("/airframe/c_n_b",c_n_b);
	ros::param::getCached("/airframe/c_n_p",c_n_p);
	ros::param::getCached("/airframe/c_n_r",c_n_r);
	ros::param::getCached("/airframe/c_n_deltaa",c_n_deltaa);
	ros::param::getCached("/airframe/c_n_deltar",c_n_deltar);
	ros::param::getCached("/motor/k_t_p",k_t_p);
	ros::param::getCached("/motor/k_omega",k_omega);
	
	//read angular rates
	double p = states.twist.twist.angular.x;
	double q = states.twist.twist.angular.y;
	double r = states.twist.twist.angular.z;
	
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
	
	//Sum torques
	double l = la + lm;
	double m = ma + mm;
	double n = na + nm;
	
	/*ROS_INFO("G: %g, %g, %g, N:%g",gx, gy, gz, sqrt(gx*gx+gy*gy+gz*gz));	
	ROS_INFO("A: %g, %g, %g, N:%g",ax, ay, az, sqrt(ax*ax+ay*ay+az*az));	
	ROS_INFO("T: %g, %g, %g, N:%g",tx, ty, tz, sqrt(tx*tx+ty*ty+tz*tz));
	ROS_INFO("F: %g, %g, %g",fx, fy, fz);*/
	
	res.torque.x = l;
	res.torque.y = m;
	res.torque.z = n;
	
	//ROS_INFO("end of service"); //***
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calc_torque_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("calc_torque",calc_torque_callback);
	airDataClient = n.serviceClient<last_letter::calc_air_data>("calc_air_data");
	liftClient = n.serviceClient<last_letter::c_lift_a>("c_lift_a");	
	dragClient = n.serviceClient<last_letter::c_drag_a>("c_drag_a");	
	ROS_INFO("calc_torque_server ready");
	ros::spin();

	return 0;
}
