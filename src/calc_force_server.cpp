#include "ros/ros.h"
#include "last_letter/calc_force.h"
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

bool calc_force_callback(last_letter::calc_force::Request &req, last_letter::calc_force::Response &res)
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
		ROS_ERROR("Failed to call service calc_air_data in %s","force_server");
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
		ROS_ERROR("Failed to call service c_lift_a in %s","force_server");
	}
	double c_lift_a = liftService.response.c_lift_a;
	
	last_letter::c_drag_a dragService;
	dragService.request.alpha = alpha;
	if (!dragClient.call(dragService)){
		ROS_ERROR("Failed to call service c_drag_a in %s","force_server");
	}	
	double c_drag_a = dragService.response.c_drag_a;

	//read from parameter server
	double rho,g,m;
	double c_lift_q,c_lift_deltae,c_drag_q,c_drag_deltae;
	double c,b,s;
	double c_y_0,c_y_b,c_y_p,c_y_r,c_y_deltaa,c_y_deltar;
	double s_prop,c_prop,k_motor;
	
	ros::param::getCached("/world/rho",rho);
	ros::param::getCached("/world/g",g);
	ros::param::getCached("/airframe/m",m);
	ros::param::getCached("/airframe/c_lift_q",c_lift_q);
	ros::param::getCached("/airframe/c_lift_deltae",c_lift_deltae);
	ros::param::getCached("/airframe/c_drag_q",c_drag_q);
	ros::param::getCached("/airframe/c_drag_deltae",c_drag_deltae);
	ros::param::getCached("/airframe/c",c);
	ros::param::getCached("/airframe/b",b);
	ros::param::getCached("/airframe/s",s);
	ros::param::getCached("/airframe/c_y_0",c_y_0);
	ros::param::getCached("/airframe/c_y_b",c_y_b);
	ros::param::getCached("/airframe/c_y_p",c_y_p);
	ros::param::getCached("/airframe/c_y_r",c_y_r);
	ros::param::getCached("/airframe/c_y_deltaa",c_y_deltaa);
	ros::param::getCached("/airframe/c_y_deltar",c_y_deltar);
	ros::param::getCached("/motor/s_prop",s_prop);
	ros::param::getCached("/motor/c_prop",c_prop);
	ros::param::getCached("/motor/k_motor",k_motor);
	
	//convert coefficients to the body frame
	double c_x_a = -c_drag_a*cos(alpha)-c_lift_a*sin(alpha);
	double c_x_q = -c_drag_q*cos(alpha)-c_lift_q*sin(alpha);
	double c_x_deltae = -c_drag_deltae*cos(alpha)-c_lift_deltae*sin(alpha);
	double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
	double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);
	double c_z_deltae = -c_drag_deltae*sin(alpha)-c_lift_deltae*cos(alpha);
	
	//read orientation and convert to Euler angles
	tf::Quaternion quat = tf::Quaternion(states.pose.pose.orientation.x, states.pose.pose.orientation.y, states.pose.pose.orientation.z, states.pose.pose.orientation.w);
	if ( isnan(quat.length()) || (abs(quat.length())<0.9) ){ //check for invalid quaternion
		quat = tf::Quaternion(0, 0, 0, 1);
	}
	//quat.normalize();
	//ROS_INFO("Quaterion: %g, %g, %g, %g, len: %g",quat[0], quat[1], quat[2], quat[3], quat.length()); //***
	double phi,theta,psi;
	tf::Matrix3x3(quat).getEulerYPR(psi, theta, phi);
	//ROS_INFO("RPY: %g, %g, %g",phi, theta, psi); //***
	
	//read angular rates
	double p = states.twist.twist.angular.x;
	double q = states.twist.twist.angular.y;
	double r = states.twist.twist.angular.z;		
	
	//calculate gravity force
	double gx = -m*g*sin(theta);
	double gy = m*g*cos(theta)*sin(phi);
	double gz = m*g*cos(theta)*cos(phi);
	//ROS_INFO("m=%g, g=%g",m,g);
	
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

	//Sum forces
	double fx = gx+ax+tx;
	double fy = gy+ay+ty;
	double fz = gz+az+tz;
	
	/*ROS_INFO("G: %g, %g, %g, N:%g",gx, gy, gz, sqrt(gx*gx+gy*gy+gz*gz));	
	ROS_INFO("A: %g, %g, %g, N:%g",ax, ay, az, sqrt(ax*ax+ay*ay+az*az));	
	ROS_INFO("T: %g, %g, %g, N:%g",tx, ty, tz, sqrt(tx*tx+ty*ty+tz*tz));
	ROS_INFO("F: %g, %g, %g",fx, fy, fz);*/
	
	res.force.x = fx;
	res.force.y = fy;
	res.force.z = fz;
	
	//ROS_INFO("end of service"); //***
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calc_force_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("calc_force",calc_force_callback);
	airDataClient = n.serviceClient<last_letter::calc_air_data>("calc_air_data");
	liftClient = n.serviceClient<last_letter::c_lift_a>("c_lift_a");
	dragClient = n.serviceClient<last_letter::c_drag_a>("c_drag_a");	
	ROS_INFO("calc_force_server ready");
	ros::spin();

	return 0;
}
