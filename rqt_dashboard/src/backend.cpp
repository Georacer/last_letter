#include <ros/ros.h>
#include <cstdlib>
#include <math.h>
#include <Eigen/Eigen>

#include "uav_utils.hpp"
#include "math_utils.hpp"

#include <geometry_msgs/Vector3.h>
#include <last_letter_msgs/SimStates.h>
#include <last_letter_msgs/Environment.h>
#include <rqt_dashboard/Dashboard.h>

#define averageN 10
#define filtCoef 0.9

rqt_dashboard::Dashboard dash;
last_letter_msgs::Environment environment;
last_letter_msgs::SimStates states;
ros::Time tprev;
double altPrev=0;

void state_callback(last_letter_msgs::SimStates p_state)
{
	states = p_state;
}

void env_callback(last_letter_msgs::Environment env)
{
	environment = env;
}

// void euler_callback(geometry_msgs::Vector3 euler)
// {
// 	dash.euler = euler;
// }

void calculations(void)
{
	// Calculate relative air velocity vector
	double x, y, z;
	x = states.velocity.linear.x - environment.wind.x;
	y = states.velocity.linear.y - environment.wind.y;
	z = states.velocity.linear.z - environment.wind.z;
	Eigen::Vector3d tempVec(x, y, z);
	Eigen::Vector3d airdata = getAirData(tempVec);
	dash.airspeed = airdata.x();
	dash.alpha = airdata.y() * 180/M_PI;
	dash.beta = airdata.z() * 180/M_PI;
	Eigen::Quaterniond tempQuat(states.pose.orientation.w, states.pose.orientation.x, states.pose.orientation.y, states.pose.orientation.z);
	Eigen::Vector3d eulVect(quat2euler(tempQuat));
	dash.euler.x = eulVect.x() * 180/M_PI;
	dash.euler.y = eulVect.y() * 180/M_PI;
	dash.euler.z = eulVect.z() * 180/M_PI;
	dash.climbRate = states.geoid.velocity.z;
	dash.altitude = states.geoid.altitude;
	dash.rotorspeed = states.rotorspeed[0]*60.0/2.0/M_PI; // Convert from RadPS to RPM
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dash_backend");
	ros::NodeHandle n;
	ros::Rate spinner(100);
	tprev = ros::Time::now();
	ros::Subscriber subStates = n.subscribe("states", 1, state_callback);
	// ros::Subscriber subEuler = n.subscribe("euler", 1, euler_callback);
	ros::Subscriber subEnv = n.subscribe("environment", 1, env_callback);
	ros::Publisher pub = n.advertise<rqt_dashboard::Dashboard>("dashboard", 1);
	// Initialize rotorspeed array
	states.rotorspeed.clear();
	states.rotorspeed.push_back((double) 0);
	ROS_INFO("dashboard backend ready");

	while (ros::ok())
	{
		dash.header.stamp = ros::Time::now();
//		ROS_ERROR("Stamped message");
		calculations();
//		ROS_ERROR("Done calculations");
		pub.publish(dash);
//		ROS_ERROR("Published message");
		ros::spinOnce();
		spinner.sleep();
	}
		ROS_ERROR("Exited Dashboard backend");
	return 0;
}
