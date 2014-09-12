#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include <cstdlib>
#include <math.h>
#include "last_letter/SimStates.h"
#include "last_letter/Environment.h"
#include "rqt_dashboard/Dashboard.h"

#define averageN 10
#define filtCoef 0.9

rqt_dashboard::Dashboard dash;
last_letter::Environment environment;
last_letter::SimStates states;
ros::Time tprev;
double altPrev=0;
//int i=0;

void state_callback(last_letter::SimStates Odo)
{
	states = Odo;
}

void env_callback(last_letter::Environment env)
{
	environment = env;
}

void euler_callback(geometry_msgs::Vector3 euler)
{
	dash.euler = euler;
}

/////////////////////////////////////////
//Aerodynamc angles/ airspeed calculation
geometry_msgs::Vector3 getAirData (geometry_msgs::Vector3 speeds)
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
	result.y = alpha*180/M_PI;
	result.z = beta*180/M_PI;
	
	return result;
}

double calcClimb(void)
{
	static double buffer[averageN];
	
	ros::Time now = ros::Time::now();
	double dt = (now - tprev).toSec();
	tprev = now;
	double deltaAlt = (states.geoid.altitude - altPrev);
	altPrev = states.geoid.altitude;
	double climbRate = deltaAlt/dt;
	
	for (int i=0; i<(averageN-1); i++) {
		buffer[i] = buffer[i+1];
	}
	buffer[averageN-1]=climbRate;
	
	double tempSum = 0.0;
	for (int i=0; i<averageN; i++) {
		tempSum += buffer[i];
	}
	climbRate = tempSum/averageN;

	return climbRate;
}

void calculations(void)
{
	geometry_msgs::Vector3 tempVec;
	tempVec = getAirData(states.velocity.linear);
	dash.airspeed = tempVec.x;
	dash.alpha = tempVec.y;
	dash.beta = tempVec.z;
	dash.climbRate = calcClimb(); //RAW warning!
	dash.altitude = states.geoid.altitude;
	dash.rotorspeed = states.rotorspeed[0]*60.0/2.0/M_PI; // Convert from RadPS to RPM
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dash_backend");
	ros::NodeHandle n;
	ros::Rate spinner(100);
	tprev = ros::Time::now();
	ros::Subscriber subStates = n.subscribe("/fw1/states", 1, state_callback);
	ros::Subscriber subEuler = n.subscribe("/fw1/euler", 1, euler_callback);
	ros::Subscriber subEnv = n.subscribe("/fw1/environment", 1, env_callback);
	ros::Publisher pub = n.advertise<rqt_dashboard::Dashboard>("/fw1/dashboard", 1);
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
