#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
//#include "LinearMath/btMatrix3x3.h"
#include <cstdlib>
#include <math.h>

#include "last_letter/SimStates.h"
ros::Publisher pub;

void translate_callback(last_letter::SimStates Odo)
{
	tf::Quaternion quat = tf::Quaternion(Odo.pose.orientation.x, Odo.pose.orientation.y, Odo.pose.orientation.z, Odo.pose.orientation.w);
	if ( isnan(quat.length()) || (abs(quat.length())<0.9) ){ //check for invalid quaternion
		quat = tf::Quaternion(0, 0, 0, 1);
	}
	//quat.normalize();
	//ROS_INFO("Quaterion: %g, %g, %g, %g, len: %g",quat[0], quat[1], quat[2], quat[3], quat.length()); //***
	double phi,theta,psi;
	tf::Matrix3x3(quat).getEulerYPR(psi, theta, phi);
	geometry_msgs::Vector3 eulerVec;
	eulerVec.x = phi;
	eulerVec.y = theta;
	eulerVec.z = psi;
	pub.publish(eulerVec);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "quat2euler");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/sim/states", 1000, translate_callback);
	pub = n.advertise<geometry_msgs::Vector3>("/sim/euler", 1000);
	ROS_INFO("quat2euler ready");
	ros::spin();
	
	return 0;
}
