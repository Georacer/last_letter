#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <cstdlib>
#include <math.h>

#include <mathutils/mathutils.hpp>
#include <last_letter_msgs/SimStates.h>
ros::Publisher pub;

void translate_callback(last_letter_msgs::SimStates Odo)
{
	tf::Quaternion quat = tf::Quaternion(Odo.pose.orientation.x, Odo.pose.orientation.y, Odo.pose.orientation.z, Odo.pose.orientation.w);
	if ( isnan(quat.length()) ) {
		quat = tf::Quaternion(0, 0, 0, 1);
		ROS_ERROR("quaternion length is NaN");
	}
	if (abs(quat.length())<0.9) {
//		ROS_ERROR("quaternion too short");
//		ROS_INFO("Quaterion: %g, %g, %g, %g, len: %g",quat[0], quat[1], quat[2], quat[3], quat.length());
		quat.normalize();
	}
//	double phi,theta,psi;
//	tf::Matrix3x3(quat).getEulerYPR(psi, theta, phi);
	geometry_msgs::Vector3 eulerVec = quat2euler(Odo.pose.orientation);
	eulerVec.x = eulerVec.x*180.0/M_PI;
	eulerVec.y = eulerVec.y*180.0/M_PI;
	eulerVec.z = eulerVec.z*180.0/M_PI;
	pub.publish(eulerVec);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "quat2euler");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("states", 1, translate_callback);
	pub = n.advertise<geometry_msgs::Vector3>("euler", 1000);
	ROS_INFO("quat2euler ready");
	ros::spin();

	return 0;
}
