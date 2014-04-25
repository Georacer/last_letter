#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include "last_letter/SimStates.h"

void broadcast(const last_letter::SimStates& states){
	static tf::TransformBroadcaster broadcaster;
	tf::Transform statesTf;
	tf::Transform groundCamera;
	tf::Transform planeCamera;
	//geometry_msgs::TransformStamped statesTf;
	//statesTf.transform.translation.x = states.pose.pose.position.x;
	//statesTf.transform.translation.y = 0.0;
	//statesTf.transform.translation.z = 0.0;
	//statesTf.transform.
	//statesTf.header.stamp = states.header.stamp;
	//statesTf.child_frame_id="base_link";
	statesTf.setOrigin( tf::Vector3(states.pose.position.x, states.pose.position.y, states.pose.position.z));
	statesTf.setRotation( tf::Quaternion(states.pose.orientation.x,states.pose.orientation.y,states.pose.orientation.z,states.pose.orientation.w));
	broadcaster.sendTransform(tf::StampedTransform(statesTf, states.header.stamp, "map", "base_link"));

	groundCamera.setOrigin(tf::Vector3(0,0,0));
	groundCamera.setRotation(tf::Quaternion(1,0,0,0));
	broadcaster.sendTransform(tf::StampedTransform(groundCamera, states.header.stamp, "map", "ground_camera"));

	planeCamera.setRotation(tf::Quaternion(1,0,0,0));
	broadcaster.sendTransform(tf::StampedTransform(planeCamera, states.header.stamp, "base_link", "plane_camera"));
	}


int main(int argc, char **argv){
	ros::init(argc,argv,"state_publisher");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("sim/states",1000, broadcast);
	//tf::TransformBroadcaster br_wheels;
	ros::spin();
	return 0;
}
