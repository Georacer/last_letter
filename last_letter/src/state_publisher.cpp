#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include <last_letter_msgs/SimStates.h>

void broadcast(const last_letter_msgs::SimStates& states){
	static tf::TransformBroadcaster broadcaster;
	tf::Transform statesTf;

	statesTf.setOrigin( tf::Vector3(states.pose.position.x, states.pose.position.y, states.pose.position.z));
	statesTf.setRotation( tf::Quaternion(states.pose.orientation.x,states.pose.orientation.y,states.pose.orientation.z,states.pose.orientation.w));
	broadcaster.sendTransform(tf::StampedTransform(statesTf, states.header.stamp, "map", "base_link"));

	}


int main(int argc, char **argv){
	ros::init(argc,argv,"state_publisher");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("states",1000, broadcast);
	while (ros::ok())
	{
		ros::spin();
	}
	return 0;
}
