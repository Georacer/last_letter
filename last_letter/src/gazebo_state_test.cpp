//Include Gazebo messages
#include "gazebo_msgs/ModelState.h"
#include "ros/ros.h"

class Listener{
public:
	gazebo_msgs::ModelState GazeboState; // Gazebo state
	ros::Subscriber subGazeboState; // Subscriber to the Gazebo plugin

	Listener(ros::NodeHandle n);
	~Listener();
	void getState(gazebo_msgs::ModelState state);
};

Listener::Listener(ros::NodeHandle n)
{
	subGazeboState = n.subscribe("modelState",1,&Listener::getState, this); //model control input subscriber
}

Listener::~Listener()
{

}

void Listener::getState(gazebo_msgs::ModelState state)
{
	GazeboState = state;
	ROS_INFO("Received a new Gazebo state");
}

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

	ros::init(argc, argv, "gazebo_state_test");
	ros::NodeHandle n;

	Listener eve(n);

	ROS_INFO("gazebo_state_test node up");

	while (ros::ok())
	{
		ros::spin();
	}

	return 0;
}