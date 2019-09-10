#include <ros/ros.h>
#include "model.hpp"

#include "prog_utils.hpp"
#include "uav_utils.hpp"

#include <rosgraph_msgs/Clock.h>

//////////////////////////////
// Simulation step callback //
//////////////////////////////
void stepCallback(const rosgraph_msgs::Clock time)
{
	uav->step();
}

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

	ros::init(argc, argv, "simNode");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/clock",1000, stepCallback);

	ROS_INFO("Generating UavModelWrapper");
	uav = new UavModelWrapper(n); //Create a UavStruct by passing the configurations bundle struct

	int statusClock=0, statusArg=0;
	while (statusClock!=1) {
		ros::param::get("nodeStatus/clock", statusClock);
	}
	// Deprecated as environment is embedded in UAV model
	// while (statusEnv!=1) {
	// 	ros::param::get("nodeStatus/environment", statusEnv);
	// }
	while (statusArg!=1) {
		ros::param::get("nodeStatus/argumentHandler", statusArg);
	}

	ros::param::set("nodeStatus/model", 1);

	// ros::WallDuration(3).sleep(); //wait for other nodes to get raised

	ROS_INFO("simNode up");

	// uav->init();

	while (ros::ok())
	{
		ros::spin();
	}

	delete uav;

	return 0;
}
