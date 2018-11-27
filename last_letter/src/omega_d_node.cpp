#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include <last_letter/ControlConfig.h>

std_msgs::Float64 omega_d;

void GainsCallback(last_letter::ControlConfig &config, uint32_t level)
{
	omega_d.data=config.omega_d;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "omega_d_node");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("/last_letter/omega_d", 1);

	ros::Rate loop_rate(1000);

	dynamic_reconfigure::Server<last_letter::ControlConfig> server;
	dynamic_reconfigure::Server<last_letter::ControlConfig>::CallbackType f;

	f = boost::bind(&GainsCallback, _1, _2);
	server.setCallback(f);

	while (ros::ok())
	{

		chatter_pub.publish(omega_d);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
