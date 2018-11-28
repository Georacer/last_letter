#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include <last_letter/ControlConfig.h>
#include <geometry_msgs/Vector3.h>

std_msgs::Float64 omega_d;
geometry_msgs::Vector3 motor_angle;

void GainsCallback(last_letter::ControlConfig &config, uint32_t level)
{
	omega_d.data=config.omega_d;
	motor_angle.x=config.motor_angle_x;
	motor_angle.y=config.motor_angle_y;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "box_input_node");

	ros::NodeHandle n;

	ros::Publisher chatter_pub1 = n.advertise<std_msgs::Float64>("/last_letter/omega_d", 1);
	ros::Publisher chatter_pub2 = n.advertise<geometry_msgs::Vector3>("/last_letter/motor_angle", 1);

	ros::Rate loop_rate(1000);

	dynamic_reconfigure::Server<last_letter::ControlConfig> server;
	dynamic_reconfigure::Server<last_letter::ControlConfig>::CallbackType f;

	f = boost::bind(&GainsCallback, _1, _2);
	server.setCallback(f);

	while (ros::ok())
	{

		chatter_pub1.publish(omega_d);
		chatter_pub2.publish(motor_angle);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
