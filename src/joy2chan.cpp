#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <last_letter/inputs.h>

ros::Publisher pub;

void joy2chan(sensor_msgs::Joy joyMsg)
{
	last_letter::inputs channels;
	channels.inputs[0] = -joyMsg.axes[0]*500+ 1500;
	channels.inputs[1] = -joyMsg.axes[1]*500+ 1500;
	channels.inputs[2] = (joyMsg.axes[3]+1)*500+ 1000;
	channels.inputs[3] = -joyMsg.axes[2]*500+ 1500;
	channels.header.stamp = ros::Time::now();
	pub.publish(channels);			
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joystick_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joy",1,joy2chan);
	pub = n.advertise<last_letter::inputs>("sim/input",1);
	ros::spin();
}


