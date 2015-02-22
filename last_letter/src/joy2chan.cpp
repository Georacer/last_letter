#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "last_letter_msgs/SimPWM.h"

ros::Publisher pub;

void joy2chan(sensor_msgs::Joy joyMsg)
{
	last_letter_msgs::SimPWM channels;
	channels.value[0] = (unsigned int)(-joyMsg.axes[0]*500+ 1500);
	channels.value[1] = (unsigned int)(-joyMsg.axes[1]*500+ 1500);
	channels.value[2] = (unsigned int)((joyMsg.axes[3]+1)*500+ 1000);
	channels.value[3] = (unsigned int)(-joyMsg.axes[2]*500+ 1500);
	channels.value[9] = (unsigned int)(joyMsg.buttons[5]*1000 + 1000); // Reset channel
	channels.value[5] = (unsigned int)(joyMsg.buttons[0]*1000 + 1000); // Breaks channel
	channels.header.stamp = ros::Time::now();
	pub.publish(channels);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joystick_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joy",1,joy2chan);
	pub = n.advertise<last_letter_msgs::SimPWM>("rawPWM",1);

	while (ros::ok())
	{
		ros::spin();
	}

	return 0;
}
