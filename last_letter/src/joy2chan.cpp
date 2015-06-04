#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <cstdlib>

#include <last_letter_msgs/SimPWM.h>

ros::Publisher pub;

int axisIndex[11];
int buttonIndex[11];
double throwIndex[11];


void joy2chan(sensor_msgs::Joy joyMsg)
{
	last_letter_msgs::SimPWM channels;
	double input[11];
	int i;
	for (i = 0; i <= 11; i++) {
		if (axisIndex[i] != -1) { // if an axis is assigned in this channel
			input[i] = 1.0/throwIndex[i]*joyMsg.axes[axisIndex[i]];
		}
		else if (buttonIndex[i] != -1) {
			input[i] = 1.0/throwIndex[i]*joyMsg.buttons[buttonIndex[i]];
		}
		else {
			input[i] = 0.0;
		}
	}

	channels.value[0] = (unsigned int)(input[0]*500+ 1500); // Aileron channel
	channels.value[1] = (unsigned int)(input[1]*500+ 1500); // Elevator channel
	channels.value[2] = (unsigned int)((input[2]+1)*500+ 1000); // Throttle channel
	channels.value[3] = (unsigned int)(input[3]*500+ 1500); // Rudder channel
	channels.value[4] = (unsigned int)(input[4]*500 + 1500); // Steering gear channel
	channels.value[5] = (unsigned int)(input[5]*1000 + 1000); // Breaks channel
	channels.value[6] = (unsigned int)(input[6]*500 + 1500); // Generic channel
	channels.value[7] = (unsigned int)(input[7]*500 + 1500); // Generic channel
	channels.value[8] = (unsigned int)(input[8]*500 + 1500); // Generic channel
	channels.value[9] = (unsigned int)(input[9]*1000 + 1000); // Init channel
	channels.value[10] = (unsigned int)(input[10]*1000 + 1000); // Reset channel
	channels.header.stamp = ros::Time::now();
	pub.publish(channels);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joystick_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("joy",1,joy2chan);
	pub = n.advertise<last_letter_msgs::SimPWM>("rawPWM",1);

	// Read the controller configuration parameters from the HID.yaml file
	XmlRpc::XmlRpcValue listInt, listDouble;
	int i;
	if(!ros::param::getCached("/HID/throws", listDouble)) {ROS_FATAL("Invalid parameters for -/HID/throws- in param server!"); ros::shutdown();}
	for (i = 0; i < listDouble.size(); ++i) {
		ROS_ASSERT(listDouble[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		throwIndex[i]=listDouble[i];
	}
	std::cout << "Reading input axes" << std::endl;
	if(!ros::param::getCached("/HID/axes", listInt)) {ROS_FATAL("Invalid parameters for -/HID/axes- in param server!"); ros::shutdown();}
	for (i = 0; i < listInt.size(); ++i) {
		ROS_ASSERT(listInt[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
		axisIndex[i]=listInt[i];
	}
	ROS_INFO("Reading input buttons configuration");
	if(!ros::param::getCached("/HID/buttons", listInt)) {ROS_FATAL("Invalid parameters for -/HID/buttons- in param server!"); ros::shutdown();}
	for (i = 0; i < listInt.size(); ++i) {
		ROS_ASSERT(listInt[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
		buttonIndex[i]=listInt[i];
	}


	// Enter spin
	while (ros::ok())
	{
		ros::spin();
	}

	return 0;
}
