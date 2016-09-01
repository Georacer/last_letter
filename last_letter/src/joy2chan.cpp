#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <cstdlib>
#include <ros/console.h>

#include <last_letter_msgs/SimPWM.h>

class JoyConverter{
public:
	ros::Subscriber sub;
	ros::Publisher pub;

	int axisIndex[11];
	int buttonIndex[11];
	double throwIndex[11];
	int mixerid;

	JoyConverter(ros::NodeHandle n);
	~JoyConverter();
	void joy2chan(sensor_msgs::Joy joyMsg);
	last_letter_msgs::SimPWM mixer(double * input, int mixerid);
};

JoyConverter::JoyConverter(ros::NodeHandle n)
{

	sub = n.subscribe("joy",1,&JoyConverter::joy2chan,this);
	pub = n.advertise<last_letter_msgs::SimPWM>("rawPWM",1);	// Read the controller configuration parameters from the HID.yaml file

	XmlRpc::XmlRpcValue listInt, listDouble;
	int i;
	if(!ros::param::getCached("/HID/throws", listDouble)) {ROS_FATAL("Invalid parameters for -/HID/throws- in param server!"); ros::shutdown();}
	for (i = 0; i < listDouble.size(); ++i) {
		ROS_ASSERT(listDouble[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		throwIndex[i]=listDouble[i];
	}
	ROS_INFO("Reading input axes");
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
	// Read the mixer type
	if(!ros::param::getCached("/HID/mixerid", mixerid)) {ROS_INFO("No mixing function selected"); mixerid=0;}
}

JoyConverter::~JoyConverter()
{
}

void JoyConverter::joy2chan(sensor_msgs::Joy joyMsg)
{
	ROS_DEBUG("joy2chan: Processing new joy msg");
	last_letter_msgs::SimPWM channels;
	double input[11];
	int i;
	for (i = 0; i < 11; i++) {
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

	ROS_DEBUG("joy2chan: Calling mixing function");
	channels = mixer(input, mixerid);
	for (i=0;i<11;i++) // Cap channel limits
	{
		if (channels.value[i]<1000) channels.value[i]=1000;
		if (channels.value[i]>2100) channels.value[i]=2100;
	}

	channels.header.stamp = ros::Time::now();
	ROS_DEBUG("joy2chan: Publishing channels");
	pub.publish(channels);
	ROS_DEBUG("joy2chan: Callback ended");
}

// Mixer function
last_letter_msgs::SimPWM JoyConverter::mixer(double * input, int mixerid)
{
	last_letter_msgs::SimPWM channels;
	int i;
	switch (mixerid)
	{
	case 0: // No mixing applied
		for (i=0;i<11;i++)
		{
			channels.value[i] = (unsigned int)(input[i]*500 + 1500);
		}
		return channels;
	case 1: // Airplane mixing
		channels.value[0] = (unsigned int)(input[0]*500+ 1500); // Aileron channel
		channels.value[1] = (unsigned int)(input[1]*500+ 1500); // Elevator channel
		channels.value[2] = (unsigned int)((input[2]+1)*500+ 1000); // Throttle channel
		channels.value[3] = (unsigned int)(input[3]*500+ 1500); // Rudder channel
		channels.value[4] = (unsigned int)(input[4]*500 + 1500); // Steering gear channel
		channels.value[5] = (unsigned int)(input[5]*1000 + 1000); // Breaks channel
		channels.value[6] = (unsigned int)(input[6]*500 + 1500); // Generic channel
		channels.value[7] = (unsigned int)(input[7]*500 + 1500); // Generic channel
		channels.value[8] = (unsigned int)(input[8]*500 + 1500); // Generic channel
		channels.value[9] = (unsigned int)(input[9]*1000 + 1000); // Reset channel
		channels.value[10] = (unsigned int)(input[10]*500 + 1500); // Generic channel
		return channels;
	case 2: // Quadrotor mixing
		channels.value[0] = (unsigned int)(500*((input[2]+1) -0.11*input[0] +0.11*input[1] -0.11*input[3]) +1000);
		channels.value[1] = (unsigned int)(500*((input[2]+1) +0.11*input[0] +0.11*input[1] +0.11*input[3]) +1000);
		channels.value[2] = (unsigned int)(500*((input[2]+1) -0.11*input[0] -0.11*input[1] +0.11*input[3]) +1000);
		channels.value[3] = (unsigned int)(500*((input[2]+1) +0.11*input[0] -0.11*input[1] -0.11*input[3]) +1000);
		channels.value[4] = (unsigned int)(input[4]*500 + 1500); // Generic channel
		channels.value[5] = (unsigned int)(input[5]*500 + 1500); // Generic channel
		channels.value[6] = (unsigned int)(input[6]*500 + 1500); // Generic channel
		channels.value[7] = (unsigned int)(input[7]*500 + 1500); // Generic channel
		channels.value[8] = (unsigned int)(input[8]*500 + 1500); // Generic channel
		channels.value[9] = (unsigned int)(input[9]*1000 + 1000); // Reset channel
		channels.value[10] = (unsigned int)(input[10]*1000 + 1000); // Generic channel
		return channels;
	case 3: // Firefly Y6 mixing
		channels.value[0] = (unsigned int)(500*((input[2]+1) -0.11*input[0] +0.11*input[1] -0.11*input[3]) +1000); // top-right motor channel
		channels.value[1] = (unsigned int)(500*((input[2]+1) -0.11*input[0] +0.11*input[1] +0.11*input[3]) +1000); // bottom-right motor channel
		channels.value[2] = (unsigned int)(500*((input[2]+1) +0.00*input[0] -0.11*input[1] -0.11*input[3]) +1000); // top-rear motor channel
		channels.value[3] = (unsigned int)(500*((input[2]+1) +0.00*input[0] -0.11*input[1] +0.11*input[3]) +1000); // bottom-rear motor channel
		channels.value[4] = (unsigned int)(500*((input[2]+1) +0.11*input[0] +0.11*input[1] -0.11*input[3]) +1000); // top-left motor channel
		channels.value[5] = (unsigned int)(500*((input[2]+1) +0.11*input[0] +0.11*input[1] +0.11*input[3]) +1000); // bottom-left channel
		channels.value[6] = (unsigned int)(-input[3]*500 + 1500); // steering wheel channel
		channels.value[7] = (unsigned int)(input[9]*1000 + 1000); // reset channel
		channels.value[8] = (unsigned int)(input[0]*500 + 1500); // aileron channel
		channels.value[9] = (unsigned int)(input[1]*500 + 1500); // elevator channel
		channels.value[10] = (unsigned int)(input[6]*500 + 1500); // motor gimbal channel
		return channels;
	default:
		ROS_FATAL("Invalid parameter for -/HID/mixerid- in param server!");
		ros::shutdown();

	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joystick_node");
	ros::NodeHandle n;

	JoyConverter converter(n);

	// Setting debug level of the node
	// if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
 //   		ros::console::notifyLoggerLevelsChanged();
	// }

	// Enter spin
	while (ros::ok())
	{
		ros::spin();
	}

	return 0;
}
