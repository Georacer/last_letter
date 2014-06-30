#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "virtual_trim_joystick");
	ros::NodeHandle n;
	ROS_INFO("created nodehandle");
	sensor_msgs::Joy trimInput;
	trimInput.axes.resize(4);
	double ail, elev, throt, rud;
	if(!ros::param::getCached("init/aileron", ail)) {ROS_FATAL("Invalid parameters for -init/position- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("init/elevator", elev)) {ROS_FATAL("Invalid parameters for -init/position- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("init/throttle", throt)) {ROS_FATAL("Invalid parameters for -init/position- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("init/rudder", rud)) {ROS_FATAL("Invalid parameters for -init/position- in param server!"); ros::shutdown();}
	trimInput.axes[0] = -1 * ail; //Set aileron
	trimInput.axes[1] = 1 * elev; //Set elevator
	trimInput.axes[2] = -1 * rud; //Set rudder
	trimInput.axes[3] = (2 * throt -1); //Set throttle
	ros::Publisher pub = n.advertise<sensor_msgs::Joy>("joy",1);
	ROS_INFO("virtual Joystick ready");
	
	while (ros::ok())
	{
		ros::spinOnce();
		trimInput.header.stamp = ros::Time::now();
		pub.publish(trimInput);
	}
	
	return 0;
}
