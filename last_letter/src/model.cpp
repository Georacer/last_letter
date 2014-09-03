#include "model.hpp"


		
///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

	ros::init(argc, argv, "simNode");
	ros::NodeHandle n;

	ros::Duration(3).sleep(); //wait for other nodes to get raised
	double simRate;
	ros::param::get("simRate",simRate); //frame rate in Hz
	ros::Rate spinner(simRate);
	
	ModelPlane uav(n);
	spinner.sleep();
	ROS_INFO("simNode up");
	
	while (ros::ok())
	{
		uav.step();
		ros::spinOnce();
		spinner.sleep();

		if (isnan(uav.states.velocity.linear.x))
		{		
			ROS_FATAL("State NAN!");
			break;
		}
	}
	
	return 0;
	
}
