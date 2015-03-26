#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <last_letter_msgs/SimPWM.h>
#include <last_letter_msgs/SimStates.h>

ros::Publisher pub;
ros::Subscriber sub;

ros::Time simTime(0.0);
ros::Duration dt;
rosgraph_msgs::Clock simClock;

////////////////////////////////////
// Control input trigger callback //
////////////////////////////////////
void controlsCallback(const last_letter_msgs::SimPWM pwm)
{
	simTime = simTime + dt;
	simClock.clock = simTime;
	pub.publish(simClock);
}

///////////////////////////////////////
// Simulation state trigger callback //
///////////////////////////////////////
void stateCallback(const last_letter_msgs::SimStates state)
{
	simTime = simTime + dt;
	simClock.clock = simTime;
	pub.publish(simClock);
}

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{
	ROS_INFO("Initializing clock node");
	ros::init(argc, argv, "simClockNode");
	ros::NodeHandle n;
	pub = n.advertise<rosgraph_msgs::Clock>("/clock",1000);
	sub = n.subscribe("ctrlPWM", 100, controlsCallback);

	ros::WallDuration(3).sleep(); //wait for other nodes to get raised
	double simRate;
	ros::param::get("/world/simRate",simRate); //frame rate in Hz
	double deltaT;
	ros::param::get("/world/deltaT",deltaT); //frame rate in Hz
	int timeControls;
	ros::param::get("/world/timeControls",timeControls); //frame rate in Hz

	ros::WallRate spinner(simRate);
	dt = ros::Duration(deltaT);

	ROS_INFO("simClockNode up");

	if (timeControls==0) // Default real-time simulation
	{
		ROS_INFO("Using default real-time simulation clock");
		sub = n.subscribe("states", 100, stateCallback);
		simClock.clock = simTime;
		pub.publish(simClock);
		while (ros::ok())
		{
			ros::spinOnce();
			spinner.sleep();
		}
	}
	else if (timeControls==1) // Simulation waits for controls message
	{
		ROS_INFO("Using controls-triggered simulation clock");
		sub = n.subscribe("ctrlPWM", 100, controlsCallback);
		simClock.clock = simTime;
		pub.publish(simClock);
		while (ros::ok())
		{
			ros::spin();
		}
	}
	else if (timeControls==2) // Simulation runs as fast as possible
	{
		ROS_INFO("Using free-spinning simulation clock");
		sub = n.subscribe("states", 100, stateCallback);
		simClock.clock = simTime;
		pub.publish(simClock);
		while (ros::ok())
		{
			ros::spin();
		}
	}
	else {
		ROS_ERROR("Invalid timeControls value!");
		ros::shutdown();
	}

	return 0;

}
