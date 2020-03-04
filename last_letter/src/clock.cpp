#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <last_letter_msgs/SimPWM.h>
#include <last_letter_msgs/SimStates.h>

ros::Publisher pub;
ros::Subscriber subCtrl, subRawCtrl, subState;

ros::Time simTime(0.0);
ros::WallTime realTimePrev = ros::WallTime::now(), realTimeNow;
ros::Duration dt;
ros::WallDuration wallCounter;
rosgraph_msgs::Clock simClock;
unsigned long frameCounter = 0;
int timeControls;
int chanPause; // The number of the channel where the pauseSim command is sent
bool pauseSim = false;  // The pauseSimd state
bool pauseButtonPressed = false;

/**
 * @brief Pause command logic
 * 
 * @param pause_pwm Value of the pause channel
 */
void pauseLogic(const uint16_t pause_pwm)
{

	if (pause_pwm>1800)
	{
		if (!pauseButtonPressed)
		{
			pauseButtonPressed = true;

			if (timeControls == 3) // Working by manual stepping
			{
					ROS_INFO("Stepping simulation");
					simTime = simTime + dt;
					simClock.clock = simTime;
					pub.publish(simClock);
					frameCounter++;
			}
			else // Working on rest of stepping methods
			{
				pauseSim = !pauseSim;

				if (pauseSim)
				{
					ROS_INFO("Paused simulation");
				}
				else
				{
					ROS_INFO("Unpausing simulation");
					simTime = simTime + dt;
					simClock.clock = simTime;
					pub.publish(simClock);
					frameCounter++;
				}
			}
		}

	}
	else
	{
		pauseButtonPressed = false;
	}
}

///////////////////////
// Raw control callback
///////////////////////
void rawControlsCallback(const last_letter_msgs::SimPWM pwm)
{
	// Register the pauseSim command
	uint16_t pause_pwm = pwm.value[chanPause];
	pauseLogic(pause_pwm);
}

////////////////////////////////////
// Control input trigger callback //
////////////////////////////////////
void controlsCallback(const last_letter_msgs::SimPWM pwm)
// Callback for the ctrlPWM topic
{
	// Publish simulated time
	if (!pauseSim && (timeControls==1))
	{
		simTime = simTime + dt;
		simClock.clock = simTime;
		pub.publish(simClock);
		frameCounter++;
	}

	// Also forward the raw controls to the ctrlPWM callback. 
	// This is to capture the pause button when rawPWM topic is remapped to ctrlPWM as well
	rawControlsCallback(pwm);
}

///////////////////////////////////////
// Simulation state trigger callback //
///////////////////////////////////////
void stateCallback(const last_letter_msgs::SimStates state)
{
	if (!pauseSim && (timeControls==0 || timeControls==2))
	{
		simTime = simTime + dt;
		simClock.clock = simTime;
		pub.publish(simClock);
		frameCounter++;
	}
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
	subCtrl = n.subscribe("ctrlPWM", 100, controlsCallback);
	subRawCtrl = n.subscribe("rawPWM", 100, rawControlsCallback);
	subState = n.subscribe("states", 100, stateCallback);

	int statusModel=0;
	ros::param::set("nodeStatus/clock", 1);
	while (statusModel!=1) {
		ros::param::get("nodeStatus/model",statusModel);
	}

	// ros::WallDuration(5).sleep(); //wait for other nodes to get raised


	double simRate;
	ros::param::get("/world/simRate",simRate); //frame rate in Hz
	double deltaT;
	ros::param::get("/world/deltaT",deltaT); //simulation time step in seconds
	ros::param::get("/world/timeControls",timeControls); //frame rate in Hz
	n.getParam("chanPause", chanPause); // Read the channel number where the pauseSim command is given
	ROS_INFO("Reading pause from %d channel", chanPause);

	ros::WallRate spinner(simRate);
	dt = ros::Duration(deltaT);

	ROS_INFO("simClockNode up");

	if (timeControls==0) // Default real-time simulation
	{
		ROS_INFO("Using default real-time simulation clock");
		ros::WallDuration(1).sleep(); //wait for other nodes to get raised
		simClock.clock = simTime;
		pub.publish(simClock);
		while (ros::ok())
		{
			realTimeNow = ros::WallTime::now();
			wallCounter = realTimeNow - realTimePrev;
			if (wallCounter.toSec() > 5) {
				ROS_INFO("Simulation rate: %lu Hz", frameCounter/5);
				realTimePrev = realTimeNow;
				frameCounter=0;
			}
			ros::spinOnce();
			spinner.sleep();

		}
	}
	else if (timeControls==1) // Simulation waits for controls message
	{
		ROS_INFO("Using controls-triggered simulation clock");
		simClock.clock = simTime;
		pub.publish(simClock);
		while (ros::ok())
		{
			realTimeNow = ros::WallTime::now();
			wallCounter = realTimeNow - realTimePrev;
			if (wallCounter.toSec() > 5) {
				ROS_INFO("Simulation rate: %lu Hz", frameCounter/5);
				realTimePrev = realTimeNow;
				frameCounter=0;
			}
			ros::spinOnce();

		}
	}
	else if (timeControls==2) // Simulation runs as fast as possible
	{
		ROS_INFO("Using free-spinning simulation clock");
		// ros::WallDuration(3).sleep(); //wait for other nodes to get raised
		simClock.clock = simTime;
		pub.publish(simClock);
		while (ros::ok())
		{
			realTimeNow = ros::WallTime::now();
			wallCounter = realTimeNow - realTimePrev;
			if (wallCounter.toSec() > 5) {
				ROS_INFO("Simulation rate: %lu Hz", frameCounter/5);
				realTimePrev = realTimeNow;
				frameCounter=0;
			}
			ros::spinOnce();
		}
	}
	else if (timeControls==3) // Simulation is manually triggered by the pause button
	{
		ROS_INFO("Manual simulation clock (pause button)");
		simClock.clock = simTime;
		pub.publish(simClock);
		while (ros::ok())
		{
			realTimeNow = ros::WallTime::now();
			wallCounter = realTimeNow - realTimePrev;
			if (wallCounter.toSec() > 5) {
				ROS_INFO("Simulation rate: %lu Hz", frameCounter/5);
				realTimePrev = realTimeNow;
				frameCounter=0;
			}
			ros::spinOnce();
		}
	}
	else {
		ROS_ERROR("Invalid timeControls value!");
		ros::shutdown();
	}

	return 0;

}
