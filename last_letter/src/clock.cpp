#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <last_letter_msgs/SimPWM.h>
#include <last_letter_msgs/SimStates.h>

ros::Publisher pub;
ros::Subscriber subCtrl, subRawCtrl, subState;

ros::Time simTime(0.0);
ros::Duration dt;
double max_clock_rate{1000};
double dead_time{1}; // Time in lack of step after which simulation loop is declared broken
rosgraph_msgs::Clock simClock;
int timeControls;
int chanPause; // The number of the channel where the pauseSim command is sent
bool pauseSim = false;  // The pauseSimd state
bool pauseButtonPressed = false;
bool step_required = false;


/**
 * @brief Class to monitor and publish simulation rate
 * 
 */
class RateMonitor {

	public:
	ros::WallTime realTimeNow;
	ros::WallTime realTimePrev;
	ros::WallTime prevFrameTime;
	ros::WallDuration wallCounter;
	double pub_interval{5};
	unsigned long frameCounter = 0;

	RateMonitor()
	{

	};

	void initialize_ros()
	{
		realTimePrev = ros::WallTime::now();
		prevFrameTime = ros::WallTime::now();
	};

	void add_frames(int count=1)
	{
		frameCounter += count;
		prevFrameTime = ros::WallTime::now();
		publish_simrate();
	};

	double get_time_since_last_frame()
	{
		wallCounter = ros::WallTime::now() - prevFrameTime;
		return wallCounter.toSec();
	};

	void publish_simrate()
	{
		realTimeNow = ros::WallTime::now();
		wallCounter = realTimeNow - realTimePrev;
		if (wallCounter.toSec() > pub_interval) {
			ROS_INFO("Simulation rate: %d Hz", int(frameCounter/pub_interval));
			realTimePrev = realTimeNow;
			frameCounter=0;
		}
	};
};

// Create rate_monitor as a global object
RateMonitor rate_monitor;


void step_simulation()
{
	simTime = simTime + dt;
	simClock.clock = simTime;
	pub.publish(simClock);
	rate_monitor.add_frames();
};


/**
 * @brief Pause command logic
 * 
 * @param pause_pwm Value of the pause channel
 */
void pauseLogic(const uint16_t pause_pwm)
{

	if (pause_pwm>1800)
	// Pause button is pressed
	{
		if (!pauseButtonPressed)
		// It has not been registered
		{
			pauseButtonPressed = true; // Acknowledge button press

			if (timeControls == 3) 
			// Working by manual stepping, trigger simulation step
			{
				step_required = true;
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
					// step_simulation();
				}
			}
		}
	}
	else
	{
		pauseButtonPressed = false; // Acknowlege button release
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
	if (timeControls==1)
	// If simulation is set to run on controls callback, request another step
	{
		step_required = true;
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
	if (timeControls==0 || timeControls==2)
	// If simulation is set to run on fixed or free clock, request another step as soon as the previous state has
	// been calculated and published
	{
		step_required = true;
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
	rate_monitor.initialize_ros();

	// Synchronize clock and model
	int statusModel=0;
	ros::param::set("nodeStatus/clock", 1);
	while (statusModel!=1) {
		ros::param::get("nodeStatus/model",statusModel);
	}

	// Read parameters regarding simulation behaviour
	double simRate;
	ros::param::get("/world/simRate",simRate); //frame rate in Hz
	double deltaT;
	ros::param::get("/world/deltaT",deltaT); //simulation time step in seconds
	dt = ros::Duration(deltaT);
	ros::param::get("/world/timeControls",timeControls); //frame rate in Hz
	n.getParam("chanPause", chanPause); // Read the channel number where the pauseSim command is given
	ROS_INFO("Reading pause from %d channel", chanPause);

	int ctrl_channel_rate = simRate; // Expected publish rate on the controls channel
	int frame_multiplier = simRate/ctrl_channel_rate; // How many times the simulation must step before a new control input needs to be generated

	ros::WallRate spinner(1); // Provisionally allocate the spinner

	ROS_INFO("simClockNode up");
	double spin_rate;
	std::string ctrls_msg;
	switch (timeControls)
	{
		case 0:
			spin_rate = simRate;
			ctrls_msg = "Using default real-time simulation clock";
			break;
		case 1:
			spin_rate = simRate;
			ctrls_msg = "Using controls-triggered simulation clock";
			break;
		case 2:
			spin_rate = max_clock_rate;
			ctrls_msg = "Using free-spinning simulation clock";
			break;
		case 3:
			spin_rate = max_clock_rate;
			ctrls_msg = "Manual simulation clock (pause button)";
			break;
		default:
			ROS_ERROR("Invalid timeControls value!");
			ros::shutdown();
			return 0;
	}
	spinner = ros::WallRate(spin_rate);

	ROS_INFO("%s", ctrls_msg.c_str());
	simClock.clock = simTime;
	pub.publish(simClock);

	int temp_ctrl_counter = frame_multiplier; // Initialize the frame multiplier. Will only be needed if timeControls == 1
	while (ros::ok())
	{
		ros::spinOnce();
		spinner.sleep(); // Sleep to control simulation rate
		// Check if the simulation has been dead for a long time
		if (rate_monitor.get_time_since_last_frame() > dead_time)
		{
			ROS_WARN("Queing a step after dead-time ellapsed");
			step_required = true;
		}
		// Check if a step was required but was not serviced (due to paused simulation)
		if (step_required && !pauseSim)
		{
			step_simulation();
			// If the clock is tied to the control channel
			if (timeControls==1)
			{
				temp_ctrl_counter -= 1;
				if (temp_ctrl_counter == 0)
				{
					step_required = false;
					temp_ctrl_counter = frame_multiplier;
				}
			}
			else
			{
				step_required = false;
			}
		}
	}

	return 0;
}
