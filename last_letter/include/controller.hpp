#include "ros/ros.h"
#include <cstdlib>
#include <math.h>
#include "geometry_msgs/Vector3.h"

#include "mathutils/mathutils.hpp"
#include "uav_utils/uav_utils.hpp"
#include "last_letter/SimStates.h"
#include "last_letter/SimPWM.h"
#include "last_letter/Environment.h"
#include "last_letter/RefCommands.h"

class BMcLAttitudeController
{
	public:
	///////////
	//Variables
	last_letter::SimStates states;
	last_letter::Environment environment;
	last_letter::RefCommands refCommands;
	geometry_msgs::Vector3 euler, airdata;
	double input[10];
	double output[10];
	ros::Time tprev;
	ros::Subscriber subInp, subState, subEnv, subRef;
	ros::Publisher pubCtrl;
	double P, I, D, satU, satL, trim, Ts, Tau, altThresh, altDes;
	/////////
	//Members
	PID * roll2Aileron;
	PID * yaw2Roll;
	PID * beta2Rudder;
	PID * pitch2Elevator;
	APID * alt2Pitch;
	PID * airspd2Pitch;
	PID * airspd2Throt;
	discrTF * pitchSmoother;
	///////////
	//Functions
	void step();
	void getInput(last_letter::SimPWM inputMsg);
	void getStates(last_letter::SimStates inpStates);
	void getReference(last_letter::RefCommands refInp);
	void writePWM(double *output);
	void getEnvironment(last_letter::Environment envUpdate);
	double aileronControl();
	double rudderControl();
	double elevatorControl();
	double throttleControl();
	
	//Constructor
	BMcLAttitudeController(ros::NodeHandle n);
	//Destructor
	~BMcLAttitudeController();
	
	private:
	///////////
	//Variables

	///////////
	//Functions

};
