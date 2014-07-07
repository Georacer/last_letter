#include "ros/ros.h"
#include <cstdlib>
#include <math.h>
#include "geometry_msgs/Vector3.h"

#include "mathutils/mathutils.hpp"
#include "uav_utils/uav_utils.hpp"
#include "last_letter/SimStates.h"
#include "last_letter/SimPWM.h"

class BMcLAttitudeController
{
	public:
	///////////
	//Variables
	last_letter::SimStates states;
	geometry_msgs::Vector3 euler;
	double input[4];
	double output[4];
	ros::Time tprev;
	ros::Subscriber subInp, subState;
	ros::Publisher pubCtrl;
	double P, I, D, satU, satL, Ts, N;
	///////////
	//Functions
	uav_utils::PID roll2Aileron, yaw2Roll;
	
	private:
	///////////
	//Variables

	///////////
	//Functions

}
