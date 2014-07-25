//Physics model node
//Performs propagation of the dynamic and kinematic model

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <cstdlib>
#include <math.h>

#include "mathutils/mathutils.hpp"
//#include "uav_utils/uav_utils.hpp"
#include "engineLib.hpp"
#include "last_letter/SimStates.h"
#include "last_letter/SimPWM.h"
#include "last_letter/Environment.h"

using namespace std;

/////////
//Classes
/////////

class ModelPlane
{
	public:
	///////////
	//Variables
	last_letter::SimStates states; // main simulation states
	geometry_msgs::WrenchStamped dynamics; // forces and torques message
	geometry_msgs::Wrench groundDynamicsVect; // ground reactions
	last_letter::Environment environment; // environmental component local to the UAV
	ros::Subscriber subInp, subEnv; // ROS subscribers
	ros::Publisher pubState; // ROS publishers
	ros::Publisher pubWrench;
	ros::Time tprev; // previous ROS time holder
	ros::Duration durTemp; // simulation timestep duration
	double dt; // simulation timestep in s
	int initTime;
	double input[4], deltaa_max, deltae_max, deltar_max;
	double* contactPoints, * spp;
	int contactPtsNo;
	
	/////////
	//Members
	engine * powerPlant;
	
	///////////
	//Functions
	
	//Constructor
	ModelPlane (ros::NodeHandle n);
	
	//Destructor
	~ModelPlane ();
	
	//Input callback
	void getInput(last_letter::SimPWM inputMsg);
	
	//Simulation step
	void step(void);
	
	//Differential equation propagation
	void diffEq(void);
	
	//Calculate Forces
	geometry_msgs::Vector3 getForce(last_letter::SimStates states, double inputs[4]);
	
	//Calculate Torques
	geometry_msgs::Vector3 getTorque(last_letter::SimStates states, geometry_msgs::Vector3 forces, double inputs[4]);
	
	//Calculate Ground Forces and Torques
	geometry_msgs::Wrench groundDynamics(geometry_msgs::Quaternion quat);
	
	//Calculate lift coefficient from alpha
	double liftCoeff (double alpha);
	
	//Calculate drag coefficient from alpha
	double dragCoeff (double alpha);
	
	//Read environmental values
	void getEnvironment(last_letter::Environment environment);
};
