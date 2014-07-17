#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <cstdlib>
#include <math.h>

#include "mathutils/mathutils.hpp"
#include "uav_utils/uav_utils.hpp"
//#include "last_letter/uavFwLib.hpp"
#include "last_letter/SimStates.h"
#include "last_letter/SimPWM.h"
#include "last_letter/Environment.h"

#define contactN 7

using namespace std;

/////////
//Classes
/////////

class ModelPlane
{
	public:
	///////////
	//Variables
	last_letter::SimStates states;
	geometry_msgs::WrenchStamped dynamics;
	geometry_msgs::Wrench groundDynamicsVect;
	last_letter::Environment environment;
	geometry_msgs::Vector3 wind;
	ros::Subscriber subInp, subEnv;
	ros::Publisher pubState;
	ros::Publisher pubWrench;
	ros::Time tprev;
	ros::Duration durTemp;
	double dt;
	int initTime;
	double input[4], deltaa_max, deltae_max, deltar_max;
	double contactpoints[contactN*3];
	
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
	
	//Store environmental values
	void getEnvironment(last_letter::Environment environment);
};
