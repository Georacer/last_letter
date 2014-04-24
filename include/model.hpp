#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <cstdlib>
#include <math.h>

#include "mathutils/utls.hpp"
#include "last_letter/inputs.h"

using namespace std;

/////////
//Classes
/////////

class ModelPlane
{
	public:
	///////////
	//Variables
	nav_msgs::Odometry kinematics;
	geometry_msgs::WrenchStamped dynamics;
	ros::Subscriber subInp;
	ros::Publisher pubState;
	ros::Publisher pubWrench;
	ros::Time tprev;
	ros::Duration durTemp;
	double dt;
	double input[4];
	
	
	///////////
	//Functions
	
	//Constructor
	ModelPlane (ros::NodeHandle n);
	
	//Destructor
	~ModelPlane ();
	
	//Input callback
	void getInput(last_letter::inputs inputMsg);
	
	//Simulation step
	void step(void);
	
	//Differential equation propagation
	void diffEq(void);
	
	//Calculate Forces
	geometry_msgs::Vector3 getForce(nav_msgs::Odometry states, double inputs[4]);
	
	//Calculate Torques
	geometry_msgs::Vector3 getTorque(nav_msgs::Odometry states, double inputs[4]);
	
	//Extract air data (airspeed, alpha, beta)
	geometry_msgs::Vector3 getAirData (geometry_msgs::Vector3 speeds);				
	
	//Calculate lift coefficient from alpha
	double liftCoeff (double alpha);
	
	//Calculate drag coefficient from alpha
	double dragCoeff (double alpha);
	
};
	
			
