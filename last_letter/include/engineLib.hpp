// Library for engine models

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Wrench.h"
#include <cstdlib>
#include <math.h>

#include "last_letter/SimStates.h"
#include "last_letter/Environment.h"
#include "uav_utils/uav_utils.hpp"

using namespace std;

/////////
//Classes
/////////


//////////////////////////
// Define Propulsion class
//////////////////////////


class Propulsion
{
	public:
	//Variables
	ModelPlane * parentObj;
	double omega; //motor angular speed in rad/s
	geometry_msgs::Wrench wrenchProp;
	
	///////////
	//Functions
	
//	//Constructor
	Propulsion(ModelPlane *);
//	
//	//Destructor
	~Propulsion();
	
	//Step the angular speed
	virtual void updateRPS() =0;
	
	//Calculate Forces
	virtual geometry_msgs::Vector3 getForce() =0;
	
	//Calculate Torques
	virtual geometry_msgs::Vector3 getTorque() =0;
};

class EngBeard: public Propulsion
{
	public:
	///////////
	//Variables
	double s_prop, c_prop, k_motor, k_t_p, k_omega;
	double airspeed, rho, deltat;
	
	///////////
	//Functions
	
	//Constructor
	EngBeard(ModelPlane *);
	
	//Destructor
	~EngBeard();
	
	//Step the angular speed
	void updateRPS();
	
	//Calculate Forces
	geometry_msgs::Vector3 getForce();
	
	//Calculate Torques
	geometry_msgs::Vector3 getTorque();
	
	private:
};

//class engBeardAbs
//{
//	public:
//	private:
//};

//class engPistonICE
//{
//	public:
//	//Calculate Forces
//	geometry_msgs::Vector3 getForce(last_letter::SimStates states, double inputs[4]);
//	
//	//Calculate Torques
//	geometry_msgs::Vector3 getTorque(last_letter::SimStates states, geometry_msgs::Vector3 forces, double inputs[4]);
//	private:
//};
