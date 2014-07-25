// Library for engine models

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <cstdlib>
#include <math.h>

#include "last_letter/SimStates.h"
#include "last_letter/Environment.h"
#include "uav_utils/uav_utils.hpp"

using namespace std;

/////////
//Classes
/////////

class engine
{
	public:
	//Variables
	double omega; //motor angular speed in rad/s
	
	///////////
	//Functions
	
//	//Constructor
	engine();
//	
//	//Destructor
	~engine();
	
	//Step the angular speed
	virtual void step(last_letter::SimStates states, last_letter::Environment environment, double input[4], double dt) =0;
//	virtual void step(double, double);
	
	//Calculate Forces
	virtual geometry_msgs::Vector3 getForce() =0;
//	virtual geometry_msgs::Vector3 getForce(double, double);
	
	//Calculate Torques
	virtual geometry_msgs::Vector3 getTorque() =0;
};

class engBeard: public engine
{
	public:
	///////////
	//Variables
	double s_prop, c_prop, k_motor, k_t_p, k_omega;
	double airspeed, rho, dt, deltat;
	
	///////////
	//Functions
	
	//Constructor
	engBeard();
	
	//Destructor
	~engBeard();
	
	//Step the angular speed
	void step(last_letter::SimStates states, last_letter::Environment environment, double input[4], double dt);
	
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
