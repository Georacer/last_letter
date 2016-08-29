///////////////////////////////////////////
// Propulsion class related declarations //
///////////////////////////////////////////

#include <tf/transform_broadcaster.h>

#include "gazebo_msgs/ModelState.h"

#include <last_letter_msgs/ElectricEng.h>

////////////////////////////////////////////
// Propulsion interface class declaration //
////////////////////////////////////////////
class Propulsion
{
	public:
	///////////
	//Variables
	ModelPlane * parentObj; // pointer to parent ModelPlane class
	int id; // The current motor ID
	geometry_msgs::Vector3 CGOffset; // vector from CG to engine coordinates
	geometry_msgs::Vector3 mountOrientation;
	geometry_msgs::Vector3 relativeWind; // relative wind vector in the propeller frame
	double inputMotor, inputGimbal; // control input (0-1)
	double gimbalAngle_max;
	int chanMotor, chanGimbal;
	double omega; // motor angular speed in rad/s
	double rotationDir; // motor direction of rotation
	double theta; // propeller angle in rads
	double normalWind; // scalar wind normal to propeller disc
	geometry_msgs::Wrench wrenchProp;

	tf::TransformBroadcaster broadcaster; // Transformations broadcaster object
	tf::Transform body_to_mount, mount_to_gimbal, gimbal_to_prop, body_to_prop; // Transformations in the propeller assembly for visual rendering
	tf::Transform body_to_mount_rot, mount_to_gimbal_rot, gimbal_to_prop_rot, body_to_prop_rot; // Transformations in the propeller assembly for force and moment rotation

	///////////
	//Functions
	Propulsion(ModelPlane *, int);
	~Propulsion();

	void getInput(); // store control input
	void stepEngine(); // engine physics step, container for the generic class
	void rotateWind(); // convert the wind to the propeller axes
	virtual void updateRadPS() =0; // Step the angular speed
	void rotateProp(); // Update the propeller angle
	void rotateForce(); // convert the resulting force to the body axes
	void rotateTorque(); // convert the resulting torque to the body axes
	virtual void getForce() =0; // Calculate Forces
	virtual void getTorque() =0; //Calculate Torques
};

#include "noEngine.hpp"

#include "beardEngine.hpp"

#include "pistonEngine.hpp"

#include "electricEngine.hpp"
