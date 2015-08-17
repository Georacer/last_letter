////////////////////////////////////////
// Define GroundReaction interface class
////////////////////////////////////////

// Class constructor
GroundReaction::GroundReaction(ModelPlane * parent)
{
	parentObj = parent;
	if(!ros::param::getCached("airframe/chanSteer", chanSteer)) {ROS_INFO("No STEER channel selected"); chanSteer=-1;}
	if(!ros::param::getCached("airframe/chanBrake", chanBrake)) {ROS_INFO("No BRAKE channel selected"); chanBrake=-1;}
	if(!ros::param::getCached("airframe/steerAngle_max", steerAngle_max)) {ROS_INFO("No STEERANGLE_MAX value selected"); steerAngle_max=0.0;}

	inputSteer = 0.0;
	inputBrake = 0.0;
}

// Class destructor
GroundReaction::~GroundReaction()
{
	delete parentObj;
}

// Store the steering and brake control inputs
void GroundReaction::getInput()
{
	ros::param::getCached("airframe/steerAngle_max", steerAngle_max);
	if (chanSteer>-1) {inputSteer = steerAngle_max * (double)(parentObj->input.value[chanSteer]-1500)/500;}
	if (chanBrake>-1) {inputBrake = (double)(parentObj->input.value[chanBrake]-1000)/1000;}
}


#include "noGroundReactions.cpp"

#include "panosContactPoints.cpp"

#include "pointFriction.cpp"
