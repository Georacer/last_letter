/// Ground reactions interface class
class GroundReaction
{
	public:
	ModelPlane * parentObj; // pointer to parent ModelPlane class
	GroundReaction(ModelPlane *);
	~GroundReaction();
	geometry_msgs::Wrench wrenchGround;
	double inputSteer, inputBrake;
	double steerAngle_max;
	int chanSteer, chanBrake;
	void getInput();
	virtual geometry_msgs::Vector3 getForce()=0;
	virtual geometry_msgs::Vector3 getTorque()=0;
};

#include "noGroundReactions.hpp"

#include "panosContactPoints.hpp"

#include "pointFriction.hpp"
