// Dynamic model aggregator class

// Container class
class Dynamics
{
	public:
	ModelPlane * parentObj; // pointer to ModelPlane parent class
	Dynamics(ModelPlane *);
	~Dynamics();
	Aerodynamics * aerodynamics;
	Gravity * gravity;
	int nMotors;
	Propulsion ** propulsion;
	GroundReaction * groundReaction;
	geometry_msgs::Vector3 getForce(); // Access class members and gather resulting forces
	geometry_msgs::Vector3 getTorque(); // Access class members and gather resulting torques
};