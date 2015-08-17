// Dynamic model aggregator class

// Container class
class Dynamics
{
	public:
	ModelPlane * parentObj; // pointer to ModelPlane parent class
	geometry_msgs::Vector3 force, forceGrav, forceAero, forceProp, forceGround;
	geometry_msgs::Vector3 torque, torqueGrav, torqueAero, torqueProp, torqueGround;
	Dynamics(ModelPlane *);
	~Dynamics();
	Aerodynamics * aerodynamics;
	Gravity * gravity;
	int nMotors; // number of motors mounted on the aircraft
	Propulsion ** propulsion;
	GroundReaction * groundReaction;
	void getInput(); // store and convert new input values
	void calcWrench(); // Calculate the forces and torques for each wrench source
	geometry_msgs::Vector3 getForce(); // Access class members and gather resulting forces
	geometry_msgs::Vector3 getTorque(); // Access class members and gather resulting torques
};