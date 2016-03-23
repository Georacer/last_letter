// Kinematics equations related classes

// Main generic class
class Kinematics
{
	public:
	ModelPlane * parentObj;
	Kinematics(ModelPlane *);
	~Kinematics();
	geometry_msgs::Vector3 forceInput;
	geometry_msgs::Vector3 torqueInput;
	geometry_msgs::Vector3 posDot;
	geometry_msgs::Vector3 speedDot;
	geometry_msgs::Vector3 rateDot;
	geometry_msgs::Quaternion quatDot;
	geometry_msgs::Vector3 linearAcc; // Temp variable
	geometry_msgs::Vector3 speedAero;

	double mass;
	double J[9], Jinv[9];
	void calcDerivatives();
	Integrator * integrator;
};

// State integrator interface class
// Propagates the state derivatives onto the ModelPlane states
class Integrator
{
	public:
	ModelPlane * parentObj;
	Integrator(ModelPlane *);
	~Integrator();
	virtual void propagation() =0;
};

// Forward Euler integrator class
class ForwardEuler : public Integrator
{
	public:
	ForwardEuler(ModelPlane *);
	void propagation();
};