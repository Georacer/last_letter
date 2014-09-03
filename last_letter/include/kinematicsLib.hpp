// Kinematic equations class : First pass
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
	
	double mass;
	double J[9], Jinv[9];
	void calcDerivatives();
	Integrator * integrator;
};

// State integrator interface class
class Integrator
{
	public:
	ModelPlane * parentObj;
	Integrator(ModelPlane *);
	~Integrator();
	virtual void propagation() =0;
};

// Forward Euler integrator class : First pass
class ForwardEuler : public Integrator
{
	public:
	ForwardEuler(ModelPlane *);
	void propagation();
};