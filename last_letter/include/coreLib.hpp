// Air data class : First pass
class Airdata
{
	public:
	ModelPlane * parentObj;
	Airdata(ModelPlane *);
	~Airdata();
	double airspeed;
	double alpha;
	double beta;
	void calcAirData();
};

// Factory Class for parametric class initializations : First pass
class Factory
{
	public:
	Integrator * buildIntegrator(ModelPlane *);
	Aerodynamics * buildAerodynamics(ModelPlane *);
	Propulsion * buildPropulsion(ModelPlane *);
	GroundReaction * buildGroundReaction(ModelPlane *);
};

class ModelPlane
{
	public:
	///////////
	//Variables
	last_letter::SimStates states; // main simulation states
	last_letter::Environment environment; // environmental component local to the UAV
	ros::Subscriber subInp, subEnv; // ROS subscribers
	ros::Publisher pubState, pubForce, pubTorque; // ROS publishers
	ros::Time tprev; // previous ROS time holder
	double dt; // simulation timestep in s
	int initTime; // first simulation loop flag
	double input[4], deltaa_max, deltae_max, deltar_max;
	
	/////////
	//Members
	Kinematics kinematics;
	Dynamics dynamics;
	Airdata airdata;
	
	///////////
	//Methods
	
	//Constructor
	ModelPlane (ros::NodeHandle n);
	
	//Reset
	void init();
	
	//Destructor
	~ModelPlane ();
	
	//Input callback
	void getInput(last_letter::SimPWM inputMsg);
	
	//Simulation step
	void step(void);
	
	//Read environmental values callback
	void getEnvironment(last_letter::Environment environment);
};