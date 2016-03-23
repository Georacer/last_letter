// Core class declarations

// Air data class declaration
class Airdata
{
	public:
	ModelPlane * parentObj;
	Airdata(ModelPlane *);
	~Airdata();
	double u_r, v_r, w_r; // relative wind vector elements
	double airspeed; // relative airspeed
	double alpha; // angle of attach
	double beta; // angle of sideslip
	void calcAirData();
};

// Factory Class for parametric class initializations
class Factory
{
	public:
	Integrator * buildIntegrator(ModelPlane *);
	Aerodynamics * buildAerodynamics(ModelPlane *, int);
	Propulsion * buildPropulsion(ModelPlane *, int);
	GroundReaction * buildGroundReaction(ModelPlane *);
	Polynomial * buildPolynomial(char * parameter);
};

// Top ModelPlane object class
class ModelPlane
{
	public:
	///////////
	//Variables
	last_letter_msgs::SimStates states; // main simulation states
	last_letter_msgs::SimPWM input; // PWM input to the model
	last_letter_msgs::Environment environment; // environmental component local to the UAV
	ros::Subscriber subInp, subEnv; // ROS subscribers
	ros::Publisher pubState, pubForce, pubTorque, pubLinAcc; // ROS publishers
	ros::Time tprev; // previous ROS time holder
	double dt; // simulation timestep in s
	int initTime; // first simulation loop flag
	int chanReset;

	/////////
	//Members
	Kinematics kinematics;
	Dynamics dynamics;
	Airdata airdata;

	///////////
	//Methods

	// Constructor
	ModelPlane (ros::NodeHandle n);

	// Initialize ModelPlane object
	void init();

	// Destructor
	~ModelPlane ();

	// Input callback
	/**
	 * getInput Read PWM input to the model and store its normalized values
	 * @param inputMsg Direct servo control commands
	 */
	void getInput(last_letter_msgs::SimPWM inputMsg);

	// Perform simulation step
	void step(void);

	// Read environmental values callback
	void getEnvironment(last_letter_msgs::Environment environment);
};
