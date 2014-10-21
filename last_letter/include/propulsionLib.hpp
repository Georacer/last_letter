// Propulsion class related declarations

// Propulsion interface class declaration
class Propulsion
{
	public:
	///////////
	//Variables
	ModelPlane * parentObj; // pointer to parent ModelPlane class
	geometry_msgs::Vector3 CGOffset; // vector from CG to engine coordinates
	double omega; // motor angular speed in rad/s
	geometry_msgs::Wrench wrenchProp;

	///////////
	//Functions	
	Propulsion(ModelPlane *);
	~Propulsion();

	virtual void updateRadPS() =0; // Step the angular speed
	virtual geometry_msgs::Vector3 getForce() =0; // Calculate Forces
	virtual geometry_msgs::Vector3 getTorque() =0; //Calculate Torques
};

// No engine, dummy class
class NoEngine: public Propulsion
{
public:
	NoEngine(ModelPlane *);
	~NoEngine();

	void updateRadPS();
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};

// Electric engine model found in R. Beard's book
class EngBeard: public Propulsion
{
	public:
	///////////
	//Variables
	double s_prop, c_prop, k_motor, k_t_p, k_omega;
	double airspeed, rho, deltat;
	
	///////////
	//Functions
	EngBeard(ModelPlane *);
	~EngBeard();
	
	void updateRadPS(); //Step the angular speed
	geometry_msgs::Vector3 getForce(); //Calculate Forces
	geometry_msgs::Vector3 getTorque(); //Calculate Torques
};

// Piston engine
class PistonEng : public Propulsion
{
public:
	////////////
	// Variables
	double omegaMin, omegaMax;
	double deltat, propDiam, engInertia;

	//////////
	// Members
	Polynomial * engPowerPoly;
	Polynomial * npPoly;
	Polynomial * propPowerPoly;
	// Polynomial1D * npPoly;
	// Polynomial1D * propPowerPoly;
	// Polynomial2D * engPowerPoly;

	////////////
	// Functions
	PistonEng(ModelPlane *);
	~PistonEng();

	void updateRadPS();
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};