//Physics model node
//Performs propagation of the dynamic and kinematic model

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <cstdlib>
#include <math.h>

#include "mathutils/mathutils.hpp"
//#include "uav_utils/uav_utils.hpp"
//#include "engineLib.hpp"
#include "last_letter/SimStates.h"
#include "last_letter/SimPWM.h"
#include "last_letter/Environment.h"

using namespace std;

/////////
//Classes
/////////

class ModelPlane;
class Kinematics;
class Dynamics;
class Integrator;
class Aerodynamics;
class Gravity;
class Propulsion;
class GroundReaction;
class Airdata;

// Top class for UAV object : First pass


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

// Dynamic model aggregator class : First pass
class Dynamics
{
	public:
	ModelPlane * parentObj;
	Dynamics(ModelPlane *);
	~Dynamics();
	Aerodynamics * aerodynamics;
	Gravity * gravity;
	Propulsion * propulsion;
	GroundReaction * groundReaction;
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};

// Aerodynamics interface class
class Aerodynamics
{
	public:
	ModelPlane * parentObj;
	Aerodynamics(ModelPlane *);
	~Aerodynamics();
	geometry_msgs::Wrench wrenchAero;
	virtual geometry_msgs::Vector3 getForce() = 0;
	virtual geometry_msgs::Vector3 getTorque() = 0;
};

class StdLinearAero : public Aerodynamics
{
	public:
	StdLinearAero(ModelPlane *);
	~StdLinearAero();
	double rho,g,m;
	double c_lift_q,c_lift_deltae,c_drag_q,c_drag_deltae;
	double c,b,s;
	double c_y_0,c_y_b,c_y_p,c_y_r,c_y_deltaa,c_y_deltar;
	double c_l_0, c_l_b, c_l_p, c_l_r, c_l_deltaa, c_l_deltar;
	double c_m_0, c_m_a, c_m_q, c_m_deltae;
	double c_n_0, c_n_b, c_n_p, c_n_r, c_n_deltaa, c_n_deltar;
	double M, alpha0, c_lift_0, c_lift_a;
	double c_drag_p, oswald, AR;
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
	//Calculate lift coefficient from alpha
	double liftCoeff(double);
	//Calculate drag coefficient from alpha
	double dragCoeff(double);
};

// Gravity class : First pass
class Gravity
{
	public:
	ModelPlane * parentObj;
	Gravity(ModelPlane *);
	~Gravity();
	double col_x, col_y, col_z;
	double g;
	geometry_msgs::Wrench wrenchGrav;
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};

// Ground reactions interface class : Blank
class GroundReaction
{
	public:
	ModelPlane * parentObj;
	GroundReaction(ModelPlane *);
	~GroundReaction();
	geometry_msgs::Wrench wrenchGround;
	virtual geometry_msgs::Vector3 getForce()=0;
	virtual geometry_msgs::Vector3 getTorque()=0;
};

// Panos ground implementation : First pass
class PanosContactPoints : public GroundReaction
{
	public:
	PanosContactPoints(ModelPlane *);
	~PanosContactPoints();
	double* contactPoints, * spp;
	double * cpi_up, * cpi_down, * spd, * pointCoords;
	double uavpos[3], normVe;
	double kspring, mspring, kfriction, len;
	bool contact;
	int contactPtsNo;
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};

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

//////////////////////////////////////////////////////////////////////////
//////////////////////////
// Define Propulsion class
//////////////////////////


class Propulsion
{
	public:
	//Variables
	ModelPlane * parentObj;
	double omega; //motor angular speed in rad/s
	geometry_msgs::Wrench wrenchProp;
	
	///////////
	//Functions
	
//	//Constructor
	Propulsion(ModelPlane *);
//	
//	//Destructor
	~Propulsion();
	
	//Step the angular speed
	virtual void updateRPS() =0;
	
	//Calculate Forces
	virtual geometry_msgs::Vector3 getForce() =0;
	
	//Calculate Torques
	virtual geometry_msgs::Vector3 getTorque() =0;
};

class EngBeard: public Propulsion
{
	public:
	///////////
	//Variables
	double s_prop, c_prop, k_motor, k_t_p, k_omega;
	double airspeed, rho, deltat;
	
	///////////
	//Functions
	
	//Constructor
	EngBeard(ModelPlane *);
	
	//Destructor
	~EngBeard();
	
	//Step the angular speed
	void updateRPS();
	
	//Calculate Forces
	geometry_msgs::Vector3 getForce();
	
	//Calculate Torques
	geometry_msgs::Vector3 getTorque();
	
	private:
};
//////////////////////////////////////////////////////////////////////////////////

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
	ros::Publisher pubState; // ROS publishers
//	ros::Publisher pubWrench;
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
