#include "coreLib.hpp"



using namespace std;

// #ifndef MODELPLANE_DEF
// #define MODELPLANE_DEF

// class ModelPlane;
// class Kinematics;
// class Dynamics;
// class Integrator;
// class Aerodynamics;
// class Gravity;
// // class Propulsion;
// class GroundReaction;
// class Airdata;

// 	class ModelPlane
// 	{
// 		public:
// 		///////////
// 		//Variables
// 		last_letter::SimStates states; // main simulation states
// 		last_letter::Environment environment; // environmental component local to the UAV
// 		ros::Subscriber subInp, subEnv; // ROS subscribers
// 		ros::Publisher pubState, pubForce, pubTorque; // ROS publishers
// 		ros::Time tprev; // previous ROS time holder
// 		double dt; // simulation timestep in s
// 		int initTime; // first simulation loop flag
// 		double input[4], deltaa_max, deltae_max, deltar_max;
		
// 		/////////
// 		//Members
// 		Kinematics kinematics;
// 		Dynamics dynamics;
// 		Airdata airdata;
		
// 		///////////
// 		//Methods
		
// 		//Constructor
// 		ModelPlane (ros::NodeHandle n);
		
// 		//Reset
// 		void init();
		
// 		//Destructor
// 		~ModelPlane ();
		
// 		//Input callback
// 		void getInput(last_letter::SimPWM inputMsg);
		
// 		//Simulation step
// 		void step(void);
		
// 		//Read environmental values callback
// 		void getEnvironment(last_letter::Environment environment);
// 	};
// #endif

/////////
//Classes
/////////


//////////////////////////
// Define Propulsion class
//////////////////////////




//class engBeardAbs
//{
//	public:
//	private:
//};

//class engPistonICE
//{
//	public:
//	//Calculate Forces
//	geometry_msgs::Vector3 getForce(last_letter::SimStates states, double inputs[4]);
//	
//	//Calculate Torques
//	geometry_msgs::Vector3 getTorque(last_letter::SimStates states, geometry_msgs::Vector3 forces, double inputs[4]);
//	private:
//};


Propulsion::Propulsion(ModelPlane * parent)
{
	parentObj = parent;
}

Propulsion::~Propulsion()
{
	delete parentObj;
}

EngBeard::EngBeard(ModelPlane * parent):Propulsion(parent)
{
	std::cout << "reading parameters for new Beard engine" << std::endl;
	omega = 0;
	if(!ros::param::getCached("motor/s_prop", s_prop)) {ROS_FATAL("Invalid parameters for -s_prop- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/c_prop", c_prop)) {ROS_FATAL("Invalid parameters for -c_prop- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_motor", k_motor)) {ROS_FATAL("Invalid parameters for -k_motor- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_t_p", k_t_p)) {ROS_FATAL("Invalid parameters for -k_t_p- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("motor/k_omega", k_omega)) {ROS_FATAL("Invalid parameters for -k_omega- in param server!"); ros::shutdown();}
}

EngBeard::~EngBeard()
{
}

void EngBeard::updateRPS()
{
	rho = parentObj->environment.density;
	deltat = parentObj->input[2];
	airspeed = parentObj->airdata.airspeed;
	omega = 1 / (0.5 + parentObj->dt) * (0.5 * omega + parentObj->dt * deltat * k_motor);
}

geometry_msgs::Vector3 EngBeard::getForce()
{
	wrenchProp.force.x = 1.0/2.0*rho*s_prop*c_prop*(pow(omega,2)-pow(airspeed,2));
	wrenchProp.force.y = 0;
	wrenchProp.force.z = 0;

	return wrenchProp.force;
}

geometry_msgs::Vector3 EngBeard::getTorque()
{
	wrenchProp.torque.x = -k_t_p*pow(k_omega*omega/k_motor,2);
//		double lm = -k_t_p*pow(k_omega*deltat,2);
	wrenchProp.torque.y = 0;
	wrenchProp.torque.z = 0;

	return wrenchProp.torque;
}
