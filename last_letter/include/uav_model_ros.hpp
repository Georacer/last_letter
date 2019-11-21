// ROS wrapper for UavModel class
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <last_letter_msgs/SimStates.h>
#include <last_letter_msgs/SimWrenches.h>
#include <last_letter_msgs/SimPWM.h>
#include <last_letter_msgs/Environment.h>
#include <last_letter_msgs/Parameter.h>

#include "uav_model.hpp"

// Top ModelPlane object class
class UavModelWrapper
{
	public:
	///////////
	//Variables
    UavModel * uavModel;
	last_letter_msgs::SimStates states, states_dot; // main simulation states and their time derivatives
	last_letter_msgs::SimPWM input; // PWM input to the model
	last_letter_msgs::Environment environment; // environmental component local to the UAV
	last_letter_msgs::SimWrenches wrenchInput;
    geometry_msgs::Vector3Stamped linearAcc;
	ros::Subscriber subInp, subParam; // ROS subscribers
	ros::Publisher pubState, pubStateDot, pubEnv, pubWrench, pubLinAcc; // ROS publishers
	ros::Time tprev; // previous ROS time holder
	double dt; // simulation timestep in s
	int initTime; // first simulation loop flag
	int chanReset;
    bool tfBroadcastEnable = true;
    tf::TransformBroadcaster broadcaster;
	bool _new_parameters = false; // Whether new parameters have been received

	///////////
	//Methods

	// Constructor
	UavModelWrapper(ros::NodeHandle n);

	// Initialize ModelPlane object
	void init();

	// Destructor
	~UavModelWrapper ();

	// Input callback
	/**
	 * getInput Read PWM input to the model and store its normalized values
	 * @param inputMsg Direct servo control commands
	 */
	void getInput(last_letter_msgs::SimPWM inputMsg);

	// Parameter callback
	void getParameters(last_letter_msgs::Parameter paramMsg);

	// Perform simulation step
	void step(void);

    void broadcastTransforms();

	// // Read environmental values callback
	// void getEnvironment(last_letter_msgs::Environment environment);
};

void convertStates(const SimState_t, last_letter_msgs::SimStates &);
void convertStatesDerivatives(const Derivatives_t, last_letter_msgs::SimStates &);
void convertEnvironment(const Environment_t, last_letter_msgs::Environment &);
void convertTfVector3(const Eigen::Vector3d, tf::Vector3 &);
void convertTfQuaternion(const Eigen::Quaterniond, tf::Quaternion &);
void convertRosVector3(const Eigen::Vector3d, geometry_msgs::Vector3 &);
void convertRosQuaternion(const Eigen::Quaterniond, geometry_msgs::Quaternion &);