// ROS wrapper for UavModel class
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Iir.h>
// #include <tf/transform_broadcaster.hpp>
#include <std_srvs/srv/empty.hpp>
#include <gazebo_msgs/msg/link_states.hpp>

// #include <geometry_msgs/msg/vector3.hpp>
// #include <geometry_msgs/msg/vector3_stamped.hpp>
// #include <geometry_msgs/msg/quaternion.hpp>
// #include <geometry_msgs/msg/quaternion_stamped.hpp>

// #include <last_letter_msgs/msg/sim_states.hpp>
// #include <last_letter_msgs/msg/sim_wrenches.hpp>
#include <last_letter_msgs/msg/sim_pwm.hpp>
// #include <last_letter_msgs/msg/Environment.hpp>
#include <last_letter_msgs/msg/parameter.hpp>
#include <last_letter_msgs/msg/sim_commands.hpp>
#include <last_letter_msgs/msg/link_states.hpp>
#include <last_letter_msgs/msg/airdata.hpp>
#include <last_letter_msgs/msg/barometer.hpp>
#include <last_letter_msgs/msg/gnss.hpp>
#include <last_letter_msgs/msg/imu.hpp>
#include <last_letter_msgs/msg/mavlink_hil_state_quaternion.hpp>

#include <last_letter_lib/uav_model.hpp>

namespace lll = last_letter_lib;

// Top ModelPlane object class
class UavModelNode : public rclcpp::Node
{
public:
	///////////
	//Variables
	lll::UavModel *uavModel;
	std::string uavName;
	LinkStateMap_t linkStates_;
	last_letter_msgs::msg::SimCommands gazebo_commands;
	last_letter_msgs::msg::SimPWM input; // PWM input to the model
	rclcpp::Subscription<last_letter_msgs::msg::LinkStates>::SharedPtr subState;
	rclcpp::Subscription<last_letter_msgs::msg::SimPWM>::SharedPtr subInp;
	rclcpp::Subscription<last_letter_msgs::msg::Parameter>::SharedPtr subParam; // ROS subscribers

	rclcpp::Publisher<last_letter_msgs::msg::SimCommands>::SharedPtr gazebo_pub_;
	rclcpp::Publisher<last_letter_msgs::msg::Airdata>::SharedPtr airdata_pub_;
	rclcpp::Publisher<last_letter_msgs::msg::Barometer>::SharedPtr baro_pub_;
	rclcpp::Publisher<last_letter_msgs::msg::Imu>::SharedPtr imu_pub_;
	rclcpp::Publisher<last_letter_msgs::msg::Gnss>::SharedPtr gnss_pub_;
	rclcpp::Publisher<last_letter_msgs::msg::MavlinkHilStateQuaternion>::SharedPtr groundtruth_pub_;

	rclcpp::Time tprev; // previous ROS time holder
	int initTime;		// first simulation loop flag
	int chanReset;
	// bool tfBroadcastEnable = true;
	// tf::TransformBroadcaster broadcaster;
	bool _new_parameters = false; // Whether new parameters have been received

	///////////
	//Methods

	// Constructor
	UavModelNode();

	// Initialize ModelPlane object
	void init();

	// Destructor
	~UavModelNode();

	// Input callback
	/**
	 * getInput Read PWM input to the model and store its normalized values
	 * @param inputMsg Direct servo control commands
	 */
	void getInput(const last_letter_msgs::msg::SimPWM::SharedPtr inputMsg);

	void getState_cb(const last_letter_msgs::msg::LinkStates::SharedPtr linkStates);

	void convertLinkStates(const last_letter_msgs::msg::LinkStates, LinkStateMap_t &);

	void publishSensors();

	last_letter_msgs::msg::Airdata buildAirdataMsg(last_letter_lib::Sensor * );
	last_letter_msgs::msg::Barometer buildBaroMsg(last_letter_lib::Sensor * );
	last_letter_msgs::msg::Imu buildImuMsg(last_letter_lib::Sensor * );
	last_letter_msgs::msg::Gnss buildGnssMsg(last_letter_lib::Sensor * );
	last_letter_msgs::msg::MavlinkHilStateQuaternion buildGroundtruthMsg(last_letter_lib::Sensor * );

	// Parameter callback
	void getParameters(const last_letter_msgs::msg::Parameter::SharedPtr paramMsg);

	// Perform simulation step
	void step();

	void pauseLogic(const uint16_t pwm);

	last_letter_msgs::msg::SimCommands buildGazeboCommands();

	// void broadcastTransforms();

	// // Read environmental values callback
	// void getEnvironment(last_letter_msgs::Environment environment);

private:
	rclcpp::TimerBase::SharedPtr timer_ptr_;
	rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_client_;
	rclcpp::Client<std_srvs::srv::Empty>::SharedPtr unpause_client_;
	bool pausedState_{false};
	bool pauseButtonPressed_{false};
	void timerCallback();

	static const int filter_order_{6};
	const float f_sample_ = 250;
	const float f_cut_ = 5;
	Iir::Butterworth::LowPass<filter_order_> filt_ax_, filt_ay_, filt_az_;
};

last_letter_msgs::msg::CommandPayload buildPayloadWrench(Wrench_t);
// void convertStates(const SimState_t, last_letter_msgs::SimStates &);
// void convertStatesDerivatives(const Derivatives_t, last_letter_msgs::SimStates &);
// // void convertEnvironment(const Environment_t, last_letter_msgs::Environment &);
// void convertTfVector3(const Eigen::Vector3d, tf::Vector3 &);
// void convertTfQuaternion(const Eigen::Quaterniond, tf::Quaternion &);
// void convertRosVector3(const Eigen::Vector3d, geometry_msgs::Vector3 &);
// void convertRosQuaternion(const Eigen::Quaterniond, geometry_msgs::Quaternion &);
