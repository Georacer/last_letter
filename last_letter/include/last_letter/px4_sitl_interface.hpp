
// #include <cstdlib>
#include <iostream>
#include <sys/socket.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <netinet/tcp.h> // Weird that this is required, not included in original PX4 code
#include <chrono>
#include <mutex>
#include <thread> //TODO: May not be useful, was originally imported because Gazebo plugins run as threads.
#include <atomic>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include <last_letter/mavlink_interface.hpp>
#include <last_letter_lib/math_utils.hpp>
#include <last_letter_msgs/msg/sim_pwm.hpp>
#include <last_letter_msgs/msg/link_states.hpp>
#include <last_letter_msgs/msg/airdata.hpp>
#include <last_letter_msgs/msg/barometer.hpp>
#include <last_letter_msgs/msg/gnss.hpp>
#include <last_letter_msgs/msg/imu.hpp>
#include <last_letter_msgs/msg/mavlink_hil_state_quaternion.hpp>

namespace lll = last_letter_lib;

//! OR operation for the enumeration and unsigned types that returns the bitmask
static inline uint32_t operator|(SensorSource lhs, SensorSource rhs)
{
    return static_cast<uint32_t>(
        static_cast<std::underlying_type<SensorSource>::type>(lhs) |
        static_cast<std::underlying_type<SensorSource>::type>(rhs));
}

static inline uint32_t operator|(uint32_t lhs, SensorSource rhs)
{
    return static_cast<uint32_t>(
        static_cast<std::underlying_type<SensorSource>::type>(lhs) |
        static_cast<std::underlying_type<SensorSource>::type>(rhs));
}

class Px4Interface : public rclcpp::Node
{

public:
    Px4Interface();
    ~Px4Interface();
    void declare_mavlink_parameters();
    void configure_mavlink_interface();
    void chan_2_signal(const last_letter_msgs::msg::SimPWM::SharedPtr msg);
    void store_states_cb(
    const last_letter_msgs::msg::Airdata::SharedPtr,
    const last_letter_msgs::msg::Barometer::SharedPtr,
    const last_letter_msgs::msg::Imu::SharedPtr,
    const last_letter_msgs::msg::Gnss::SharedPtr,
    const last_letter_msgs::msg::MavlinkHilStateQuaternion::SharedPtr
    );
    std::vector<double> get_actuator_controls();
    void publish_control_inputs();
    void channel_functions();
    double generate_noise(double std_dev);
    void send_sensor_message();
    void send_gps_message();
    void send_ground_truth();
    void send_rc_inputs_message();
    void custom_sigint_handler();

private:
    void convert_controls_from_px4_to_ll(const std::vector<double> inputs);
    void print_link_states_ned();
    void print_link_states_body();

    //Subscribers
    rclcpp::Subscription<last_letter_msgs::msg::SimPWM>::SharedPtr sub_chan;
    // Message filter synchronizer subscribers
	message_filters::Subscriber<last_letter_msgs::msg::Airdata> airdata_sub_;
	message_filters::Subscriber<last_letter_msgs::msg::Barometer> baro_sub_;
	message_filters::Subscriber<last_letter_msgs::msg::Imu> imu_sub_;
	message_filters::Subscriber<last_letter_msgs::msg::Gnss> gnss_sub_;
	message_filters::Subscriber<last_letter_msgs::msg::MavlinkHilStateQuaternion> groundtruth_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<
        last_letter_msgs::msg::Airdata,
        last_letter_msgs::msg::Barometer,
        last_letter_msgs::msg::Imu,
        last_letter_msgs::msg::Gnss,
        last_letter_msgs::msg::MavlinkHilStateQuaternion
    >> msg_synchronizer_;
    // rclcpp::Subscription<last_letter_msgs::msg::LinkStates>::SharedPtr sub_mod_st;

    //Publishers
    rclcpp::Publisher<last_letter_msgs::msg::LinkStates>::SharedPtr pub_state;
    rclcpp::Publisher<last_letter_msgs::msg::SimPWM>::SharedPtr pub_ctrl;

    last_letter_msgs::msg::SimPWM channels_;
    // last_letter_msgs::msg::LinkStates model_states_, model_states_prev_;
    last_letter_msgs::msg::Airdata airdata_reading;
    last_letter_msgs::msg::Barometer baro_reading;
    last_letter_msgs::msg::Imu imu_reading;
    last_letter_msgs::msg::Gnss gnss_reading;
    last_letter_msgs::msg::MavlinkHilStateQuaternion groundtruth_reading;

    Eigen::VectorXf commands;
    Eigen::VectorXf input_signal_vector;

    // Essential variables
    int i;
    int num_motors{8};
    int roll_in_chan{0}, pitch_in_chan{1}, yaw_in_chan{3}, throttle_in_chan{2};
    float roll_input, pitch_input, yaw_input, thrust_input;
    float new_roll_input, new_pitch_input, new_yaw_input, new_thrust_input;

    // variables for PD controller algorithm
    float prev_roll_error, prev_pitch_error, prev_yaw_error, prev_alt_error;
    float altitude, yaw_direction;
    float dt{0.004};

    // MAVLink configuration
    std::unique_ptr<MavlinkInterface> mavlink_interface_;
    float protocol_version_{2.0};
    static const unsigned int n_out_max_{16};

    rclcpp::Time last_time_;
    rclcpp::Time current_time_;
    rclcpp::Time last_actuator_time_{0};
    Eigen::VectorXd input_reference_;
    int input_index_[n_out_max_];
    bool received_first_actuator_{false};
    bool enable_lockstep_{true};
    bool hil_mode_;
    bool hil_state_level_;

    int64_t previous_imu_seq_{0};
    unsigned update_skip_factor_{1};
    std::atomic<bool> got_sig_int_{false};
    Eigen::Vector3d wind_vel_ = Eigen::Vector3d::Zero();
    rclcpp::Time px4_starting_timestamp_;
    rclcpp::Time px4_time_;
    rclcpp::Time px4_time_prev_;

    rclcpp::Time readings_timestamp_;
    last_letter_msgs::msg::SimPWM ctrl_output_;
    Eigen::Quaterniond q_ned_body_;
    Eigen::Vector3d position_ned_, velocity_linear_ned_, velocity_angular_ned_, acceleration_linear_ned_;
};
