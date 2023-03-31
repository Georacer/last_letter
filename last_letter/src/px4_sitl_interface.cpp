/**
 * @file px4_sitl_interface.cpp
 * @author George Zogopoulos (tailwhipx4@gmail.com)
 * @brief Px4Interface class definition which bridge with PX4 SITL
 * Based on https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_mavlink_interface.cpp
 * @date 2020-11-05
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <signal.h>
#include <random>

#include <last_letter_lib/uav_utils.hpp>

#include <last_letter/px4_sitl_interface.hpp>

using Eigen::Quaterniond;
using Eigen::Vector3d;

/**
 * @brief Construct a new Px4Interface:: Px4Interface object
 * @details This rclcpp::Node subclass interfaces the last_letter model with
 * the PX4 SITL. It sends aircraft controls from PX4 to the simulated model
 * and sends telemetry and user-controls from last_letter to PX4
 *
 */
Px4Interface::Px4Interface() : Node("px4_interface_node")
{

    // Init Subscribers
    sub_chan = this->create_subscription<last_letter_msgs::msg::SimPWM>(
        "rawPWM",
        1,
        std::bind(&Px4Interface::chan_2_signal, this, std::placeholders::_1));
    airdata_sub_.subscribe(this, "/airdata");
    baro_sub_.subscribe(this, "/barometer");
    imu_sub_.subscribe(this, "/imu");
    gnss_sub_.subscribe(this, "/gnss");
    groundtruth_sub_.subscribe(this, "/groundtruth");

    msg_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<
        last_letter_msgs::msg::Airdata,
        last_letter_msgs::msg::Barometer,
        last_letter_msgs::msg::Imu,
        last_letter_msgs::msg::Gnss,
        last_letter_msgs::msg::MavlinkHilStateQuaternion>>(
        airdata_sub_,
        baro_sub_,
        imu_sub_,
        gnss_sub_,
        groundtruth_sub_,
        10);
    // msg_synchronizer.registerCallback(std::bind(
    //     &Px4Interface::store_states_cb,
    //     this,
    //     std::placeholders::_1,
    //     std::placeholders::_2,
    //     std::placeholders::_3,
    //     std::placeholders::_4,
    //     std::placeholders::_5
    // ));
    msg_synchronizer_->registerCallback(&Px4Interface::store_states_cb, this);

    // Init publishers
    // pub_state = create_publisher<last_letter_msgs::msg::LinkStates>("processed_model_state", 1);
    pub_ctrl = create_publisher<last_letter_msgs::msg::SimPWM>("/ctrlPWM", 1);

    // Initize MAVLinkInterface object...
    mavlink_interface_ = std::make_unique<MavlinkInterface>();
    // configure it...
    declare_mavlink_parameters();
    configure_mavlink_interface();
    // and start it.
    mavlink_interface_->Load();

    // Capture sigint and call the appropriate function in mavlink_interface_
    // This is needed to close the network ports held by mavlink_interfce_ properly
    std::function<void(void)> f = std::bind(&Px4Interface::custom_sigint_handler, this);
    rclcpp::on_shutdown(f);

    px4_starting_timestamp_ = readings_timestamp_;

    RCLCPP_INFO(get_logger(), "PX4-SITL interface spawned");
}

/**
 * @brief Destroy the Px4Interface:: Px4Interface object
 *
 */
Px4Interface::~Px4Interface()
{
    RCLCPP_WARN(get_logger(), "Node closing...");
    got_sig_int_ = true; // Set flag
    mavlink_interface_->close();
    rclcpp::shutdown();
}

/**
 * @brief Declare the ROS parameters available for this node, related to the
 * mavlink_interface_.
 *
 */
void Px4Interface::declare_mavlink_parameters()
{
    declare_parameter<bool>("hil_mode", false);
    declare_parameter<bool>("hil_state_level", false);
    declare_parameter<bool>("serial_enabled", false);
    declare_parameter<bool>("use_tcp", true);
    declare_parameter<bool>("enable_lockstep", true);
    declare_parameter<std::string>("mavlink_addr", "INADDR_ANY");
    declare_parameter<int>("mavlink_udp_port", 14560);
    declare_parameter<int>("mavlink_tcp_port", 4560);
    declare_parameter<std::string>("qgc_addr", "INADDR_ANY");
    declare_parameter<int>("qgc_udp_port", 14550);
    declare_parameter<std::string>("sdk_addr", "INADDR_ANY");
    declare_parameter<int>("sdk_udp_port", 14540);
    declare_parameter<std::string>("serial_device", "/dev/ttyACM0");
    declare_parameter<int>("baud_rate", 921600);
}

/**
 * @brief Before starting, the MAVLink interface needs to be configured
 * properly. The necessary parameters are parsed from the node and set.
 *
 */
void Px4Interface::configure_mavlink_interface()
{
    std::string string_param;
    bool bool_param;
    int int_param;

    get_parameter("hil_mode", hil_mode_);
    mavlink_interface_->SetHILMode(hil_mode_);

    get_parameter("hil_state_level", hil_state_level_);
    mavlink_interface_->SetHILStateLevel(hil_state_level_);

    get_parameter("serial_enabled", bool_param);
    mavlink_interface_->SetSerialEnabled(bool_param);
    if (bool_param)
    {
        get_parameter("serial_device", string_param);
        mavlink_interface_->SetDevice(string_param);
        get_parameter("baud_rate", int_param);
        mavlink_interface_->SetBaudrate(int_param);
    }

    get_parameter("use_tcp", bool_param);
    mavlink_interface_->SetUseTcp(bool_param);

    get_parameter("enable_lockstep", bool_param);
    mavlink_interface_->SetEnableLockstep(bool_param);

    get_parameter("mavlink_addr", string_param);
    mavlink_interface_->SetMavlinkAddr(string_param);

    get_parameter("mavlink_udp_port", int_param);
    mavlink_interface_->SetMavlinkUdpPort(int_param);

    get_parameter("mavlink_tcp_port", int_param);
    mavlink_interface_->SetMavlinkTcpPort(int_param);

    get_parameter("qgc_addr", string_param);
    mavlink_interface_->SetQgcAddr(string_param);

    get_parameter("qgc_udp_port", int_param);
    mavlink_interface_->SetQgcUdpPort(int_param);

    get_parameter("sdk_addr", string_param);
    mavlink_interface_->SetSdkAddr(string_param);

    get_parameter("sdk_udp_port", int_param);
    mavlink_interface_->SetSdkUdpPort(int_param);

    mavlink_status_t *chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

    // set the Mavlink protocol version to use on the link
    if (protocol_version_ == 2.0)
    {
        chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
        RCLCPP_INFO(get_logger(), "Using MAVLink protocol v2.0\n");
    }
    else if (protocol_version_ == 1.0)
    {
        chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        RCLCPP_INFO(get_logger(), "Using MAVLink protocol v1.0\n");
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unknown protocol version! using v%g by default\n", protocol_version_);
    }
}

/**
 * @brief Receive PWM channels and store them locally
 *
 * @param msg_ptr
 */
void Px4Interface::chan_2_signal(const last_letter_msgs::msg::SimPWM::SharedPtr msg_ptr)
{
    channels_ = *msg_ptr; // Store the control inputs
    channel_functions();  // Currently does nothing
}

/**
 * @brief Callback to the last_letter link states from Gazebo. Stores them for later parsing.
 *
 * @param msg_ptr
 */
void Px4Interface::store_states_cb(
    const last_letter_msgs::msg::Airdata::SharedPtr airdata_msg_ptr,
    const last_letter_msgs::msg::Barometer::SharedPtr baro_msg_ptr,
    const last_letter_msgs::msg::Imu::SharedPtr imu_msg_ptr,
    const last_letter_msgs::msg::Gnss::SharedPtr gnss_msg_ptr,
    const last_letter_msgs::msg::MavlinkHilStateQuaternion::SharedPtr groundtruth_msg_ptr)
{
    RCLCPP_DEBUG(get_logger(), "Got a sensor callback at time %d.", px4_time_.nanoseconds() / 1000);
    readings_timestamp_ = now();
    // model_states_prev_ = model_states_;
    // model_states_ = *msg_ptr;
    // store sensor readings
    airdata_reading = *airdata_msg_ptr;
    baro_reading = *baro_msg_ptr;
    imu_reading = *imu_msg_ptr;
    gnss_reading = *gnss_msg_ptr;
    groundtruth_reading = *groundtruth_msg_ptr;

    publish_control_inputs();
}

// Entry point for the model_node controls query
// calculate and send back to Model class new control model inputs

// Build actuator controls
std::vector<double> Px4Interface::get_actuator_controls()
{
    bool armed = mavlink_interface_->GetArmedState();

    last_actuator_time_ = now();

    for (unsigned i = 0; i < n_out_max_; i++)
    {
        input_index_[i] = i;
    }
    input_reference_.resize(n_out_max_);

    Eigen::VectorXd actuator_controls = mavlink_interface_->GetActuatorControls();
    if (actuator_controls.size() < n_out_max_)
    {
        RCLCPP_ERROR(get_logger(), "Unexpected actuator vector size: %d", actuator_controls.size());
        std::vector<double> empty;
        return empty;
    }

    for (int i = 0; i < input_reference_.size(); i++)
    {
        if (armed)
        {
            input_reference_[i] = (actuator_controls[input_index_[i]]);
        }
        else
        {
            input_reference_[i] = 0;
        }
    }

    received_first_actuator_ = mavlink_interface_->GetReceivedFirstActuator();
    if (!received_first_actuator_)
    {
        RCLCPP_DEBUG(get_logger(), "Not reveiced first actuator yet");
    }

    // Convert from MAVLink SITL commands to LL model inputs
    std::vector<double> inputs(num_motors, 0);
    for (i = 0; i < num_motors; i++) // store calculated motor inputs
    {
        inputs[i] = std::max(std::min((double)input_reference_[i], 1.0), -1.0); // keep input singals in range [-1, 1]
    }

    return inputs;
}

// bool Px4Interface::return_control_inputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
//                                      last_letter_2_msgs::get_control_inputs_srv::Response &res)
void Px4Interface::publish_control_inputs()
{
    RCLCPP_DEBUG(get_logger(), "Publishing control inputs");
    if (!rclcpp::ok())
    {
        got_sig_int_ = true;
    }

    // Check if PX4 has replied once. This will allow the px4 message timestamps to start increasing
    // from 0.
    // if (!received_first_actuator_)
    // {
    //     px4_starting_timestamp_ = readings_timestamp_;
    // }

    /// Warning: in a perfect world where PX4 SITL will start completely in sync with gazebo,
    /// previous and current times will be the same, resulting in deltat=0. This will cause
    /// acceleration calculation to be skipped.
    px4_time_prev_ = px4_time_;
    // px4_time_ = now();
    px4_time_ += rclcpp::Duration(0, 4000000);
    long int timestamp_us = px4_time_.nanoseconds() / 1000;
    RCLCPP_DEBUG(get_logger(), "The time is %g", px4_time_.seconds());

    // px4_time_ = readings_timestamp_ - px4_starting_timestamp_;

    // Select the source of control inputs from PX4
    // do
    // {
    if (hil_mode_)
    {
        mavlink_interface_->pollFromQgcAndSdk();
    }
    else
    {
        int result = mavlink_interface_->pollForMAVLinkMessages(); // Reads msgs from SITL, updates input_reference_.

        // Parse return code
        switch (result)
        {
        case 0:
            RCLCPP_DEBUG(get_logger(), "Got a control response from SITL at time %d.", timestamp_us);
            break;
        case 1:
            RCLCPP_DEBUG(get_logger(), "MAVLink interface poll error at time %d.", timestamp_us);
            break;
        case 2:
            RCLCPP_DEBUG(get_logger(), "MAVLink interface poll timeout at time %d.", timestamp_us);
            break;
        case 5:
            RCLCPP_DEBUG(get_logger(), "MAVLink interface got SIGINT and will close soon at time %d.", timestamp_us);
            break;

        default:
            RCLCPP_WARN(get_logger(), "Unsupported return code from pollFromMAVLinkMessages() at time %d.", timestamp_us);
            break;
        }

        if (!mavlink_interface_->GetReceivedActuator())
        {
            RCLCPP_DEBUG(get_logger(), "MAVLink interface did not receive actuator at time %d.", timestamp_us);
        }
    }
    // } while (mavlink_interface_->GetReceivedActuator());

    // Send previously generated simulation state to PX4
    RCLCPP_DEBUG(get_logger(), "Sending HIL_ msgs to SITL, with stamp %d", px4_time_.nanoseconds() / 1000);
    send_sensor_message();
    send_gps_message();
    send_ground_truth();
    send_rc_inputs_message();

    std::vector<double> inputs = get_actuator_controls();
    if (inputs.size() == 0) // MAVLink adapter didn't find anyting, probably hasn't received anything yet
    {
        return; // Return without publishing any control inputs
    }

    // check for model_states_ update. If previous model_states_, spin once to call storeState clb for new onces and then continue
    //  if (req.header.seq != model_states_.header.seq)
    //  {
    //      RCLCPP_WARN("model_states msg is stale (%i/%i), waiting for the next one", req.header.seq, model_states_.header.seq);
    //      ros::spinOnce();
    //  }

    convert_controls_from_px4_to_ll(inputs);

    pub_ctrl->publish(ctrl_output_);
}

/**
 * @brief Controll allocation from PX4 channel order to what the last_letter
 * model expects.
 *
 * @param inputs A vector of [-1,1] doubles
 */
void Px4Interface::convert_controls_from_px4_to_ll(const std::vector<double> inputs)
{
    // This must match the PX4 SITL channels to the model channels
    // Multicopter part
    ctrl_output_.value[4] = inputs[0] * 1000 + 1000;
    ctrl_output_.value[5] = inputs[1] * 1000 + 1000;
    ctrl_output_.value[6] = inputs[2] * 1000 + 1000;
    ctrl_output_.value[7] = inputs[3] * 1000 + 1000;
    // Airplane part
    ctrl_output_.value[0] = inputs[5] * 500 + 1500;  // Aileron
    ctrl_output_.value[1] = inputs[6] * 500 + 1500;  // Elevator
    ctrl_output_.value[2] = inputs[4] * 1000 + 1000; // Throttle
    ctrl_output_.value[3] = inputs[7] * 500 + 1500;  // Rudder
    // ctrl_output_.header.stamp = rclcpp::Time(0, 0) + px4_time_;
    ctrl_output_.header.stamp = px4_time_;
}

/**
 * @brief Helper function for functionality not related to the simulated model
 * e.g. manipulating the physics engine etc.
 *
 */
void Px4Interface::channel_functions()
{
}

/**
 * @brief Print the link state member in NED coordinates
 *
 */
void Px4Interface::print_link_states_ned()
{
    std::cout << "Position " << position_ned_.transpose() << std::endl;
    std::cout << "Quaternion " << q_ned_body_.w() << ", " << q_ned_body_.vec().transpose() << std::endl;
    std::cout << "Velocity linear " << velocity_linear_ned_.transpose() << std::endl;
    std::cout << "Velocity angular " << velocity_angular_ned_.transpose() << std::endl;
    std::cout << "Acceleration linear " << acceleration_linear_ned_.transpose() << std::endl;
}

/**
 * @brief Print the link state member in Body Frame coordinates
 *
 */
void Px4Interface::print_link_states_body()
{
    auto velocity_linear_body = q_ned_body_ * velocity_linear_ned_;
    auto velocity_angular_body = q_ned_body_ * velocity_angular_ned_;
    auto acceleration_linear_body = q_ned_body_ * acceleration_linear_ned_;
    std::cout << "Position " << position_ned_.transpose() << std::endl;
    std::cout << "Quaternion " << q_ned_body_.w() << ", " << q_ned_body_.vec().transpose() << std::endl;
    std::cout << "Velocity linear " << velocity_linear_body.transpose() << std::endl;
    std::cout << "Velocity angular " << velocity_angular_body.transpose() << std::endl;
    std::cout << "Acceleration linear " << acceleration_linear_body.transpose() << std::endl;
}

/**
 * @brief Send the HIL_SENSOR MAVLink message to the PX4 SITL
 *
 */
void Px4Interface::send_sensor_message()
{

    // Create and stamp the sensor MAVLink message
    mavlink_hil_sensor_t sensor_msg;
    uint64_t timestamp = px4_time_.nanoseconds() / 1000;
    sensor_msg.time_usec = timestamp;
    sensor_msg.id = 0;
    RCLCPP_DEBUG(get_logger(), "Publishing sensor_msg with timestamp %d", timestamp);

    sensor_msg.xacc = imu_reading.accelerometer.x;
    sensor_msg.yacc = imu_reading.accelerometer.y;
    sensor_msg.zacc = imu_reading.accelerometer.z;

    sensor_msg.xgyro = imu_reading.gyroscope.x;
    sensor_msg.ygyro = imu_reading.gyroscope.y;
    sensor_msg.zgyro = imu_reading.gyroscope.z;

    sensor_msg.xmag = imu_reading.magnetometer.x;
    sensor_msg.ymag = imu_reading.magnetometer.y;
    sensor_msg.zmag = imu_reading.magnetometer.z;

    sensor_msg.temperature = airdata_reading.temperature;
    sensor_msg.abs_pressure = 0.01f * baro_reading.pressure;
    sensor_msg.pressure_alt = baro_reading.altitude;

    sensor_msg.diff_pressure = 0.01f * airdata_reading.differential_pressure;

    // Tick which fields have been filled
    sensor_msg.fields_updated = SensorSource::ACCEL | SensorSource::GYRO | SensorSource::MAG | SensorSource::BARO | SensorSource::DIFF_PRESS;

    if (!hil_mode_ || (hil_mode_ && !hil_state_level_))
    {
        mavlink_message_t msg;
        mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
        mavlink_interface_->send_mavlink_message(&msg);
    }
}

/**
 * @brief Send HIL_GPS MAVLink message to PX4 SITL
 *
 */
void Px4Interface::send_gps_message()
{
    // Construct the orientation quaternion

    // fill HIL GPS Mavlink msg
    mavlink_hil_gps_t hil_gps_msg;
    hil_gps_msg.time_usec = px4_time_.nanoseconds() / 1000;
    hil_gps_msg.id = 0;

    // Assume 3D fix available always
    hil_gps_msg.fix_type = gnss_reading.status.status;
    // Insert coordinate information
    hil_gps_msg.lat = gnss_reading.latitude * 1e7;
    hil_gps_msg.lon = gnss_reading.longitude * 1e7;
    hil_gps_msg.alt = gnss_reading.altitude * 1000;
    hil_gps_msg.eph = gnss_reading.eph;
    hil_gps_msg.epv = gnss_reading.epv;

    // Insert inertial velocity information
    hil_gps_msg.vn = gnss_reading.velocity.x * 100;
    hil_gps_msg.ve = gnss_reading.velocity.y * 100;
    hil_gps_msg.vd = gnss_reading.velocity.z * 100;
    Vector3d velocity{
        gnss_reading.velocity.x,
        gnss_reading.velocity.y,
        gnss_reading.velocity.z};
    hil_gps_msg.vel = velocity.norm() * 100.0;

    // Calculate course over ground
    hil_gps_msg.cog = static_cast<uint16_t>(gnss_reading.cog * 180 / M_PI * 100.0);
    hil_gps_msg.satellites_visible = gnss_reading.satellites;

    // send HIL_GPS Mavlink msg
    if (!hil_mode_ || (hil_mode_ && !hil_state_level_))
    {
        mavlink_message_t msg;
        mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);
        mavlink_interface_->send_mavlink_message(&msg);
    }
}

/**
 * @brief Send HIL_STATE_QUATERNION MAVLink message to PX4 SITL
 *
 */
void Px4Interface::send_ground_truth()
{
    mavlink_hil_state_quaternion_t hil_state_quat;

    // Insert time information.
    hil_state_quat.time_usec = px4_time_.nanoseconds() / 1000;

    // Insert attitude information.
    hil_state_quat.attitude_quaternion[0] = groundtruth_reading.quaternion.w;
    hil_state_quat.attitude_quaternion[1] = groundtruth_reading.quaternion.x;
    hil_state_quat.attitude_quaternion[2] = groundtruth_reading.quaternion.y;
    hil_state_quat.attitude_quaternion[3] = groundtruth_reading.quaternion.z;

    // Insert angular velocity information.
    hil_state_quat.rollspeed = groundtruth_reading.velocity_angular.x;
    hil_state_quat.pitchspeed = groundtruth_reading.velocity_angular.y;
    hil_state_quat.yawspeed = groundtruth_reading.velocity_angular.z;

    // Insert coordinate information
    hil_state_quat.lat = groundtruth_reading.latitude * 1e7;
    hil_state_quat.lon = groundtruth_reading.longitude * 1e7;
    hil_state_quat.alt = groundtruth_reading.altitude * 1000;

    // Insert inertial velocity information
    hil_state_quat.vx = groundtruth_reading.gnss_velocity.x * 100;
    hil_state_quat.vy = groundtruth_reading.gnss_velocity.y * 100;
    hil_state_quat.vz = groundtruth_reading.gnss_velocity.z * 100;

    // Insert airspeed information
    hil_state_quat.ind_airspeed = groundtruth_reading.airspeed_indicated * 100;
    hil_state_quat.true_airspeed = groundtruth_reading.airspeed_true * 100;

    // Insert acceleration information
    double g = 9.81;
    hil_state_quat.xacc = groundtruth_reading.acceleration.x * 1000 / g;
    hil_state_quat.yacc = groundtruth_reading.acceleration.y * 1000 / g;
    hil_state_quat.zacc = groundtruth_reading.acceleration.z * 1000 / g;

    if (!hil_mode_ || (hil_mode_ && !hil_state_level_))
    {
        mavlink_message_t msg;
        mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_state_quat);
        mavlink_interface_->send_mavlink_message(&msg);
    }
}

/**
 * @brief Send the pilot's RC commands to the PX4 SITL as a RC_CHANNELS message
 *
 */
void Px4Interface::send_rc_inputs_message()
{
    mavlink_rc_channels_t rc_channels_msg;
    rc_channels_msg.time_boot_ms = px4_time_.nanoseconds() / 1000000;
    rc_channels_msg.chancount = 8;
    rc_channels_msg.chan1_raw = channels_.value[0];
    rc_channels_msg.chan2_raw = channels_.value[1];
    rc_channels_msg.chan3_raw = channels_.value[2];
    rc_channels_msg.chan4_raw = channels_.value[3];
    rc_channels_msg.chan5_raw = channels_.value[4];
    rc_channels_msg.chan6_raw = channels_.value[5];
    rc_channels_msg.chan7_raw = channels_.value[6];
    rc_channels_msg.chan8_raw = channels_.value[7];
    rc_channels_msg.rssi = 254;

    if (!hil_mode_ || (hil_mode_ && !hil_state_level_))
    {
        mavlink_message_t msg;
        mavlink_msg_rc_channels_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &rc_channels_msg);
        mavlink_interface_->send_mavlink_message(&msg);
    }
}

// Handler of SIGINT for nodes raised by roslaunch
void Px4Interface::custom_sigint_handler()
{
    mavlink_interface_->onSigInt();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4Interface>());
    rclcpp::shutdown();
    return 0;
}
