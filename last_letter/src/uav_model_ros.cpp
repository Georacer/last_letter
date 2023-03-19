// UavModelNode class definitions
#include <chrono>
#include "last_letter/uav_model_ros.hpp"

// #include <geometry_msgs/Transform.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <sensor_msgs/JointState.h>

//////////////////////////
// Define UavModelNode class
//////////////////////////

using namespace std::chrono_literals;

///////////////////
// Class Constructor
UavModelNode::UavModelNode() : Node("uav_model_node")
{
    // Get the UAV name from the parameter server and build the relevant UAV configuration struct
    this->declare_parameter<std::string>("uav_name", "uav_name");
    this->get_parameter("uav_name", uavName);
    RCLCPP_INFO(this->get_logger(), "Loading uav configuration for %s", uavName.c_str());
    lll::programming_utils::ParameterManager configStruct = lll::programming_utils::loadModelConfig(uavName);
    // Parameter randomization is done internally in loadModelConfig

    RCLCPP_INFO(this->get_logger(), "Creating new UavModel");
    uavModel = new lll::UavModel(configStruct); // Create a UavStruct by passing the configurations bundle struct

    tprev = this->now();

    // Subscribe and advertize
    subState = this->create_subscription<last_letter_msgs::msg::LinkStates>(
        "/ll_link_states", 10, std::bind(&UavModelNode::getState_cb, this, std::placeholders::_1));
    subInp = this->create_subscription<last_letter_msgs::msg::SimPWM>(
        "/ctrlPWM",
        1,
        std::bind(&UavModelNode::getInput, this, std::placeholders::_1)); // model control input subscriber
    subParam = this->create_subscription<last_letter_msgs::msg::Parameter>(
        "parameter_changes",
        1000,
        std::bind(&UavModelNode::getParameters, this, std::placeholders::_1));
    gazebo_pub_ = create_publisher<last_letter_msgs::msg::SimCommands>("/sim_commands", 10);

    pause_client_ = this->create_client<std_srvs::srv::Empty>("/pause_physics");
    unpause_client_ = this->create_client<std_srvs::srv::Empty>("/unpause_physics");

    timer_ptr_ = this->create_wall_timer(1000ms, std::bind(&UavModelNode::timerCallback, this));

    // Initialize acceleration filters
    filt_ax_.setup(f_sample_, f_cut_);
    filt_ay_.setup(f_sample_, f_cut_);
    filt_az_.setup(f_sample_, f_cut_);

    // Initialize sensor publishers
    airdata_pub_ = create_publisher<last_letter_msgs::msg::Airdata>("/airdata", 10);
    baro_pub_ = create_publisher<last_letter_msgs::msg::Barometer>("/barometer", 10);
    imu_pub_ = create_publisher<last_letter_msgs::msg::Imu>("/imu", 10);
    gnss_pub_ = create_publisher<last_letter_msgs::msg::Gnss>("/gnss", 10);
    groundtruth_pub_ = create_publisher<last_letter_msgs::msg::MavlinkHilStateQuaternion>("/groundtruth", 10);
}

// Initialize states
void UavModelNode::init()
{
    uavModel->init();
}

void UavModelNode::timerCallback()
{
    gazebo_commands = buildGazeboCommands();
    // Publish commands
    gazebo_pub_->publish(gazebo_commands);
}

///////////////////////////////////////
// make one step of the plane simulation
void UavModelNode::step()
{
    if (_new_parameters)
    {
        uavModel->update_model();
        _new_parameters = false;
    }

    // Step with the pre-assigned link states
    uavModel->step();

    // Build commands
    gazebo_commands = buildGazeboCommands();

    // Publish commands
    gazebo_pub_->publish(gazebo_commands);

    tprev = this->now();
}

void UavModelNode::pauseLogic(const uint16_t pause_pwm)
{
    // Pause button is pressed
    if (pause_pwm > 1800)
    {
        if (!pauseButtonPressed_)
        // It has not been registered
        {
            pauseButtonPressed_ = true; // Acknowledge button press

            // if (timeControls == 3)
            // // Working by manual stepping, trigger simulation step
            // {
            // 	step_required = true;
            // }
            // else // Working on rest of stepping methods
            {
                pausedState_ = !pausedState_;
                auto request = std::make_shared<std_srvs::srv::Empty::Request>();

                if (pausedState_)
                {
                    RCLCPP_INFO(get_logger(), "Paused simulation");
                    pause_client_->async_send_request(request);
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "Unpausing simulation");
                    unpause_client_->async_send_request(request);
                }
            }
        }
    }
    else
    {
        pauseButtonPressed_ = false; // Acknowlege button release
    }
}

/////////////////////////////////////////////////
// Pass PWM control input signals to uavModel
void UavModelNode::getInput(const last_letter_msgs::msg::SimPWM::SharedPtr inputMsgPtr)
{
    input = *inputMsgPtr;
    InputPwm_t modelInput;
    for (int i = 0; i < 12; i++)
    {
        modelInput.value[i] = (*inputMsgPtr).value[i];
    }
    uavModel->setInputPwm(modelInput);

    // Handle pausing of physics
    pauseLogic(input.value[9]);

    step();
}

void UavModelNode::getState_cb(const last_letter_msgs::msg::LinkStates::SharedPtr linkStatesPtr)
{
    // perform step actions serially

    convertLinkStates(*linkStatesPtr, linkStates_);

    // Set the states in the uav model
    uavModel->setLinkStates(linkStates_);

    // publish the sensor messages
    publishSensors();
}

void UavModelNode::convertLinkStates(const last_letter_msgs::msg::LinkStates linkStates, LinkStateMap_t &linkStateMap)
{
    for (uint idx = 0; idx < linkStates.name.size(); ++idx)
    {
        std::vector<std::string> names;
        lll::programming_utils::split_string(linkStates.name[idx], names, ':');
        std::string model_name = names[0];
        std::string link_name = names[2];
        SimState_t link_state;

        // Distribute link state elements and rotate from ENU to NED
        Vector3d position_enu{
            linkStates.pose[idx].position.x,
            linkStates.pose[idx].position.y,
            linkStates.pose[idx].position.z,
        };
        link_state.pose.position = q_enu_ned * position_enu;

        Quaterniond q_m_enu{
            linkStates.pose[idx].orientation.w,
            linkStates.pose[idx].orientation.x,
            linkStates.pose[idx].orientation.y,
            linkStates.pose[idx].orientation.z};
        Quaterniond q_enu_m = q_m_enu.conjugate();
        link_state.pose.orientation = q_enu_m * q_ned_enu; // build q_ned_m

        Vector3d velocity_linear_enu{
            linkStates.velocity[idx].linear.x,
            linkStates.velocity[idx].linear.y,
            linkStates.velocity[idx].linear.z};
        link_state.velocity.linear = q_enu_ned * velocity_linear_enu;

        Vector3d velocity_angular_enu{
            linkStates.velocity[idx].angular.x,
            linkStates.velocity[idx].angular.y,
            linkStates.velocity[idx].angular.z};
        link_state.velocity.angular = q_enu_ned * velocity_angular_enu;

        // Filter acceleration when close to the ground to avoid physics engine jitter
        Vector3d acceleration_linear_enu;
        if (link_state.pose.position.z() > -0.5)
        {
            acceleration_linear_enu.x() = filt_ax_.filter(linkStates.acceleration[idx].linear.x);
            acceleration_linear_enu.y() = filt_ay_.filter(linkStates.acceleration[idx].linear.y);
            acceleration_linear_enu.z() = filt_az_.filter(linkStates.acceleration[idx].linear.z);
        }
        else
        {
            acceleration_linear_enu.x() = linkStates.acceleration[idx].linear.x;
            acceleration_linear_enu.y() = linkStates.acceleration[idx].linear.y;
            acceleration_linear_enu.z() = linkStates.acceleration[idx].linear.z;
        };
        link_state.acceleration.linear = q_enu_ned * acceleration_linear_enu;

        linkStateMap[link_name] = link_state;
    }
}

void UavModelNode::publishSensors()
{
    // Initialize sensor publishers
    // for (auto it = std::begin(uavModel->sensors); it != std::end(uavModel->sensors); ++ it)
    for (auto sensor_ptr : uavModel->sensors)
    {
        switch (sensor_ptr->type)
        {
        case last_letter_lib::sensor_t::IMU:
            imu_pub_->publish(buildImuMsg(sensor_ptr.get()));
            break;
        case last_letter_lib::sensor_t::BAROMETER:
            baro_pub_->publish(buildBaroMsg(sensor_ptr.get()));
            break;
        case last_letter_lib::sensor_t::AIRDATA:
            airdata_pub_->publish(buildAirdataMsg(sensor_ptr.get()));
            break;
        case last_letter_lib::sensor_t::GPS:
            gnss_pub_->publish(buildGnssMsg(sensor_ptr.get()));
            break;
        case last_letter_lib::sensor_t::MAVLINK_HIL_STATE_QUATERNION:
            groundtruth_pub_->publish(buildGroundtruthMsg(sensor_ptr.get()));
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown sensor type %d", sensor_ptr->type);
            std::cout << "Unknown sensor type: " << int(sensor_ptr->type) << std::endl;
            break;
        }
    }
}

last_letter_msgs::msg::Airdata UavModelNode::buildAirdataMsg(last_letter_lib::Sensor *base_sensor)
{
    auto *sensor_ptr = static_cast<last_letter_lib::AirdataSensor *>(base_sensor);
    last_letter_msgs::msg::Airdata msg;
    msg.header.stamp = now();
    msg.differential_pressure = sensor_ptr->differential_pressure;
    msg.static_pressure = sensor_ptr->static_pressure;
    msg.temperature = sensor_ptr->temperature;
    msg.aoa = sensor_ptr->aoa;
    msg.aos = sensor_ptr->aos;
    return msg;
}

last_letter_msgs::msg::Barometer UavModelNode::buildBaroMsg(last_letter_lib::Sensor *base_sensor)
{
    auto sensor_ptr = static_cast<last_letter_lib::Barometer *>(base_sensor);
    last_letter_msgs::msg::Barometer msg;
    msg.header.stamp = now();
    msg.temperature = sensor_ptr->temperature_reading;
    msg.pressure = sensor_ptr->pressure_reading;
    msg.altitude = sensor_ptr->altitude_reading;
    return msg;
}

last_letter_msgs::msg::Imu UavModelNode::buildImuMsg(last_letter_lib::Sensor *base_sensor)
{
    auto sensor_ptr = static_cast<last_letter_lib::Imu *>(base_sensor);
    last_letter_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.accelerometer.x = sensor_ptr->accelerometer_reading.x();
    msg.accelerometer.y = sensor_ptr->accelerometer_reading.y();
    msg.accelerometer.z = sensor_ptr->accelerometer_reading.z();
    msg.gyroscope.x = sensor_ptr->gyroscope_reading.x();
    msg.gyroscope.y = sensor_ptr->gyroscope_reading.y();
    msg.gyroscope.z = sensor_ptr->gyroscope_reading.z();
    msg.magnetometer.x = sensor_ptr->magnetometer_reading.x();
    msg.magnetometer.y = sensor_ptr->magnetometer_reading.y();
    msg.magnetometer.z = sensor_ptr->magnetometer_reading.z();
    return msg;
}

last_letter_msgs::msg::Gnss UavModelNode::buildGnssMsg(last_letter_lib::Sensor *base_sensor)
{
    auto sensor_ptr = static_cast<last_letter_lib::Gnss *>(base_sensor);
    last_letter_msgs::msg::Gnss msg;
    msg.header.stamp = now();
    msg.status.status = sensor_ptr->fix_type;
    msg.latitude = sensor_ptr->latitude;
    msg.longitude = sensor_ptr->longitude;
    msg.altitude = sensor_ptr->altitude;
    msg.eph = sensor_ptr->eph;
    msg.epv = sensor_ptr->epv;
    msg.velocity.x = sensor_ptr->velocity_ned.x();
    msg.velocity.y = sensor_ptr->velocity_ned.y();
    msg.velocity.z = sensor_ptr->velocity_ned.z();
    msg.evh = sensor_ptr->evh;
    msg.evv = sensor_ptr->evv;
    msg.cog = sensor_ptr->course_over_ground;
    msg.satellites = sensor_ptr->satellites;
    return msg;
}

last_letter_msgs::msg::MavlinkHilStateQuaternion UavModelNode::buildGroundtruthMsg(last_letter_lib::Sensor *base_sensor)
{
    auto sensor_ptr = static_cast<last_letter_lib::MavlinkHilStateQuaternion *>(base_sensor);
    last_letter_msgs::msg::MavlinkHilStateQuaternion msg;
    msg.header.stamp = now();
    Quaterniond orientation = sensor_ptr->attitude.conjugate();
    msg.quaternion.x = orientation.x();
    msg.quaternion.y = orientation.y();
    msg.quaternion.z = orientation.z();
    msg.quaternion.w = orientation.w();
    msg.acceleration.x = sensor_ptr->acceleration_linear.x();
    msg.acceleration.y = sensor_ptr->acceleration_linear.y();
    msg.acceleration.z = sensor_ptr->acceleration_linear.z();
    msg.velocity_angular.x = sensor_ptr->velocity_angular.x();
    msg.velocity_angular.y = sensor_ptr->velocity_angular.y();
    msg.velocity_angular.z = sensor_ptr->velocity_angular.z();
    msg.latitude = sensor_ptr->latitude;
    msg.longitude = sensor_ptr->longitude;
    msg.altitude = sensor_ptr->altitude;
    msg.gnss_velocity.x = sensor_ptr->velocity_ned.x();
    msg.gnss_velocity.y = sensor_ptr->velocity_ned.y();
    msg.gnss_velocity.z = sensor_ptr->velocity_ned.z();
    msg.airspeed_indicated = sensor_ptr->airspeed_indicated;
    msg.airspeed_true = sensor_ptr->airspeed_true;
    return msg;
}

///////////////////////////////////////
// Read model parameters and apply them
void UavModelNode::getParameters(const last_letter_msgs::msg::Parameter::SharedPtr paramMsgPtr)
{
    lll::programming_utils::ParamType_t msg_type = static_cast<lll::programming_utils::ParamType_t>(paramMsgPtr->type);
    if (uavModel->set_parameter(msg_type, paramMsgPtr->name, paramMsgPtr->value))
    {
        _new_parameters = true;
    }
}

//////////////////
// Class destructor
UavModelNode::~UavModelNode()
{
    delete uavModel;
}

last_letter_msgs::msg::SimCommands UavModelNode::buildGazeboCommands()
{
    // Construct fake control msg
    auto ctrl_msg = last_letter_msgs::msg::SimCommands();
    ctrl_msg.header.stamp = this->now();
    // Push aerodynamics
    for (int idx = 0; idx < uavModel->dynamics.nWings; idx++)
    {
        auto link_ptr = uavModel->dynamics.aerodynamicLinks[idx];
        ctrl_msg.name.push_back(uavName + std::string("::") + link_ptr->name);
        ctrl_msg.target_type.push_back(last_letter_msgs::msg::SimCommands::TARGET_LINK);
        ctrl_msg.command_type.push_back(last_letter_msgs::msg::SimCommands::TYPE_WRENCH);
        ctrl_msg.values.push_back(buildPayloadWrench(link_ptr->wrenchLinkFrame));
    }
    // Push propulsion
    for (int idx = 0; idx < uavModel->dynamics.nMotors; idx++)
    {
        auto link_ptr = uavModel->dynamics.propulsionLinks[idx];
        ctrl_msg.name.push_back(uavName + std::string("::") + link_ptr->name);
        ctrl_msg.target_type.push_back(last_letter_msgs::msg::SimCommands::TARGET_LINK);
        ctrl_msg.command_type.push_back(last_letter_msgs::msg::SimCommands::TYPE_WRENCH);
        ctrl_msg.values.push_back(buildPayloadWrench(link_ptr->wrenchLinkFrame));
    }

    return ctrl_msg;
}

last_letter_msgs::msg::CommandPayload buildPayloadWrench(Wrench_t wrench)
{
    auto payload = last_letter_msgs::msg::CommandPayload();
    // Rotate from NED to ENU
    payload.linear.x = wrench.force.x();
    payload.linear.y = wrench.force.y();
    payload.linear.z = wrench.force.z();
    payload.angular.x = wrench.torque.x();
    payload.angular.y = wrench.torque.y();
    payload.angular.z = wrench.torque.z();

    return payload;
}

// void convertStatesDerivatives(const Derivatives_t simDerivatives, last_letter_msgs::SimStates &wrapperStateDot)
// {
//     wrapperStateDot.pose.position.x = simDerivatives.posDot.x();
//     wrapperStateDot.pose.position.y = simDerivatives.posDot.y();
//     wrapperStateDot.pose.position.z = simDerivatives.posDot.z();
//     wrapperStateDot.pose.orientation.x = simDerivatives.quatDot.x();
//     wrapperStateDot.pose.orientation.y = simDerivatives.quatDot.y();
//     wrapperStateDot.pose.orientation.z = simDerivatives.quatDot.z();
//     wrapperStateDot.pose.orientation.w = simDerivatives.quatDot.w();
//     wrapperStateDot.velocity.linear.x = simDerivatives.speedDot.x();
//     wrapperStateDot.velocity.linear.y = simDerivatives.speedDot.y();
//     wrapperStateDot.velocity.linear.z = simDerivatives.speedDot.z();
//     wrapperStateDot.velocity.angular.x = simDerivatives.rateDot.x();
//     wrapperStateDot.velocity.angular.y = simDerivatives.rateDot.y();
//     wrapperStateDot.velocity.angular.z = simDerivatives.rateDot.z();
//     wrapperStateDot.geoid.latitude = simDerivatives.coordDot.x();
//     wrapperStateDot.geoid.longitude = simDerivatives.coordDot.y();
//     wrapperStateDot.geoid.altitude = simDerivatives.coordDot.z();
//     wrapperStateDot.geoid.velocity.x = simDerivatives.posDot.x();
//     wrapperStateDot.geoid.velocity.y = simDerivatives.posDot.y();
//     wrapperStateDot.geoid.velocity.z = -simDerivatives.posDot.z();
// }

// void convertEnvironment(const Environment_t simEnv, last_letter_msgs::Environment &wrapperEnv)
// {
//     wrapperEnv.wind.x = simEnv.wind.x(); // Note: Wind is expressed in body frame
//     wrapperEnv.wind.y = simEnv.wind.y();
//     wrapperEnv.wind.z = simEnv.wind.z();
//     wrapperEnv.density = simEnv.density;
//     wrapperEnv.pressure = simEnv.pressure;
//     wrapperEnv.temperature = simEnv.temperature;
//     wrapperEnv.gravity = simEnv.gravity;
// }

// void convertRosVector3(const Eigen::Vector3d vector3Eigen, geometry_msgs::Vector3 &vector3Ros)
// {
//     vector3Ros.x = vector3Eigen.x();
//     vector3Ros.y = vector3Eigen.y();
//     vector3Ros.z = vector3Eigen.z();
// }

// void convertRosQuaternion(const Eigen::Quaterniond quatEigen, geometry_msgs::Quaternion &quatRos)
// {
//     quatRos.x = quatEigen.x();
//     quatRos.y = quatEigen.y();
//     quatRos.z = quatEigen.z();
//     quatRos.w = quatEigen.w();
// }

// void convertTfVector3(const Eigen::Vector3d vector3Eigen, tf::Vector3 &vector3Tf)
// {
//     vector3Tf = tf::Vector3(vector3Eigen.x(), vector3Eigen.y(), vector3Eigen.z());
// }

// void convertTfQuaternion(const Eigen::Quaterniond quatEigen, tf::Quaternion &quatTf)
// {
//     quatTf = tf::Quaternion(quatEigen.x(), quatEigen.y(), quatEigen.z(), quatEigen.w());
// }

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UavModelNode>());
    rclcpp::shutdown();
    return 0;
}
