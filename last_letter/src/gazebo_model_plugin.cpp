#include <last_letter/gazebo_model_plugin.hpp>

#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
// #include <rclcpp/rclcpp.hpp>
#include <gazebo/common/Plugin.hh>
// #include <gazebo/gazebo_client.hh> //gazebo version >6
#include <gazebo/physics/physics.hh>
// #include "ros/callback_queue.h"
// #include "ros/subscribe_options.h"
// #include <last_letter_2_msgs/model_states.h>
// #include <last_letter_2_msgs/link_states.h>
// #include <last_letter_2_msgs/apply_model_wrenches_srv.h>
// #include <last_letter_2_msgs/channels.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/convert.h>
#include <ignition/math/Vector3.hh>
// #include <kdl_parser/kdl_parser.hpp>
// #include <boost/bind.hpp>
// #include <thread>
// #include <mutex>
// #include <condition_variable>
#include <gazebo_ros/conversions/geometry_msgs.hpp>

#include <last_letter_lib/prog_utils.hpp>

namespace lll = last_letter_lib;

bool commands_applied;

void ModelPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _plugin_sdf) //Called when a Plugin is first created,
{                                                                                     //and after the World has been loaded.Νot be blocking.
    model_ = _model;
    model_name_ = _model->GetName();
    plugin_sdf_ = _plugin_sdf;

    this->ros_node_ = gazebo_ros::Node::Get(_plugin_sdf);                                           // Get reference to a #gazebo_ros::Node and add it to the global #gazebo_ros::Executor.
    RCLCPP_DEBUG(ros_node_->get_logger(), "ModelPlugin started for model %s", model_name_.c_str()); // TODO: ROS2 update

    while (!rclcpp::ok())
    {
        RCLCPP_DEBUG(ros_node_->get_logger(), "Waiting for node to rise"); // TODO: ROS2 update
    }

    // // Spin up the queue helper thread.
    // this->rosQueueThread =
    //     std::thread(std::bind(&ModelPlugin::QueueThread, this));

    //Connect a callback to the world update start signal.
    before_update_connection_ = gazebo::event::Events::ConnectBeforePhysicsUpdate(std::bind(&ModelPlugin::before_update, this));
    update_connection_end_ = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&ModelPlugin::on_update, this));

    // // Service server
    // ros::AdvertiseServiceOptions so = (ros::AdvertiseServiceOptions::create<last_letter_2_msgs::apply_model_wrenches_srv>("last_letter_2/apply_model_wrenches_srv",
    //                                                                                                                       boost::bind(&ModelPlugin::applyWrenchesOnModel, this, _1, _2), ros::VoidPtr(), &this->wrenches_rosQueue));
    // this->apply_wrenches_server = this->ros_node_->advertiseService(so);

    //Subscriber
    commands_sub_ = ros_node_->create_subscription<last_letter_msgs::msg::SimCommands>(
        "/sim_commands",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&ModelPlugin::applied_wrenches_cb, this, std::placeholders::_1));
    // rclcpp::SubscribeOptions soo = (ros::SubscribeOptions::create<last_letter_2_msgs::channels>("last_letter_2/channels", 1, boost::bind(&ModelPlugin::manageChan, this, _1), ros::VoidPtr(), &this->wrenches_rosQueue));
    // this->channels_sub = this->ros_node_->subscribe(soo);

    // // Publish code
    pub_state_ = ros_node_->create_publisher<last_letter_msgs::msg::LinkStates>("/ll_link_states", 100);

    // //Read the number of airfoils
    // if (!ros::param::getCached("nWings", num_wings))
    // {
    //     ROS_FATAL("Invalid parameters for wings_number in param server!");
    //     ros::shutdown();
    // }
    // //Read the number of motors
    // if (!ros::param::getCached("nMotors", num_motors))
    // {
    //     ROS_FATAL("Invalid parameters for motor_number in param server!");
    //     ros::shutdown();
    // }
    // //Read the camera angle channel if available
    // if (ros::param::getCached("channels/camera_angle_chan", camera_angle_chan))
    // {
    //     ROS_INFO("Camera angle channel loaded");
    // }
    // //Read the laser angle channel if available
    // if (ros::param::getCached("channels/laser_angle_chan", laser_angle_chan))
    // {
    //     ROS_INFO("Laser angle channel loaded");
    // }
    // //Read the reset Simulation channel
    // if (ros::param::getCached("channels/reset_sim_chan", reset_sim_chan))
    // {
    //     ROS_INFO("Reset simulation channel loaded");
    // }

    // char paramMsg[50];
    commands_applied = false;
    model_state_init();
    RCLCPP_DEBUG(ros_node_->get_logger(), "Done loading plugin %s", model_name_.c_str()); // TODO: ROS2 update
}

// Init variables
void ModelPlugin::model_state_init()
{
    // omega = 0;
    // camera_angle = 0;
    // laser_angle = 0;

    // //Get initial model states from parameter server
    // XmlRpc::XmlRpcValue list;

    // if (!ros::param::getCached("init/position", list))
    // {
    //     ROS_FATAL("Invalid parameters for init/position in param server!");
    //     ros::shutdown();
    // }
    // ignition::math::Vector3d xyz_pose(list[0], list[1], list[2]);
    // ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    // if (!ros::param::getCached("init/orientation", list))
    // {
    //     ROS_FATAL("Invalid parameters for init/orientation in param server!");
    //     ros::shutdown();
    // }
    // ignition::math::Vector3d rpy_pose(list[0], list[1], list[2]);
    // if (!ros::param::getCached("init/velLin", list))
    // {
    //     ROS_FATAL("Invalid parameters for init/velLin in param server!");
    //     ros::shutdown();
    // }
    // ignition::math::Vector3d velLin(list[0], list[1], list[2]);
    // if (!ros::param::getCached("init/velAng", list))
    // {
    //     ROS_FATAL("Invalid parameters for init/velAng in param server!");
    //     ros::shutdown();
    // }
    // ignition::math::Vector3d velAng(list[0], list[1], list[2]);

    // //Set the initial position and rotation
    // ignition::math::Pose3d init_pose;
    // init_pose.Set(xyz_pose, rpy_pose);
    // this->model->SetWorldPose(init_pose);

    // // Transform linear and angular velocity from body frame to world frame for initialized launching
    // KDL::Frame transformation_matrix;
    // tf2::Stamped<KDL::Vector> v_out;

    // transformation_matrix = KDL::Frame(KDL::Rotation::EulerZYX(-rpy_pose[2], -rpy_pose[1], rpy_pose[0]), KDL::Vector(0, 0, 0)); // negative sign is used to bring data on FLU frame that gazebo uses
    // v_out = tf2::Stamped<KDL::Vector>(transformation_matrix.Inverse() * KDL::Vector(velLin[0], velLin[1], velLin[2]), ros::Time::now(), "airfoil");

    // velLin[0] = v_out[0];
    // velLin[1] = v_out[1];
    // velLin[2] = v_out[2];

    // v_out = tf2::Stamped<KDL::Vector>(transformation_matrix.Inverse() * KDL::Vector(velAng[0], velAng[1], velAng[2]), ros::Time::now(), "airfoil");

    // velAng[0] = v_out[0];
    // velAng[1] = v_out[1];
    // velAng[2] = v_out[2];

    // // Set velocities on model
    // this->model->SetLinearVel(velLin);
    // this->model->SetAngularVel(velAng);
}

//  ROS helper function that processes messages
void ModelPlugin::QueueThread()
{
    // // Set thread rate= 2.2 * simulation frequency
    // // After many tries, it seems to be the most efficent rate
    // thread_rate_multiplier = 2.2;
    // thread_rate = thread_rate_multiplier * model->GetWorld()->Physics()->GetRealTimeUpdateRate();
    // ROS_INFO(" Gazebo queue thread started, rate = %d\n", thread_rate);

    // // the sleep rate, increase dramaticaly the preformance
    // ros::WallRate r(thread_rate);
    // while (this->ros_node_->ok())
    // {
    //     this->wrenches_rosQueue.callAvailable();
    //     r.sleep();
    // }
}

// service that apply the calculated aerodynamic and propulsion wrenches on relative links of model
void ModelPlugin::applied_wrenches_cb(const last_letter_msgs::msg::SimCommands::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lk(m);

    // Iterate over all the links
    for (uint idx = 0; idx < msg->name.size(); ++idx)
    {
        RCLCPP_DEBUG(ros_node_->get_logger(), "Got command for link %s", msg->name[idx].c_str());
        std::vector<std::string> names;
        lll::programming_utils::split_string(msg->name[idx], names, ':');
        std::string model_name = names[0];
        std::string target_name = names[2];
        // Filter for this model
        if (model_name_.compare(model_name) == 0)
        {
            // TODO: Check if this link exists, otherwise plugin will crash
            // See if this command refers to a link or a joint
            if (msg->target_type[idx] == last_letter_msgs::msg::SimCommands::TARGET_LINK)
            {
                // Switch on command type
                switch (msg->command_type[idx])
                {
                case last_letter_msgs::msg::SimCommands::TYPE_POSE:
                    break;
                case last_letter_msgs::msg::SimCommands::TYPE_VELOCITY:
                    break;
                case last_letter_msgs::msg::SimCommands::TYPE_ACCELERATION:
                    break;
                case last_letter_msgs::msg::SimCommands::TYPE_WRENCH:
                    // Apply wrenches on mentioned links (link_frame)
                    auto force = ignition::math::Vector3d(
                        msg->values[idx].linear.x,
                        msg->values[idx].linear.y,
                        msg->values[idx].linear.z);
                    model_->GetLink(target_name)->AddLinkForce(force);
                    auto torque = ignition::math::Vector3d(
                        msg->values[idx].angular.x,
                        msg->values[idx].angular.y,
                        msg->values[idx].angular.z);
                    model_->GetLink(target_name)->AddRelativeTorque(torque);
                    RCLCPP_DEBUG(ros_node_->get_logger(), "Applying force %g, %g, %g to link %s",
                                 force.X(),
                                 force.Y(),
                                 force.Z(),
                                 target_name.c_str());
                    break;
                }
            }
            else if (msg->target_type[idx] == last_letter_msgs::msg::SimCommands::TARGET_JOINT)
            {
                // Apply commands on joints
                // Switch on command type
                switch (msg->command_type[idx])
                {
                case last_letter_msgs::msg::SimCommands::TYPE_POSE:
                    break;
                case last_letter_msgs::msg::SimCommands::TYPE_VELOCITY:
                    double omega;
                    omega = msg->values[idx].angular.x;
                    model_->GetJoint(target_name)->SetVelocity(0, omega);
                    break;
                case last_letter_msgs::msg::SimCommands::TYPE_ACCELERATION:
                    break;
                case last_letter_msgs::msg::SimCommands::TYPE_WRENCH:
                    break;
                }
            }
        }

        // //Handle sensors

        // //Check if camera_joint exists to set the camera_angle
        // if (model->GetJoint("camera_joint"))
        // {
        //     model->GetJoint("camera_joint")->SetPosition(0, camera_angle);
        // }

        // //Check if laser_joint exists to set the laser_angle
        // if (model->GetJoint("laser_joint"))
        // {
        //     model->GetJoint("laser_joint")->SetPosition(0, laser_angle);
        // }
    }
    //unlock gazebo step
    commands_applied = true;
    cv.notify_one();
    RCLCPP_DEBUG(ros_node_->get_logger(), "Exiting commands callback");
}

// //Manage channel functions
// void ModelPlugin::manageChan(const last_letter_2_msgs::channels::ConstPtr &channels)
// {
// //Handle Camera's angle
// if (model->GetJoint("camera_joint"))
// {
//     if (channels->value[camera_angle_chan] == -1 && camera_angle >= -1.9)
//     {
//         camera_angle -= 0.1;
//     }
//     if (channels->value[camera_angle_chan] == 1 && camera_angle <= 1.9)
//     {
//         camera_angle += 0.1;
//     }
// }

// //Handle Laser's angle
// if (model->GetJoint("laser_joint"))
// {
//     if (channels->value[laser_angle_chan] == 1 && laser_angle >= -0.9)
//     {
//         laser_angle -= 0.1;
//     }
//     if (channels->value[laser_angle_chan] == -1 && laser_angle <= 0.9)
//     {
//         laser_angle += 0.1;
//     }
// }

// //Reset simulation
// if (channels->value[reset_sim_chan] == 1)
// {
//     model_state_init();
// }
// }

// Callback that listens to signal before physics Engine start.
void ModelPlugin::before_update()
{
    std::unique_lock<std::mutex> lk(m);
    // //wait until commands are ready
    // May not be necessary because PX4 interface will skip polling if no actuators are received
    if (step_number_ > 1000) // do 50 steps without being stuck, to be sure that everything is ready
    {
        RCLCPP_DEBUG(ros_node_->get_logger(), "Waiting for lock release");
        cv.wait(lk, [] { return commands_applied == true; });
        RCLCPP_DEBUG(ros_node_->get_logger(), "Lock released");
    }
    //lock gazebo's next step
    commands_applied = false;
    RCLCPP_DEBUG(ros_node_->get_logger(), "Exiting before_update");
}

// Callback connected to world update step end event
void ModelPlugin::on_update()
{
    step_number_++;

    // Fill link_states to publish them to ROS
    last_letter_msgs::msg::LinkStates link_states;
    for (unsigned int j = 0; j < model_->GetChildCount(); ++j) // Loop over children links
    {
        auto link = boost::dynamic_pointer_cast<gazebo::physics::Link>(model_->GetChild(j));
        if (link)
        {
            // Store name
            link_states.name.push_back(link->GetScopedName());
            // Store pose
            auto pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(link->WorldPose());
            link_states.pose.push_back(pose);
            // Store velocity
            geometry_msgs::msg::Twist velocity;
            velocity.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(link->WorldLinearVel());
            velocity.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(link->WorldAngularVel());
            link_states.velocity.push_back(velocity);
            // Store acceleration
            geometry_msgs::msg::Twist acceleration;
            acceleration.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(link->WorldLinearAccel());
            acceleration.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(link->WorldAngularAccel());
            link_states.acceleration.push_back(acceleration);
        }
    }
    // Stamp it
    link_states.header.stamp = ros_node_->now();

    pub_state_->publish(link_states);
}

// Register this plugin with the Gazebo
GZ_REGISTER_MODEL_PLUGIN(ModelPlugin)
