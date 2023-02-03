#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include <last_letter_msgs/msg/sim_commands.hpp>
#include <last_letter_msgs/msg/link_states.hpp>

class ModelPlugin : public gazebo::ModelPlugin
{
public:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    // Pointer to the model

private:
    gazebo::physics::ModelPtr model_;
    std::string model_name_;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection_end_;
    gazebo::event::ConnectionPtr before_update_connection_;

    gazebo_ros::Node::SharedPtr ros_node_; //  A node use for ROS2 transport
    sdf::ElementPtr plugin_sdf_;

    // // ROS publisher
    rclcpp::Publisher<last_letter_msgs::msg::LinkStates>::SharedPtr pub_state_;

    /// ROS subscriber
    rclcpp::Subscription<last_letter_msgs::msg::SimCommands>::SharedPtr commands_sub_;

    // ros::ServiceServer apply_wrenches_server; // Ros services
    // ros::CallbackQueue wrenches_rosQueue;     ///  A ROS callbackqueue that helps process messages

    ///  A thread the keeps running the rosQueue
    // std::thread rosQueueThread;

    // mutex and cond_variable
    std::condition_variable cv;
    std::mutex m;

    // last_letter_2_msgs::link_states base_link_states;
    // last_letter_2_msgs::link_states airfoil_states[10];
    // last_letter_2_msgs::link_states motor_states[10];
    // last_letter_2_msgs::model_states model_states;
    // ignition::math::Vector3d relLinVel;
    // ignition::math::Vector3d rotation;
    // ignition::math::Vector3d relAngVel;
    // ignition::math::Vector3d relLinAccel;
    // ignition::math::Vector3d position;
    // ignition::math::Vector3d force, torque;
    // std::string link_name, joint_name;

    // char name_temp[30];
    // int num_wings, num_motors;
    // int i, thread_rate, thread_rate_multiplier, step_number;
    int step_number_{0};
    // float omega;
    // float camera_angle, laser_angle;
    // int reset_sim_chan, camera_angle_chan, laser_angle_chan;
    void Load();
    void model_state_init();
    void QueueThread();
    // template <class Container>
    // void split_string(const std::string &c_name, Container &cont, char delim = ' ');
    void applied_wrenches_cb(const last_letter_msgs::msg::SimCommands::ConstSharedPtr msg);
    // void manageChan(const last_letter_2_msgs::channels::ConstPtr &channels);
    void before_update();
    void on_update();
};