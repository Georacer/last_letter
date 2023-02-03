#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <last_letter/sensor.hpp>
#include <gazebo_msgs/msg/link_states.hpp>

using std::placeholders::_1;

class SensorNode : public rclcpp::Node
{
public:
    SensorNode()
        : Node("sensor_node")
    {
        sub_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(
            "/link_states", 10, std::bind(&SensorNode::topic_callback, this, _1));
        pub_ = create_publisher<std_msgs::msg::String>("sensor_output", 10);
    }

private:
    void topic_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I received: %d links", msg->name.size());
        last_msg_ = *msg;

        // Publish local ROS debug message
        auto message = std_msgs::msg::String();
        message.data = msg->name[0];
        pub_->publish(message);
    }
    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    gazebo_msgs::msg::LinkStates last_msg_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNode>());
    rclcpp::shutdown();
    return 0;
}