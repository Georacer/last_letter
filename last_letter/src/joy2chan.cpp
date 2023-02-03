#include <last_letter/joy2chan.hpp>

JoystickNode::JoystickNode() : Node("joystick_node")
{
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        1,
        std::bind(&JoystickNode::joy2chan, this, std::placeholders::_1));
    pwm_pub_ = this->create_publisher<last_letter_msgs::msg::SimPWM>("rawPWM", 1);

    this->declare_parameter("throws");
    rclcpp::Parameter throws_param = this->get_parameter("throws");
    throwIndex_ = throws_param.as_double_array();

    this->declare_parameter("axes");
    rclcpp::Parameter axis_param = this->get_parameter("axes");
    axisIndex_ = axis_param.as_integer_array();

    this->declare_parameter("buttons");
    rclcpp::Parameter buttons_param = this->get_parameter("buttons");
    buttonIndex_ = buttons_param.as_integer_array();

    this->declare_parameter("mixerid");
    rclcpp::Parameter mixerid_param = this->get_parameter("mixerid");
    mixerid_ = mixerid_param.as_int();
}

JoystickNode::~JoystickNode()
{
}

void JoystickNode::joy2chan(const sensor_msgs::msg::Joy::SharedPtr joyMsgPtr)
{
    last_letter_msgs::msg::SimPWM channels;
    double input[11];
    int i;
    for (i = 0; i < 11; i++)
    {
        if (axisIndex_[i] != -1)
        { // if an axis is assigned in this channel
            input[i] = 1.0 / throwIndex_[i] * joyMsgPtr->axes[axisIndex_[i]];
        }
        else if (buttonIndex_[i] != -1)
        {
            input[i] = 1.0 / throwIndex_[i] * joyMsgPtr->buttons[buttonIndex_[i]];
        }
        else
        {
            input[i] = 0.0;
        }
    }

    channels = mixer(input, mixerid_);
    for (i = 0; i < 11; i++) // Cap channel limits
    {
        if (channels.value[i] < 1000)
            channels.value[i] = 1000;
        if (channels.value[i] > 2100)
            channels.value[i] = 2100;
    }

    channels.header.stamp = this->now();
    pwm_pub_->publish(channels);
}

// Mixer function
last_letter_msgs::msg::SimPWM JoystickNode::mixer(double *input, int mixerid_)
{
    last_letter_msgs::msg::SimPWM channels;
    int i;
    switch (mixerid_)
    {
    case 0: // No mixing applied
        for (i = 0; i < 11; i++)
        {
            channels.value[i] = (unsigned int)(input[i] * 500 + 1500);
        }
        return channels;
    case 1:                                                              // Airplane mixing
        channels.value[0] = (unsigned int)(input[0] * 500 + 1500);       // Aileron channel
        channels.value[1] = (unsigned int)(input[1] * 500 + 1500);       // Elevator channel
        channels.value[2] = (unsigned int)((input[2] + 1) * 500 + 1000); // Throttle channel
        channels.value[3] = (unsigned int)(input[3] * 500 + 1500);       // Rudder channel
        channels.value[4] = (unsigned int)(input[4] * 500 + 1500);       // Steering gear channel
        channels.value[5] = (unsigned int)(input[5] * 1000 + 1000);      // Breaks channel
        channels.value[6] = (unsigned int)(input[6] * 500 + 1500);       // Generic channel
        channels.value[7] = (unsigned int)(input[7] * 500 + 1500);       // Generic channel
        channels.value[8] = (unsigned int)(input[8] * 500 + 1500);       // Generic channel
        channels.value[9] = (unsigned int)(input[9] * 1000 + 1000);      // Reset channel
        channels.value[10] = (unsigned int)(input[10] * 500 + 1500);     // Generic channel
        return channels;
    case 2: // Quadrotor mixing
        channels.value[0] = (unsigned int)(500 * ((input[2] + 1) - 0.11 * input[0] + 0.11 * input[1] - 0.11 * input[3]) + 1000);
        channels.value[1] = (unsigned int)(500 * ((input[2] + 1) + 0.11 * input[0] + 0.11 * input[1] + 0.11 * input[3]) + 1000);
        channels.value[2] = (unsigned int)(500 * ((input[2] + 1) - 0.11 * input[0] - 0.11 * input[1] + 0.11 * input[3]) + 1000);
        channels.value[3] = (unsigned int)(500 * ((input[2] + 1) + 0.11 * input[0] - 0.11 * input[1] - 0.11 * input[3]) + 1000);
        channels.value[4] = (unsigned int)(input[4] * 500 + 1500);    // Generic channel
        channels.value[5] = (unsigned int)(input[5] * 500 + 1500);    // Generic channel
        channels.value[6] = (unsigned int)(input[6] * 500 + 1500);    // Generic channel
        channels.value[7] = (unsigned int)(input[7] * 500 + 1500);    // Generic channel
        channels.value[8] = (unsigned int)(input[8] * 500 + 1500);    // Generic channel
        channels.value[9] = (unsigned int)(input[9] * 1000 + 1000);   // Reset channel
        channels.value[10] = (unsigned int)(input[10] * 1000 + 1000); // Generic channel
        return channels;
    case 3:                                                                                                                      // Firefly Y6 mixing
        channels.value[0] = (unsigned int)(500 * ((input[2] + 1) - 0.11 * input[0] + 0.11 * input[1] - 0.11 * input[3]) + 1000); // top-right motor channel
        channels.value[1] = (unsigned int)(500 * ((input[2] + 1) - 0.11 * input[0] + 0.11 * input[1] + 0.11 * input[3]) + 1000); // bottom-right motor channel
        channels.value[2] = (unsigned int)(500 * ((input[2] + 1) + 0.00 * input[0] - 0.11 * input[1] - 0.11 * input[3]) + 1000); // top-rear motor channel
        channels.value[3] = (unsigned int)(500 * ((input[2] + 1) + 0.00 * input[0] - 0.11 * input[1] + 0.11 * input[3]) + 1000); // bottom-rear motor channel
        channels.value[4] = (unsigned int)(500 * ((input[2] + 1) + 0.11 * input[0] + 0.11 * input[1] - 0.11 * input[3]) + 1000); // top-left motor channel
        channels.value[5] = (unsigned int)(500 * ((input[2] + 1) + 0.11 * input[0] + 0.11 * input[1] + 0.11 * input[3]) + 1000); // bottom-left channel
        channels.value[6] = (unsigned int)(-input[3] * 500 + 1500);                                                              // steering wheel channel
        channels.value[7] = (unsigned int)(input[9] * 1000 + 1000);                                                              // reset channel
        channels.value[8] = (unsigned int)(input[0] * 500 + 1500);                                                               // aileron channel
        channels.value[9] = (unsigned int)(input[1] * 500 + 1500);                                                               // elevator channel
        channels.value[10] = (unsigned int)(input[6] * 500 + 1500);                                                              // motor gimbal channel
        return channels;
    case 4:                                                                      // Testing a VTOL
        channels.value[0] = (unsigned int)((input[0]) * 500 + 1500);             // Aileron
        channels.value[1] = (unsigned int)((input[1]) * 500 + 1500);             // Elevator
        channels.value[2] = (unsigned int)((input[2]) * 1000 + 1000);            // Throttle
        channels.value[3] = (unsigned int)((input[3]) * 500 + 1500);             // Rudder
        channels.value[4] = (unsigned int)((input[4] + input[8]) * 1000 + 1000); // Motor 1
        channels.value[5] = (unsigned int)((input[5] + input[8]) * 1000 + 1000); // Motor 2
        channels.value[6] = (unsigned int)((input[6] + input[8]) * 1000 + 1000); // Motor 3
        channels.value[7] = (unsigned int)((input[7] + input[8]) * 1000 + 1000); // Motor 4
        channels.value[8] = (unsigned int)(input[8] * 500 + 1500);               // Generic channel
        channels.value[9] = (unsigned int)(input[9] * 1000 + 1000);              // Pause channel
        channels.value[10] = (unsigned int)(input[10] * 500 + 1500);             // Generic channel
        return channels;
    case 5:                                                          // PX4 SITL mixing
        channels.value[0] = (unsigned int)((input[0]) * 500 + 1500); // Roll
        channels.value[1] = (unsigned int)((input[1]) * 500 + 1500); // Pitch
        channels.value[2] = (unsigned int)((input[2]) * 500 + 1500); // Throttle
        channels.value[3] = (unsigned int)((input[3]) * 500 + 1500); // Yaw
        toggler_arm_.read_input(input[4]);
        channels.value[4] = (unsigned int)(toggler_arm_.get_state_normalized() * 1000 + 1000); // Button 1
        toggler_vtol_.read_input(input[5]);
        channels.value[5] = (unsigned int)(toggler_vtol_.get_state_normalized() * 1000 + 1000); // Button 2
        toggler_mode_.read_input(input[6]);
        channels.value[6] = (unsigned int)(toggler_mode_.get_state_normalized() * 1000 + 1000); // Button 3
        toggler_rtl_.read_input(input[7]);
        channels.value[7] = (unsigned int)(toggler_rtl_.get_state_normalized() * 1000 + 1000); // Button 4
        channels.value[8] = (unsigned int)(input[8] * 500 + 1500);                             // Generic channel
        channels.value[9] = (unsigned int)(input[9] * 1000 + 1000);                            // Pause channel
        channels.value[10] = (unsigned int)(input[10] * 500 + 1500);                           // Generic channel
        return channels;
    default:
        RCLCPP_FATAL(this->get_logger(), "Invalid parameter for -mixerid_- in param server!");
        rclcpp::shutdown();
        return channels;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickNode>());
    rclcpp::shutdown();
    return 0;
}
