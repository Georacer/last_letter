#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <cstdlib>

#include <last_letter_lib/prog_utils.hpp>
#include <last_letter_msgs/msg/sim_pwm.hpp>

using last_letter_lib::programming_utils::TogglerInput;

class JoystickNode : public rclcpp::Node
{
public:
    // Variables
    /////////////

    // Methods
    //////////
    JoystickNode();
    ~JoystickNode();
    void joy2chan(const sensor_msgs::msg::Joy::SharedPtr joyMsgPtr);
    // Mixer function
    last_letter_msgs::msg::SimPWM mixer(double *input, int mixerid);

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<last_letter_msgs::msg::SimPWM>::SharedPtr pwm_pub_;
    std::vector<int64_t> axisIndex_;
    std::vector<int64_t> buttonIndex_;
    std::vector<double> throwIndex_;
    int mixerid_;

    TogglerInput toggler_arm_{2};
    TogglerInput toggler_vtol_{2};
    TogglerInput toggler_mode_{3};
    TogglerInput toggler_rtl_{2};
};
