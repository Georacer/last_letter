#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <cstdlib>

#include <last_letter_msgs/SimPWM.h>

// Joystick input callback
void joy2chan(sensor_msgs::Joy joyMsg);

// Mixer function
last_letter_msgs::SimPWM mixer(double * input, int mixerid);

// Main function
int main(int argc, char **argv);
