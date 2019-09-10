//Main executable
//Instantiates a UavModel object and commands its step

// #include <cstdlib>
// #include <math.h>

// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Wrench.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <geometry_msgs/Vector3.h>
// #include <geometry_msgs/Quaternion.h>
#include <rosgraph_msgs/Clock.h>

#include "uav_model_ros.hpp"

UavModelWrapper * uav;

void stepCallback(const rosgraph_msgs::Clock time);
