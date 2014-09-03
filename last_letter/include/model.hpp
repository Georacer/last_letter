//Physics model node
//Performs propagation of the dynamic and kinematic model

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <cstdlib>
#include <math.h>

#include "modelPlaneLib.hpp"