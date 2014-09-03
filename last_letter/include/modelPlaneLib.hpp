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

#include "mathutils/mathutils.hpp"
//#include "uav_utils/uav_utils.hpp"

#include "last_letter/SimStates.h"
#include "last_letter/SimPWM.h"
#include "last_letter/Environment.h"

// Top class for UAV object : First pass
class ModelPlane;
class Kinematics;
class Dynamics;
class Integrator;
class Aerodynamics;
class Gravity;
class Propulsion;
class GroundReaction;
class Airdata;

#include "kinematicsLib.hpp"
#include "dynamicsLib.hpp"
#include "gravityLib.hpp"
#include "groundReactionLib.hpp"
#include "propulsionLib.hpp"
#include "coreLib.hpp"
