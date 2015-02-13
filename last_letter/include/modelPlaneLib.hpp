//Library header for the ModelPlane class

#ifndef modelPlaneLib_include
#define modelPlaneLib_include

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <cstdlib>
#include <math.h>

#include "mathutils/mathutils.hpp"
#include "uav_utils/uav_utils.hpp"

//Include custom messages
#include "last_letter/SimStates.h"
#include "last_letter/SimPWM.h"
#include "last_letter/Geoid.h"
#include "last_letter/Environment.h"


//Forward class declarations
class ModelPlane;
class Kinematics;
class Dynamics;
class Integrator;
class Aerodynamics;
class Gravity;
class Propulsion;
class GroundReaction;
class Airdata;

#include "kinematicsLib.hpp" //Kinematics related declarations
#include "dynamicsLib.hpp" //Dynamics container class declarations
#include "aerodynamicsLib.hpp" //Aerodynamics related declarations
#include "gravityLib.hpp" //Gravity related declarations
#include "groundReactionLib.hpp" //Ground reactions related declarations
#include "propulsionLib.hpp" //Propulsion related declarations
#include "coreLib.hpp" //Main class and other generic classes

#endif