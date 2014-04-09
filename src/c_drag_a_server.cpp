#include "ros/ros.h"
#include "last_letter/c_drag_a.h"
#define _USE_MATH_DEFINES
#include "math.h"

bool c_drag_a_callback(last_letter::c_drag_a::Request &req, last_letter::c_drag_a::Response &res)
{
	double alpha = req.alpha;
	double c_drag_p, c_lift_0, c_lift_a, oswald, b, S, AR;
	ros::param::getCached("/airframe/c_drag_p",c_drag_p);
	ros::param::getCached("/airframe/c_lift_0",c_lift_0);
	ros::param::getCached("/airframe/c_lift_a",c_lift_a);
	ros::param::getCached("/world/oswald",oswald);
	ros::param::getCached("/airframe/b",b);
	ros::param::getCached("/airframe/s",S);
	AR = pow(b,2)/S;
	res.c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a*alpha,2)/(M_PI*oswald*AR);

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "c_drag_a_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("c_drag_a",c_drag_a_callback);
	ROS_INFO("c_drag_a_server ready");
	ros::spin();

	return 0;
}
