#include "ros/ros.h"
#include "last_letter/c_lift_a.h"
#include "math.h"

bool c_lift_a_callback(last_letter::c_lift_a::Request &req, last_letter::c_lift_a::Response &res)
{
	double alpha = req.alpha;
	double M, alpha0, c_lift_0, c_lift_a;
	ros::param::getCached("/airframe/mcoeff",M);
	ros::param::getCached("/airframe/alpha_stall",alpha0);
	ros::param::getCached("/airframe/c_lift_0",c_lift_0);
	ros::param::getCached("/airframe/c_lift_a",c_lift_a);
	
	double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
	double linear = (1-sigmoid) * (c_lift_0 + c_lift_a*alpha); //Lift at small AoA
	double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall
	
	res.c_lift_a = linear+flatPlate;
	
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "c_lift_a_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("c_lift_a",c_lift_a_callback);
	ROS_INFO("c_lift_a_server ready");
	ros::spin();

	return 0;
}
