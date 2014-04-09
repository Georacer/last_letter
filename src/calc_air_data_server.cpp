#include "ros/ros.h"
#include "last_letter/calc_air_data.h"


bool calc_air_data_callback(last_letter::calc_air_data::Request &req, last_letter::calc_air_data::Response &res)
{
	double u = req.speeds.x;
	double v = req.speeds.y;
	double w = req.speeds.z;

	double airspeed = sqrt(pow(u,2)+pow(v,2)+pow(w,2));
	double alpha = atan2(w,u);
	double beta = 0;
	if (u==0) {
		if (v==0) {
			beta=0;
		}
		else {
			beta=asin(v/abs(v));
		}
	
	}
	else {
		beta = atan2(v,u);
	}
	
	res.airspeed = airspeed;
	res.alpha = alpha;
	res.beta = beta;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calc_air_data_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("calc_air_data",calc_air_data_callback);
	ROS_INFO("calc_air_data_server ready");
	ros::spin();

	return 0;
}
