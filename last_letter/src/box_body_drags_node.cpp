#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include <geometry_msgs/Vector3.h>



int sgn(double v) {
	if (v < 0) return -1;
	if (v > 0) return 1;
	return 0;
}

class IOClass
{
public:
	ros::NodeHandle n;

	geometry_msgs::Vector3 rel_velocity;
	float F1d[3];
	float b=1;
	float rel_vel[3];
	geometry_msgs::Vector3 F1;

	ros::Publisher chatter_pub;
	ros::Subscriber sub;
//public:
	IOClass()
	{
		ROS_INFO("Starting constructor");
		chatter_pub = n.advertise<geometry_msgs::Vector3>("/last_letter/body/drags", 1);
		sub = n.subscribe("/last_letter/body/rel_velocity", 1000, &IOClass::callback, this);
		ROS_INFO("Done with constructor");
	}

	void callback(geometry_msgs::Vector3 msg)
	{
		rel_velocity = msg;

		rel_vel[0]=rel_velocity.x;
		rel_vel[1]=rel_velocity.y;
		rel_vel[2]=rel_velocity.z;

		for(int i=0; i<3; i++){								//F1d calculation
			F1d[i]=-b*pow(rel_vel[i],2)*sgn(rel_vel[i]);
		}

		printf("F1d.x=%f F1d.y=%f F1d.z=%f\n", F1d[0],F1d[1],F1d[2]);
		F1.x=F1d[0];
		F1.y=F1d[1];
		F1.z=F1d[2];

		this->chatter_pub.publish(F1);
	}

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "box_body_drags_node");
	printf("start box_body_drags_node\n");

	IOClass io_obj;
	printf("object has been done\n");
//	ros::Rate loop_rate(1000);


	ros::spin();

	return 0;
}
