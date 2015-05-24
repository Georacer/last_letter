#include <ros/ros.h>
#include <last_letter_msgs/SimPWM.h>

ros::WallTime lastStamp;

/////////////////////////
// Subscriber callback //
/////////////////////////
void ctrlCallback(const last_letter_msgs::SimPWM ctrlInput)
{
	lastStamp = ros::WallTime::now();
}


///////////////////
// Main Function //
///////////////////

int main(int argc, char **argv) {

	ros::init(argc, argv, "ctrlInjector");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("ctrlPWM",100, ctrlCallback);
	ros::Publisher pub = n.advertise<last_letter_msgs::SimPWM>("ctrlPWM",1);

	ros::WallDuration counter;
	ros::WallRate rate(1);
	ros::WallTime currTime;

	last_letter_msgs::SimPWM samplePWM;

	lastStamp = ros::WallTime::now();

	ros::WallDuration(3).sleep(); //wait for other nodes to get raised
	ROS_INFO("injector up");

	while (ros::ok())
	{
		ros::spinOnce();
		currTime = ros::WallTime::now();
		counter = currTime - lastStamp;
		if (counter.toSec()>1) {
			pub.publish(samplePWM);
			ROS_INFO("Injected SimPWM topic onto ctrlPWM");
		}
	}

	return 0;

}