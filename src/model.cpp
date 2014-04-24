#include "ros/ros.h"
#include "last_letter/calc_force.h"
#include "last_letter/calc_torque.h"
#include "last_letter/calc_air_data.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "last_letter/inputs.h"
//#include "LinearMath/btMatrix3x3.h"
#include <cstdlib>
#include <math.h>
#include "mathutils/utls.hpp"

using namespace std;


// Define ModelPlane class
class ModelPlane
{
	public:
	nav_msgs::Odometry kinematics;
	geometry_msgs::WrenchStamped dynamics;
	ros::Subscriber subInp;
	ros::Publisher pubState;
	ros::Publisher pubWrench;
	ros::Time tprev;
	ros::Duration durTemp;
	double dt;
	double input[4];
	ros::ServiceClient forceClient, torqueClient;
	
	geometry_msgs::Vector3 getForce(nav_msgs::Odometry states, double input[4])
	{
		last_letter::calc_force forceService;
		ros::service::waitForService("calc_force",-1);		
		forceService.request.states = kinematics;
		forceService.request.inputs[0] = input[0];
		forceService.request.inputs[1] = input[1];
		forceService.request.inputs[2] = input[2];
		forceService.request.inputs[3] = input[3];
		if (!forceClient.call(forceService)){
			ROS_ERROR("Failed to call service calc_force in %s","model");
		}
		return forceService.response.force;
	}

	geometry_msgs::Vector3 getTorque(nav_msgs::Odometry states, double input[4])
	{
		last_letter::calc_torque torqueService;
		ros::service::waitForService("calc_torque",-1);		
		torqueService.request.states = kinematics;
		torqueService.request.inputs[0] = input[0];		
		torqueService.request.inputs[1] = input[1];
		torqueService.request.inputs[2] = input[2];
		torqueService.request.inputs[3] = input[3];
		if (!torqueClient.call(torqueService)){
			ROS_ERROR("Failed to call service calc_torque in %s","model");
		}
		return torqueService.response.torque;
	}
	
	void diffEq(void)
	{
		//variables declaration
		geometry_msgs::Vector3 tempVect;
		geometry_msgs::Quaternion quat = kinematics.pose.pose.orientation;		
		double Reb[9];
		
		//create transformation matrix
		quat2rotmtx (kinematics.pose.pose.orientation, Reb);			
		
		//create position derivatives
		geometry_msgs::Vector3 posDot = Reb*kinematics.twist.twist.linear;
		
		//create speed derivatives
		double mass;
		ros::param::getCached("airframe/m",mass);
		geometry_msgs::Vector3 linearAcc = (1.0/mass)*dynamics.wrench.force;
		geometry_msgs::Vector3 corriolisAcc;
		vector3_cross(-kinematics.twist.twist.angular, kinematics.twist.twist.linear, &corriolisAcc);
		geometry_msgs::Vector3 speedDot = linearAcc + corriolisAcc;		
		
		//create angular derivatives
		geometry_msgs::Quaternion wquat;
		wquat.w = 1.0;
		wquat.x = kinematics.twist.twist.angular.x*0.5*dt;
		wquat.y = kinematics.twist.twist.angular.y*0.5*dt;
		wquat.z = kinematics.twist.twist.angular.z*0.5*dt;				
		
		//create angular rate derivatives
		double j_x, j_y, j_z, j_xz;
		ros::param::getCached("airframe/j_x",j_x);
		ros::param::getCached("airframe/j_y",j_y);
		ros::param::getCached("airframe/j_z",j_z);
		ros::param::getCached("airframe/j_xz",j_xz);
		double G = j_x*j_z-pow(j_xz,2);	
		double J[9] = {j_x, 0, -j_xz, 0, j_y, 0, -j_xz, 0, j_z};
		double Jinv[9] = {j_z/G, 0, j_xz/G, 0, 1/j_y, 0, j_xz/G, 0, j_x/G};

		vector3_cross(kinematics.twist.twist.angular, J*kinematics.twist.twist.angular, &tempVect);
		tempVect = -tempVect+dynamics.wrench.torque;
		geometry_msgs::Vector3 rateDot = Jinv*tempVect;
		
		//integrate quantities using forward Euler
		kinematics.pose.pose.position.x = kinematics.pose.pose.position.x + posDot.x*dt;
		kinematics.pose.pose.position.y = kinematics.pose.pose.position.y + posDot.y*dt;
		kinematics.pose.pose.position.z = kinematics.pose.pose.position.z + posDot.z*dt;
		
		tempVect = dt*speedDot;
		kinematics.twist.twist.linear = kinematics.twist.twist.linear + tempVect;
		
		quat_product(quat,wquat,&kinematics.pose.pose.orientation);
		quat_normalize(&kinematics.pose.pose.orientation);		
		
		tempVect = dt*rateDot;
		kinematics.twist.twist.angular = kinematics.twist.twist.angular + tempVect;

	}	

	//Constructor
	ModelPlane (ros::NodeHandle n)
	{
		//Initialize states
		kinematics.header.frame_id = "bodyFrame";
		tprev = ros::Time::now();
		kinematics.header.stamp = tprev;
		kinematics.pose.pose.position.x = 0;
		kinematics.pose.pose.position.y = 0;
		kinematics.pose.pose.position.z = -300;
		kinematics.pose.pose.orientation.x = 0;
		kinematics.pose.pose.orientation.y = 0;
		kinematics.pose.pose.orientation.z = 0;
		kinematics.pose.pose.orientation.w = 1;
		kinematics.twist.twist.linear.x = 10;
		kinematics.twist.twist.linear.y = 0;
		kinematics.twist.twist.linear.z = 0;
		kinematics.twist.twist.angular.x = 0;
		kinematics.twist.twist.angular.y = 0;
		kinematics.twist.twist.angular.z = 0;
		input[0] = 0;
		input[1] = 0;
		input[2] = 0;
		input[3] = 0;
		dynamics.header.frame_id = "bodyFrame";
		//Subscribe and advertize
		subInp = n.subscribe("/sim/input",1,&ModelPlane::getInput, this);
		pubState = n.advertise<nav_msgs::Odometry>("/sim/states",1000);
		pubWrench = n.advertise<geometry_msgs::WrenchStamped>("/sim/wrenchStamped",1000);
		//Create service clients
		forceClient = n.serviceClient<last_letter::calc_force>("calc_force");
		torqueClient = n.serviceClient<last_letter::calc_torque>("calc_torque");
		
	}
	
	void step(void)
	{
		durTemp = (ros::Time::now() - tprev);
		dt = durTemp.toSec();
		tprev = ros::Time::now();
		kinematics.header.stamp = tprev;
		
		//calclulate body force/torques
		dynamics.wrench.force = getForce(kinematics, input);
		dynamics.wrench.torque = getTorque(kinematics, input);
		//make a simulation step
		diffEq();
		//publish results
		pubState.publish(kinematics);
		pubWrench.publish(dynamics);
	}
	

	void getInput(last_letter::inputs inputMsg)
	{
		//Convert PPM to -1/1 ranges (0/1 for throttle)
		input[0] = (inputMsg.inputs[0]-1500)/500;
		input[1] = -(inputMsg.inputs[1]-1500)/500;
		input[2] = (inputMsg.inputs[2]-1000)/1000;
		input[3] = -(inputMsg.inputs[3]-1500)/500;
	}
	
};

int main(int argc, char **argv)
{
	int simRate;
	ros::param::getCached("/simRate",simRate); //frame rate in Hz
	
	ros::init(argc, argv, "simNode");
	ros::NodeHandle n;
	ros::Rate spinner(simRate);
	ROS_INFO("simNode up");
	
	ModelPlane aerosonde(n);
	ros::Duration(5).sleep(); //wait for other nodes to get raised	
	aerosonde.tprev = ros::Time::now();
	spinner.sleep();
	
	while (ros::ok())
	{
		aerosonde.step();
		ros::spinOnce();
		spinner.sleep();

		/*if (isnan(aerosonde.kinematics.twist.twist.linear.x))
		{		
			ROS_FATAL("State NAN!");
			break;
		}*/
	}
	
	return 0;
	
}
