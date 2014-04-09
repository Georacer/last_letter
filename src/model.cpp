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


class ModelPlane
{
	nav_msgs::Odometry kinematics;
	geometry_msgs::WrenchStamped dynamics;
	ros::NodeHandle nh;
	ros::Subscriber subInp;
	ros::Publisher pubState;
	ros::Publisher pubWrench;
	ros::Time tprev;
	//ros::Duration timeDiff;
	double dt;
	//last_letter::inputs input;
	double input[4];
	ros::ServiceClient forceClient, torqueClient;
	
	void getInput(last_letter::inputs inputMsg)
	{
		//input = inputMsg.inputs; //does not work
		input[0] = inputMsg.inputs[0];
		input[1] = inputMsg.inputs[1];
		input[2] = inputMsg.inputs[2];
		input[3] = inputMsg.inputs[3];
	}
	
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
		tf::Vector3 tempVect;
		//tf::Quaternion quat = kinematics.pose.pose.orientation;
		tf::Quaternion quat = tf::Quaternion(kinematics.pose.pose.orientation.x, kinematics.pose.pose.orientation.y, kinematics.pose.pose.orientation.z, kinematics.pose.pose.orientation.w);
		//quat.x = kinematics.pose.pose.orientation->x;
		//quat.y = &kinematics.pose.pose.orientation.y;
		//quat.z = (&kinematics).pose.pose.orientation.z;
		//quat.w = kinematics.pose.pose.orientation.w;
		
		//create angles, linear speeds, angular speeds
		double phi, theta, psi;
		tf::Matrix3x3(quat).getEulerYPR(psi, theta, phi);
		double u = kinematics.twist.twist.linear.x;
		double v = kinematics.twist.twist.linear.y;
		double w = kinematics.twist.twist.linear.z;
		double p = kinematics.twist.twist.angular.x;
		double q = kinematics.twist.twist.angular.y;
		double r = kinematics.twist.twist.angular.z;
		
		//create position derivatives
		tf::Matrix3x3 R = tf::Matrix3x3(quat);
		double pndot, pedot, pddot;
		tempVect = tf::Vector3(kinematics.twist.twist.linear.x, kinematics.twist.twist.linear.y, kinematics.twist.twist.linear.z);
		tf::Vector3 posDot = R*tempVect;
		//tf::Vector3 posDot = R*kinematics.twist.twist.linear;
		
		//create speed derivatives
		double mass;
		ros::param::getCached("airframe/m",mass);
		tempVect = tf::Vector3(dynamics.wrench.force.x, dynamics.wrench.force.y, dynamics.wrench.force.z);
		tf::Vector3 linearForces = tempVect*(1.0/mass); //linear acceleration
		tf::Vector3 corriolisForces = tf::Vector3(r*v-q*w, p*w-r*u, q*u-p*v);
		tf::Vector3 speedDot = linearForces + corriolisForces;
		
		//create angular derivatives
		tempVect = tf::Vector3(kinematics.twist.twist.angular.x, kinematics.twist.twist.angular.y, kinematics.twist.twist.angular.z);
		tf::Vector3 angleDot = tf::Matrix3x3(1, sin(phi)*tan(theta), cos(phi)*tan(theta), 0, cos(phi), -sin(phi), 0, sin(phi)/cos(theta), cos(phi)/cos(theta))*tempVect;
		
		//create rate derivatives
		double j_x, j_y, j_z, j_xz;
		ros::param::getCached("airframe/j_x",j_x);
		ros::param::getCached("airframe/j_y",j_y);
		ros::param::getCached("airframe/j_z",j_z);
		ros::param::getCached("airframe/j_xz",j_xz);
		double G = j_x*j_z-pow(j_xz,2);
		double G1 = j_xz*(j_x-j_y+j_z)/G;
		double G2 = (j_z*(j_z-j_y)+pow(j_xz,2))/G;
		double G3 = j_z/G;
		double G4 = j_xz/G;
		double G5 = (j_z-j_x)/j_y;
		double G6 = j_xz/j_y;
		double G7 = ((j_x-j_y)*j_x+pow(j_xz,2))/G;
		double G8 = j_x/G;
		double l = dynamics.wrench.torque.x;
		double m = dynamics.wrench.torque.y;
		double n = dynamics.wrench.torque.z;				
		tf::Vector3 rate1 = tf::Vector3(G1*p*q-G2*q*r , G5*p*r-G6*(pow(p,2)-pow(r,2)) , G7*p*q-G1*q*r);
		tf::Vector3 rate2 = tf::Vector3(G3*l+G4*n , 1/j_y*m , G4*l+G8*n);
		tf::Vector3 rateDot = rate1 + rate2;
		
		//integrate quantities using forward Euler
		kinematics.pose.pose.position.x = kinematics.pose.pose.position.x + posDot.x()*dt;
		kinematics.pose.pose.position.y = kinematics.pose.pose.position.y + posDot.y()*dt;
		kinematics.pose.pose.position.z = kinematics.pose.pose.position.z + posDot.z()*dt;
		
		kinematics.twist.twist.linear.x = kinematics.twist.twist.linear.x + speedDot.x()*dt;
		kinematics.twist.twist.linear.y = kinematics.twist.twist.linear.y + speedDot.y()*dt;
		kinematics.twist.twist.linear.z = kinematics.twist.twist.linear.z + speedDot.z()*dt;
		
		tf::Vector3 angles = tf::Vector3(phi, theta, psi);
		angles = angles + angleDot*dt;
		quat.setEuler(angles.z(), angles.y(), angles.x());
		kinematics.pose.pose.orientation.x = quat[0];
		kinematics.pose.pose.orientation.y = quat[1];
		kinematics.pose.pose.orientation.z = quat[2];
		kinematics.pose.pose.orientation.w = quat[3];
		//kinematics.pose.pose.orientation = quat;
		
		kinematics.twist.twist.angular.x = kinematics.twist.twist.angular.x + rateDot.x()*dt;
		kinematics.twist.twist.angular.y = kinematics.twist.twist.angular.y + rateDot.y()*dt;		
		kinematics.twist.twist.angular.z = kinematics.twist.twist.angular.z + rateDot.z()*dt;
		//kinematics.twist.twist.angular = kinematics.twist.twist.angular + rateDot*dt;
	}
	
	public:
	ModelPlane (void)
	{
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
		kinematics.twist.twist.linear.x = 0;
		kinematics.twist.twist.linear.y = 0;
		kinematics.twist.twist.linear.z = 0;
		kinematics.twist.twist.angular.x = 0;
		kinematics.twist.twist.angular.y = 0;
		kinematics.twist.twist.angular.z = 0;
		input[0] = 0;
		input[1] = 0;
		input[2] = 0;
		input[3] = 0;
		dynamics.header.frame_id = "boydFrame";
		subInp = nh.subscribe("/sim/input",1,&ModelPlane::getInput, this);
		pubState = nh.advertise<nav_msgs::Odometry>("/sim/states",1000);
		pubWrench = nh.advertise<geometry_msgs::WrenchStamped>("/sim/wrenchStamped",1000);
		forceClient = nh.serviceClient<last_letter::calc_force>("calc_force");
		torqueClient = nh.serviceClient<last_letter::calc_torque>("calc_torque");
		
	}
	
	void step(void)
	{
		dt = (ros::Time::now() - tprev).toSec();
		tprev = ros::Time::now();
		kinematics.header.stamp = tprev;
		
		//calclulate body force/torques
		dynamics.wrench.force = getForce(kinematics, input);
		dynamics.wrench.torque = getTorque(kinematics, input);
		diffEq();
		pubState.publish(kinematics);
		pubWrench.publish(dynamics);
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
	ros::Duration(5).sleep(); //wait for other nodes to get raised
	ModelPlane aerosonde;

	
	while (ros::ok())
	{
		aerosonde.step();
		spinner.sleep();
	}
	
}
