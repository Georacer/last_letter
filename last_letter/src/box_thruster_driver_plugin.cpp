#include <gazebo/gazebo_client.hh>    //gazebo version >6
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
//#include <ignition/math/Vector3.hh>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <boost/bind.hpp>
////#include <boost/shared_ptr.hpp>
#include <gazebo_plugins/PubQueue.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo
{
class Thruster_plugin : public ModelPlugin
{
	// Pointer to the model
private: physics::ModelPtr model;

// Pointer to the update event connection
event::ConnectionPtr updateConnectionStart;  //A class that encapsulates a connection

///  A node use for ROS transport
ros::NodeHandle* rosNode;

///  A ROS subscriber
ros::Subscriber rosSub;

///  A ROS callbackqueue that helps process messages
ros::CallbackQueue rosQueue;

///  A thread the keeps running the rosQueue
std::thread rosQueueThread;

double theta_d_x=0;
double theta_d_y=0;

public:
Thruster_plugin() : ModelPlugin()   //constructor
{

}

void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) //Called when a Plugin is first created,
{														  //and after the World has been loaded.Νot be blocking.
	this->model=_model;

	ROS_INFO("Thruster_plugin just started");

	this->rosNode = new ros::NodeHandle;   //Create a ros node for transport

	while(!this->rosNode->ok()){
		ROS_INFO("Waiting for node to rise");
	}
//	Create a named topic, and subscribe to it. /force
	ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
			"/last_letter/motor_angle", 1, boost::bind(&Thruster_plugin::OnRosMsg, this, _1),
			ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);

	// Spin up the queue helper thread.
	this->rosQueueThread =
			std::thread(std::bind(&Thruster_plugin::QueueThread, this));

	//Connect a callback to the world update start signal.
	this->updateConnectionStart = event::Events::ConnectWorldUpdateBegin(std::bind(&Thruster_plugin::OnPhysicsStepStart, this));
}

void OnRosMsg(const geometry_msgs::Vector3::ConstPtr& _msg)
{
	printf("change joint position at x=%f y=%f deg\n",_msg->x,_msg->y);
	theta_d_x=_msg->x;
	theta_d_y=_msg->y;


}

///  ROS helper function that processes messages
void QueueThread()
{
	static const double timeout = 0.01;
	ROS_INFO(" i am in QueueThread now\n");
	while (this->rosNode->ok()) {
//		this->rosQueue.callAvailable(ros::WallDuration(timeout));
		this->rosQueue.callAvailable();
	}
}

void OnPhysicsStepStart()
{

	model->GetJoint("flap_joint_x")->SetPosition(0,theta_d_x,true);
	model->GetJoint("flap_joint_y")->SetPosition(0,theta_d_y,true);
}


};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Thruster_plugin)
}
