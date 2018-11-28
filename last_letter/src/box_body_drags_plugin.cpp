#include <gazebo/gazebo_client.hh>    //gazebo version >6
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <boost/bind.hpp>
//#include <boost/shared_ptr.hpp>
#include <gazebo_plugins/PubQueue.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo
{
class box_body_drags_plugin : public ModelPlugin
{
	// Pointer to the model
private: physics::ModelPtr model;

// Pointer to the update event connection
event::ConnectionPtr updateConnectionStart;  //A class that encapsulates a connection
event::ConnectionPtr updateConnectionEnd;

///  A node use for ROS transport
ros::NodeHandle* rosNode;

///  A ROS subscriber
ros::Subscriber rosSub;

// ROS publisher
ros::Publisher pub_;

///  A ROS callbackqueue that helps process messages
ros::CallbackQueue rosQueue;

///  A thread the keeps running the rosQueue
std::thread rosQueueThread;

ignition::math::Vector3d force;

public:
box_body_drags_plugin() : ModelPlugin()   //constructor
{

}

void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) //Called when a Plugin is first created,
{														  //and after the World has been loaded.Νot be blocking.
	this->model=_model;

	ROS_INFO("box_body_drags_plugin just started");

	this->rosNode = new ros::NodeHandle;   //Create a ros node for transport

	while(!this->rosNode->ok()){
		ROS_INFO("Waiting for node to rise");
	}
//	Create a named topic, and subscribe to it. /force
	ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
			"/last_letter/body/drags", 1, boost::bind(&box_body_drags_plugin::OnRosMsg, this, _1),
			ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);

	// Spin up the queue helper thread.
	this->rosQueueThread =
			std::thread(std::bind(&box_body_drags_plugin::QueueThread, this));

	//Connect a callback to the world update start signal.
	this->updateConnectionStart = event::Events::ConnectWorldUpdateBegin(std::bind(&box_body_drags_plugin::OnPhysicsStepStart, this));
	this->updateConnectionEnd = event::Events::ConnectWorldUpdateEnd(std::bind(&box_body_drags_plugin::OnPhysicsStepEnd, this));

	// Publish code
	this->pub_ = this->rosNode->advertise<geometry_msgs::Vector3>("/last_letter/body/rel_velocity", 100);
}

void OnRosMsg(const geometry_msgs::Vector3::ConstPtr& _msg)
{
	force[0]=_msg->x;
	force[1]=_msg->y;
	force[2]=_msg->z;
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
	//Set force, sub for force topic
	model->GetLink("base_link")->AddRelativeForce(force);
	printf("box plugin now\n");
}

void OnPhysicsStepEnd()
{
	//Publish on /motor_multi_plugin/box/rel_velocity topic
	geometry_msgs::Vector3 velocity;
	ignition::math::Vector3d rel_vel;
	rel_vel=model->RelativeLinearVel();
	velocity.x=rel_vel.X();
	velocity.y=rel_vel.Y();
	velocity.z=rel_vel.Z();
	this->pub_.publish(velocity);
}

};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(box_body_drags_plugin)
}
