#include <gazebo/gazebo_client.hh>    //gazebo version >6
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/bullet/bullet_inc.h>
#include <gazebo/physics/bullet/BulletTypes.hh>
#include "gazebo/physics/Link.hh"
#include "gazebo/util/system.hh"
#include <gazebo/physics/bullet/BulletPhysics.hh>
#include <ignition/math/Vector3.hh>
#include <boost/bind.hpp>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <math.h>       /* pow */

namespace gazebo
{
class Box_prop_plugin : public ModelPlugin
{
	// Pointer to the model
private: physics::ModelPtr model;

// Pointer to the update event connection
event::ConnectionPtr updateConnection;  //A class that encapsulates a connection

///  A node use for ROS transport
ros::NodeHandle* rosNode;

ignition::math::Vector3d force;
//std_msgs::Float64 ang_prop_vel;
//geometry_msgs::Vector3 ang_vel;
float c2=10;

public:
Box_prop_plugin() : ModelPlugin()   //constructor
{

}

void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) //Called when a Plugin is first created,
{														  //and after the World has been loaded.Νot be blocking.
	this->model=_model;

	ROS_INFO("Box_prop_plugin just started");

	this->rosNode = new ros::NodeHandle;   //Create a ros node for transport

	while(!this->rosNode->ok()){
		ROS_INFO("Waiting for node to rise");
	}

	//Connect a callback to the world update start signal.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Box_prop_plugin::OnUpdate, this));

}

void OnUpdate()
{
	printf("prop plugin now\n");
	//publish ang_prop_vel
	double vel;
	vel=model->GetJoint("boxToArm")->GetVelocity(0);
	force[2]=c2*pow(vel,2);			//prop force calculation
	physics::LinkPtr my_link = model->GetLink("arm");
	physics::BulletLinkPtr bullet_Link;

//	boost::shared_ptr<physics::BulletLinkPtr>
//	bullet_Link = boost::dynamic_pointer_cast<physics::BulletLink>(my_link);
	bullet_Link = boost::dynamic_pointer_cast<physics::BulletLink>(my_link);
	btRigidBody * btLink = bullet_Link->GetBulletLink();
	btVector3 force, rel_pos;
	force.setX(force[0]);
	force.setY(force[1]);
	force.setZ(force[2]);
	rel_pos.setX(0);
	rel_pos.setY(0);
	rel_pos.setZ(0);
	btLink->applyForce(force, rel_pos);

//	model->GetLink("arm")->AddRelativeForce(force);


	//	ang_prop_vel.data=vel;
	//	this->pub_.publish(ang_prop_vel);
	////
	////	Set force, sub for force topic
	//	printf("force[0,1,2]=[%f, %f, %f]\n",force[0],force[1],force[2]);
	//
	//	physics::Link_V children_vec = model->GetLinks();
	//	int c_num = children_vec.size();
	//
	//	for (int i=0; i<c_num; i++)
	//	{
	//		printf("Plugin: Child found: %s\n", children_vec[i]->GetName().c_str());
	//	}
	//
	//	model->GetLink("arm")->AddRelativeForce(force);
	//	printf("prop plugin end\n");
}



};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Box_prop_plugin)
}
