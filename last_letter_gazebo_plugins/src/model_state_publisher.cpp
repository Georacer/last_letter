#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include "gazebo_msgs/ModelState.h"
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

//#include "math/Pose.hh"

namespace gazebo
{
  class modelStateBroadcaster : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&modelStateBroadcaster::OnUpdate, this, _1));

      this->rosPub = this->rosHandle.advertise<gazebo_msgs::ModelState>("/" + this->model->GetName() + "/modelState",100); //model states publisher

      ROS_INFO("modelStateBroadcaster plugin initialized");
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // Read the model state
      this->modelPose = this->model->GetWorldPose();
        // Velocities required in the body frame
      this->modelVelLin = this->model->GetRelativeLinearVel();
      this->modelVelAng = this->model->GetRelativeAngularVel();

      this->modelState.model_name = this->model->GetName();
      this->modelState.pose.position.x = this->modelPose.pos.x;
      this->modelState.pose.position.y = this->modelPose.pos.y;
      this->modelState.pose.position.z = this->modelPose.pos.z;
      this->modelState.pose.orientation.x = this->modelPose.rot.x;
      this->modelState.pose.orientation.y = this->modelPose.rot.y;
      this->modelState.pose.orientation.z = this->modelPose.rot.z;
      this->modelState.pose.orientation.w = this->modelPose.rot.w;
      this->modelState.twist.linear.x = this->modelVelLin.x;
      this->modelState.twist.linear.y = this->modelVelLin.y;
      this->modelState.twist.linear.z = this->modelVelLin.z;
      this->modelState.twist.angular.x = this->modelVelAng.x;
      this->modelState.twist.angular.y = this->modelVelAng.y;
      this->modelState.twist.angular.z = this->modelVelAng.z;
      this->modelState.reference_frame = "world";

      this->rosPub.publish(this->modelState);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS related variables
    private:
      ros::NodeHandle rosHandle;
      ros::Publisher rosPub;
      gazebo_msgs::ModelState modelState;
      math::Pose modelPose;
      math::Vector3 modelVelLin, modelVelAng;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(modelStateBroadcaster)
}
