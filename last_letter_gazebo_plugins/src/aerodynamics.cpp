#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Wrench.h"
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>


namespace gazebo
{
  class aerodynamics : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      // Store the pointer to the fuselage link
      this->linkFuse = this->model->GetLink("fuselage");

      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&aerodynamics::OnUpdate, this, _1));

      this->rosPub = this->rosHandle.advertise<gazebo_msgs::ModelState>("/" + this->model->GetName() + "/propState",100); //model states publisher
      this->rosSubAero = this->rosHandle.subscribe("/" + this->model->GetName() + "/wrenchAero",1,&aerodynamics::getWrenchAero, this); //get the applied wrench

      ROS_INFO("aerodynamics plugin initialized");
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
    }

    public: void getWrenchAero(const geometry_msgs::Wrench& wrench)
    {
      this->linkFuse->AddRelativeForce(math::Vector3(wrench.force.x, wrench.force.y, wrench.force.z));
      this->linkFuse->AddRelativeTorque(math::Vector3(wrench.torque.x, wrench.torque.y, wrench.torque.z));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to propeller link
    physics::LinkPtr linkFuse;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS related variables
    private:
      ros::NodeHandle rosHandle;
      ros::Publisher rosPub;
      ros::Subscriber rosSubAero;
      gazebo_msgs::ModelState propState;
      math::Pose modelPose;
      math::Vector3 modelVelLin, modelVelAng;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(aerodynamics)
}
