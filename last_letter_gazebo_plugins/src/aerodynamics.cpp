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
      this->linkINS = this->model->GetLink("INS");
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

      math::Quaternion BodyQuat(0,1,0,0); // Rotation quaternion from gazebo body frame to aerospace body frame
      math::Vector3 CoL(-0.02, 0.00, 0.05); // CoL location

      math::Vector3 inpForce(wrench.force.x, wrench.force.y, wrench.force.z); // in aerodpace body frame
      math::Vector3 inpTorque(wrench.torque.x, wrench.torque.y, wrench.torque.z); // in aerospace body frame

      math::Vector3 newForce = BodyQuat.GetInverse()*inpForce; // in gazebo frame
      math::Vector3 newTorque = BodyQuat.GetInverse()*inpTorque + CoL.Cross(newForce);// in gazebo frame

      // ROS_INFO("aerodynamics.cpp plugin: applying new force (xyz): %g\t%g\t%g",newForce.x, newForce.y, newForce.z);
      this->linkFuse->AddRelativeForce(newForce);

      // ROS_INFO("aerodynamics.cpp plugin: applying new torque (xyz): %g\t%g\t%g",newTorque.x, newTorque.y, newTorque.z);
      this->linkFuse->AddRelativeTorque(newTorque);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to INS link
    physics::LinkPtr linkINS, linkFuse;

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
