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
  class engineControl : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      // Store the pointer to the motor shaft
      this->jointAxis = this->model->GetJoint("propeller_shaft");
      // Store teh point to the propeller link
      this->linkProp = this->model->GetLink("propeller");
      this->linkMot = this->model->GetLink("motor");

      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&engineControl::OnUpdate, this, _1));

      this->rosPub = this->rosHandle.advertise<gazebo_msgs::ModelState>("/" + this->model->GetName() + "/propState",100); //model states publisher
      this->rosSubMotor = this->rosHandle.subscribe("/" + this->model->GetName() + "/wrenchMotor",1,&engineControl::getWrenchMotor, this); //get the applied wrench

      ROS_INFO("engineControl plugin initialized");
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {

      // Read the linkProp state
      this->modelPose = this->linkProp->GetWorldPose();
        // Velocities required in the body frame
      this->modelVelLin = this->linkProp->GetRelativeLinearVel();
      this->modelVelAng = this->linkProp->GetRelativeAngularVel();

      this->propState.model_name = this->linkProp->GetName();
      this->propState.pose.position.x = this->modelPose.pos.x;
      this->propState.pose.position.y = this->modelPose.pos.y;
      this->propState.pose.position.z = this->modelPose.pos.z;
      this->propState.pose.orientation.x = this->modelPose.rot.x;
      this->propState.pose.orientation.y = this->modelPose.rot.y;
      this->propState.pose.orientation.z = this->modelPose.rot.z;
      this->propState.pose.orientation.w = this->modelPose.rot.w;
      this->propState.twist.linear.x = this->modelVelLin.x;
      this->propState.twist.linear.y = this->modelVelLin.y;
      this->propState.twist.linear.z = this->modelVelLin.z;
      this->propState.twist.angular.x = this->modelVelAng.x;
      this->propState.twist.angular.y = this->modelVelAng.y;
      this->propState.twist.angular.z = this->modelVelAng.z;
      this->propState.reference_frame = "propeller";

      this->rosPub.publish(this->propState);
    }

    public: void getWrenchMotor(const geometry_msgs::Wrench& wrench)
    {
      this->jointAxis->SetVelocity(0,wrench.torque.y); //Abuse of the wrench struct
      this->linkProp->AddRelativeForce(math::Vector3(wrench.force.x, wrench.force.y, wrench.force.z));
      // this->linkMot->AddRelativeTorque(math::Vector3(wrench.torque.y, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to motor joint
    physics::JointPtr jointAxis;
    // Pointer to propeller link
    physics::LinkPtr linkProp, linkMot;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS related variables
    private:
      ros::NodeHandle rosHandle;
      ros::Publisher rosPub;
      ros::Subscriber rosSubMotor;
      gazebo_msgs::ModelState propState;
      math::Pose modelPose;
      math::Vector3 modelVelLin, modelVelAng;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(engineControl)
}
