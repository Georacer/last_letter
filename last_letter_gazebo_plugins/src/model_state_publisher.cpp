#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <stdio.h>

#include "gazebo_msgs/ModelState.h"
#include "last_letter_msgs/SimStates.h"
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
      this->world = this->model->GetWorld();

      this->WGS84 = this->world->GetSphericalCoordinates();

      this->last_time = this->world->GetSimTime();
      this->last_velLin = this->model->GetWorldLinearVel();
      this->last_pose = this->model->GetWorldPose();

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

      this->rosPub = this->rosHandle.advertise<last_letter_msgs::SimStates>("/" + this->model->GetName() + "/modelState",1); //model states publisher

      ROS_INFO("modelStateBroadcaster plugin initialized");
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {

      math::Vector3 tempVect;
      math::Quaternion tempQuat;
      math::Quaternion InertialQuat(0,0.7071,0.7071,0); // Rotation quaterion from NED to ENU
      math::Quaternion BodyQuat(0,1,0,0); // Rotation quaternion from default body to aerospace body frame

      // Read the model state
      this->modelPose = this->model->GetWorldPose();
      // Convert rotation
      this->modelPose.rot = this->modelPose.rot*BodyQuat;

        // Velocities required in the aerospace body frame
      this->modelVelLin = BodyQuat*this->model->GetRelativeLinearVel();
      this->modelVelAng = BodyQuat*this->model->GetRelativeAngularVel();

      this->modelState.header.frame_id = this->model->GetName();

      this->modelState.header.stamp = ros::Time::now();

      tempVect = InertialQuat*this->modelPose.pos; // Rotate position vector to NED frame
      if (!std::isfinite(this->modelPose.pos.x)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in Gazebo east"); this->model->Reset(); return;}
      if (!std::isfinite(this->modelPose.pos.y)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in Gazebo north"); this->model->Reset(); return;}
      if (!std::isfinite(this->modelPose.pos.z)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in Gazeo up"); this->model->Reset(); return;}
      this->modelState.pose.position.x = tempVect.x;
      if (!std::isfinite(tempVect.x)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in NED north"); this->model->Reset(); return;}
      this->modelState.pose.position.y = tempVect.y;
      if (!std::isfinite(tempVect.y)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in NED east"); this->model->Reset(); return;}
      this->modelState.pose.position.z = tempVect.z;
      if (!std::isfinite(tempVect.z)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in NED down"); this->model->Reset(); return;}

      // ROS_INFO("Gazebo quaternion (w/x/y/z): %g\t%g\t%g\t%g", this->modelPose.rot.w, this->modelPose.rot.x, this->modelPose.rot.y, this->modelPose.rot.z);
      if (!std::isfinite(this->modelPose.rot.x)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in Gazebo body rot.x"); this->model->Reset(); return;}
      if (!std::isfinite(this->modelPose.rot.y)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in Gazebo body rot.y"); this->model->Reset(); return;}
      if (!std::isfinite(this->modelPose.rot.z)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in Gazebo body rot.z"); this->model->Reset(); return;}
      if (!std::isfinite(this->modelPose.rot.w)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in Gazebo body rot.w"); this->model->Reset(); return;}

      tempQuat = InertialQuat*this->modelPose.rot; // Rotate orientation quaternion to NED frame
      if (tempQuat.w<0)
      {
        tempQuat.w = -tempQuat.w;
        tempQuat.x = -tempQuat.x;
        tempQuat.y = -tempQuat.y;
        tempQuat.z = -tempQuat.z;
      }
      this->modelState.pose.orientation.x = tempQuat.x;
      this->modelState.pose.orientation.y = tempQuat.y;
      this->modelState.pose.orientation.z = tempQuat.z;
      this->modelState.pose.orientation.w = tempQuat.w;
      // ROS_INFO("aerospace quaternion (w/x/y/z): %g\t%g\t%g\t%g", tempQuat.w, tempQuat.x, tempQuat.y, tempQuat.z);
      if (!std::isfinite(tempQuat.x)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in aerospace body rot.x"); this->model->Reset(); return;}
      if (!std::isfinite(tempQuat.y)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in aerospace body rot.y"); this->model->Reset(); return;}
      if (!std::isfinite(tempQuat.z)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in aerospace body rot.z"); this->model->Reset(); return;}
      if (!std::isfinite(tempQuat.w)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in aerospace body rot.w"); this->model->Reset(); return;}

      this->modelState.velocity.linear.x = this->modelVelLin.x;
      if (!std::isfinite(this->modelVelLin.x)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in u"); this->model->Reset(); return;}
      this->modelState.velocity.linear.y = this->modelVelLin.y;
      if (!std::isfinite(this->modelVelLin.y)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in v"); this->model->Reset(); return;}
      this->modelState.velocity.linear.z = this->modelVelLin.z;
      if (!std::isfinite(this->modelVelLin.y)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in w"); this->model->Reset(); return;}

      this->modelState.velocity.angular.x = this->modelVelAng.x;
      if (!std::isfinite(this->modelVelAng.x)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in p"); this->model->Reset(); return;}
      this->modelState.velocity.angular.y = this->modelVelAng.y;
      if (!std::isfinite(this->modelVelAng.y)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in q"); this->model->Reset(); return;}
      this->modelState.velocity.angular.z = this->modelVelAng.z;
      if (!std::isfinite(this->modelVelAng.z)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in r"); this->model->Reset(); return;}

      // Get WGS84 coordinates
      math::Vector3 coords = this->WGS84->SphericalFromLocal(ignition::math::Vector3d(this->modelPose.pos.x, this->modelPose.pos.y, this->modelPose.pos.z));
      this->modelState.geoid.latitude = coords.x;
      this->modelState.geoid.longitude = coords.y;
      this->modelState.geoid.altitude = coords.z;
      if (!std::isfinite(coords.x)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in lat"); this->model->Reset(); return;}
      if (!std::isfinite(coords.y)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in lon"); this->model->Reset(); return;}
      if (!std::isfinite(coords.z)) {ROS_ERROR("model_state_publisher.cpp plugin: NaN value in alt"); this->model->Reset(); return;}

      // ROS_INFO("WGS84 coordinates (lat/lon/alt): %g\t%g\t%g", coords.x, coords.y, coords.z);

      // // Fill the acceleration field with the virtual accelerometer readings
      // tempVect = this->model->GetWorldLinearAccel(); // Read inertial acceleration in inertial frame

      common::Time cur_time = this->world->GetSimTime();
      double dt = this->last_time.Double() - cur_time.Double();
      if (dt != 0)
      {
        // Get acceleration from velocity
        tempVect = (this->last_velLin - this->model->GetWorldLinearVel())/dt;
        this->last_velLin = this->model->GetWorldLinearVel();

        //// Get acceleration from position
        // this->curr_velLin = (this->modelPose.pos - this->last_pose.pos)/dt; // Differentiate pos to get vel
        // tempVect = (this->curr_velLin - this->last_velLin)/dt; // Differentiate vel to get acc
        // this->last_pose = this->modelPose;
        // this->last_velLin = this->curr_velLin;

        this->last_time = cur_time;
      }

      ignition::math::Vector3d gravity(this->world->Gravity()); // Read gravity acceleration in inertial frame
      tempVect = tempVect - math::Vector3(gravity.X(), gravity.Y(), gravity.Z()); // Subtract gravity
      tempVect = this->modelPose.rot*tempVect; // Rotate acceleration to body frame
      this->modelState.acceleration.linear.x = tempVect.x;
      this->modelState.acceleration.linear.y = tempVect.y;
      this->modelState.acceleration.linear.z = tempVect.z;

        // Apply (to Z-axis) Butterworth low-pass filter  /w cutoff freq=100Hz @ Sampling 400Hz
        double inputX = tempVect.x;
        double inputY = tempVect.y;
        double inputZ = tempVect.z;
        tempVect.z = -a[1]*accZfHist[0]-a[2]*accZfHist[1]-a[3]*accZfHist[2]+b[0]*inputZ+b[1]*accZHist[0]+b[2]*accZHist[1]+b[3]*accZHist[2];
        accZHist[2]=accZHist[1];
        accZHist[1]=accZHist[0];
        accZHist[0]=inputZ;
        accZfHist[2]=accZfHist[1];
        accZfHist[1]=accZfHist[0];
        accZfHist[0]=tempVect.z;

        //// Apply averaging filter, size 10
        // for (int i = 10; i>0; i--)
        // {
        //   accZHist[i] = accZHist[i-1];

        // }
        // accZHist[0] = inputZ;
        // sum = 0;
        // for (int i = 0; i<10; i++)
        // {
        //   sum+=accZHist[i];
        // }
        // tempVect.z = sum;

      this->rosPub.publish(this->modelState);
    }

    // Pointer to the world
    private: physics::WorldPtr world;
    // Pointer to the model
    private: physics::ModelPtr model;

    private: gazebo::common::SphericalCoordinatesPtr WGS84;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS related variables
    private:
      ros::NodeHandle rosHandle;
      ros::Publisher rosPub;
      last_letter_msgs::SimStates modelState;
      math::Pose modelPose;
      math::Vector3 modelVelLin, modelVelAng;
      math::Vector3 curr_velLin, last_velLin;
      math::Pose last_pose;
      common::Time last_time;

    private:
      double b [4] = {0.0376e-4, 0.1127, 0.1227, 0.0376};
      double a [4] = {1.000, -2.9372, 2.8763, -9.9391};
      double sum = 0;
      double accXHist[10] ={0};
      double accYHist[10] ={0};
      double accZHist[10] ={0};
      double accXfHist[10] ={0};
      double accYfHist[10] ={0};
      double accZfHist[10] ={0};
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(modelStateBroadcaster)
}
