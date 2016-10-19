#include "ros/ros.h"
#include "last_letter/gazebo_step.h"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

bool gazebo_step_callback(last_letter::gazebo_step::Request &req, last_letter::gazebo_step::Response &res)
{
  // Publish the step message for the simulation.
  gazebo::msgs::WorldControl msg;
  msg.set_step(1);
//  msg.step = true;
  pub->Publish(msg);

  res.response = 1;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_step_server");
  ros::NodeHandle n;

  // Gazebo WorldControl Topic
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  // Gazebo Publisher
  gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

  ros::ServiceServer service = n.advertiseService("gazebo_step",gazebo_step_callback);
  ROS_INFO("gazebo_step_server ready");
  ros::spin();

  return 0;
}
