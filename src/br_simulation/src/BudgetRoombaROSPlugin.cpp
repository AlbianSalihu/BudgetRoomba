#include "br_simulation/BudgetRoombaROSPlugin.h"

#include <gazebo/physics/physics.hh>
#include <iostream>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace gazebo
{

BudgetRoombaROSPlugin::BudgetRoombaROSPlugin()
{
}

void BudgetRoombaROSPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get model and world references
  ros_node_ = gazebo_ros::Node::Get(sdf);
  RCLCPP_INFO(ros_node_->get_logger(), "Loading budget_roomba_ROS_plugin ...");
  this->tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(this->ros_node_));
  ground_truth_tf_ = sdf->GetElement("ground_truth_tf")->Get<bool>();
  robot_base_frame_id_ = sdf->GetElement("robot_base_frame_id")->Get<std::string>();
  robot_base_link_ = model->GetLink(robot_base_frame_id_);
  world_frame_id_ = sdf->GetElement("world_frame_id")->Get<std::string>();

  
  
  model_ = model;
  world_ = model_->GetWorld();
  // Hook into simulation update loop
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&BudgetRoombaROSPlugin::Update, this));
  RCLCPP_INFO(ros_node_->get_logger(), "... budget_roomba_ROS_plugin loaded !");
  
}

void BudgetRoombaROSPlugin::Update()
{
  if(this->ground_truth_tf_){
    const ignition::math::Pose3d robot_pose_link_pose = robot_base_link_->WorldPose();
    geometry_msgs::msg::TransformStamped robot_pose_tf;
    robot_pose_tf.child_frame_id = this->robot_base_frame_id_;
    robot_pose_tf.header.frame_id = this->world_frame_id_;
    
    robot_pose_tf.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(this->world_->SimTime());

    robot_pose_tf.transform.translation.x = robot_pose_link_pose.Pos().X();
    robot_pose_tf.transform.translation.y = robot_pose_link_pose.Pos().Y();
    robot_pose_tf.transform.translation.z = robot_pose_link_pose.Pos().Z();
    robot_pose_tf.transform.rotation.x = robot_pose_link_pose.Rot().X();
    robot_pose_tf.transform.rotation.y = robot_pose_link_pose.Rot().Y();
    robot_pose_tf.transform.rotation.z = robot_pose_link_pose.Rot().Z();
    robot_pose_tf.transform.rotation.w = robot_pose_link_pose.Rot().W();
    this->tf_broadcaster_->sendTransform(robot_pose_tf);
    RCLCPP_DEBUG_STREAM(ros_node_->get_logger(), "[BudgetRoombaROSPlugin] Callback");


  }
  

  

}

GZ_REGISTER_MODEL_PLUGIN(BudgetRoombaROSPlugin)

}  
