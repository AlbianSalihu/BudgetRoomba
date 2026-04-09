// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BUDGET_ROOMBA_ROS_PLUGIN_H
#define BUDGET_ROOMBA_ROS_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>


namespace gazebo
{

class BudgetRoombaROSPlugin : public ModelPlugin
{
public:
  BudgetRoombaROSPlugin();

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  void Update();


  std::string robot_namespace_;

  physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;


  event::ConnectionPtr update_connection_;
  bool ground_truth_tf_;
  std::string robot_base_frame_id_;
  physics::LinkPtr robot_base_link_;
  std::string world_frame_id_;


  rclcpp::Node::SharedPtr ros_node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

}

#endif  // BUDGET_ROOMBA_ROS_PLUGIN_H
