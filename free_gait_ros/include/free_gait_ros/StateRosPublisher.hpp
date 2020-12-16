/*
 * StateRosPublisher.hpp
 *
 *  Created on: Dec 6, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_msgs/RobotState.h>
#include <free_gait_ros/RosVisualization.hpp>

// ROS
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf2_ros/transform_broadcaster.h>

// STD
#include <memory>
#include <string>

#include <quadruped_model/quadrupedkinematics.h>
#include <quadruped_model/quadruped_state.h>

using namespace quadruped_model;

namespace free_gait {

class StateRosPublisher
{
 public:
  StateRosPublisher(ros::NodeHandle& nodeHandle, AdapterBase& adapter);
  virtual ~StateRosPublisher();
  StateRosPublisher(const StateRosPublisher& other);

  void setTfPrefix(const std::string tfPrefix);
  bool publish(const State& state, const StepQueue& step_queue);
  bool publish(const State& state);
  bool publish();
  bool publish(const Pose& base_pose);

  //  QuadrupedKinematics QK_;
    std::shared_ptr<QuadrupedKinematics> QK_;
 private:
  bool initializeRobotStatePublisher();

  ros::NodeHandle& nodeHandle_;
  std::string tfPrefix_;
  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisher_;
  AdapterBase& adapter_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  //! eric_wang: Publish desired robot state to the controller.
  ros::Publisher robot_state_pub_, stance_marker_pub_;
  free_gait_msgs::RobotState robot_state_;


};

} /* namespace free_gait */
