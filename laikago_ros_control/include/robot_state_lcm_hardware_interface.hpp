#pragma once
// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/robot_hw.h>
#include "hardware_interface/imu_sensor_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include "robot_state_interface.hpp"
#include "transmission_interface/transmission_info.h"
#include <transmission_interface/transmission_parser.h>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>

// URDF
#include <urdf/model.h>

// Laikago SDK
#include <laikago_sdk/laikago_sdk.hpp>
#include <laikago_msgs/LowCmd.h>
#include <laikago_msgs/LowState.h>
#include <laikago_msgs/HighState.h>
#include <laikago_msgs/HighCmd.h>

#include "eigen3/Eigen/Eigen"
#include "unordered_map"

#include <fstream>
#include <iostream>

#include<algorithm>
#include "ros/package.h"


namespace laikago_ros_control {

class RobotStateLcmHardwareInterface : public hardware_interface::RobotHW
{

public:
  RobotStateLcmHardwareInterface();
  ~RobotStateLcmHardwareInterface();
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

  void read(const ros::Time& time, const ros::Duration& period);

  void write(const ros::Time& time, const ros::Duration& period);

  void update_loop();


  //bool loadParameters(ros::NodeHandle& nh);
  //bool checkPositionLimits(const int index, const double position);

protected:

  ros::NodeHandle node_handle_;

  std::vector<double> motor_friction_mu, motor_friction_bias,
                      motor_zero_offsets, motor_friction_proportion,
                      motor_max_limits, motor_min_limits, motor_directions;
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID, STANCE_LEG, FREEZE};


  unsigned int n_dof_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::ImuSensorInterface imu_interface_;
  hardware_interface::ImuSensorHandle::Data imu_data_;

  hardware_interface::RobotStateInterface robot_state_interface_;

  hardware_interface::RobotStateHandle::Data robot_state_data_;

  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<ControlMethod> joint_control_methods_, last_joint_control_methods_;
  std::vector<control_toolbox::Pid> pid_controllers_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> last_joint_position_command_;
  std::vector<double> joint_velocity_command_;

  std::string physics_type_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;
  std::vector<bool> motor_disabled, motor_unused;

  double pos_read[12], pos_write[12], vel_read[12], vel_write[12], eff_read[12],eff_write[12];
  double position[3], orinetation[4], linear_vel[3], angular_vel[3];
  int foot_contact[4];

  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  boost::recursive_mutex r_mutex_;
  boost::mutex mutex;
  boost::thread update_thread_;

private:

  //Laikago sdk
  laikago_msgs::LowCmd SendLowROS;
  laikago_msgs::LowState RecvLowROS;
  laikago_msgs::HighCmd SendHighROS;
  laikago_msgs::HighState RecvHighROS;
  laikago::LowCmd SendLowLCM;
  laikago::LowState RecvLowLCM;
  laikago::HighCmd SendHighLCM;
  laikago::HighState RecvHighLCM;
  laikago::LCM roslcm;

};


typedef boost::shared_ptr<RobotStateLcmHardwareInterface> RobotStateLcmHardwareInterfacePtr;

};
