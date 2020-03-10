/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr_ros/towr_ros_interface.h>

#include <std_msgs/Int32.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr/terrain/height_map.h>
#include <towr/variables/euler_converter.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace towr {


TowrRosInterface::TowrRosInterface ()
{
  ros::NodeHandle n;

  user_command_sub_ = n.subscribe(towr_msgs::user_command, 1,
                                  &TowrRosInterface::UserCommandCallback, this);

  initial_state_sub_ = n.subscribe<free_gait_msgs::RobotState>("/gazebo/robot_states", 1, &TowrRosInterface::InitialStateCallback, this);

  // static const std::string robot_state_desired("/xpp/state_des")
  initial_state_pub_  = n.advertise<xpp_msgs::RobotStateCartesian>
                                          (xpp_msgs::robot_state_desired, 1);// the whole trajectory is also published by the topic.

  // static const std::string robot_parameters("/xpp/params");
  robot_parameters_pub_  = n.advertise<xpp_msgs::RobotParameters>
                                    (xpp_msgs::robot_parameters, 1);
  // publish the marker by eric.
  base_pose_pub_ = n.advertise<geometry_msgs::PoseArray>("base_pose", 1);

  ee_motion_pub_ = n.advertise<visualization_msgs::Marker>("ee_motion", 1);

  solver_ = std::make_shared<ifopt::IpoptSolver>();

  visualization_dt_ = 0.01;
}


void TowrRosInterface::InitialStateCallback(const free_gait_msgs::RobotStateConstPtr &robot_state_msg)
{
//  ROS_INFO("Get initial base state");
    // Transmit state_ to formulation.
    state_ = GetInitialState(robot_state_msg);
}

BaseState TowrRosInterface::GetInitialState(const free_gait_msgs::RobotStateConstPtr &robot_state_msg)
{

    BaseState initial_state;
    initial_state.lin.at(kPos).x() = robot_state_msg->base_pose.pose.pose.position.x;
    initial_state.lin.at(kPos).y() = robot_state_msg->base_pose.pose.pose.position.y;
    initial_state.lin.at(kPos).z() = 0.52;//robot_state_msg->base_pose.pose.pose.position.z;
//    ROS_INFO_STREAM("Initial base pose" << initial_state.lin.at(kPos).x() << initial_state.lin.at(kPos).y() << std::endl);

    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(robot_state_msg->base_pose.pose.pose.orientation, quaternion);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    initial_state.ang.at(kPos).x() = roll;
    initial_state.ang.at(kPos).y() = pitch;
    initial_state.ang.at(kPos).z() = yaw;
//    ROS_INFO_STREAM("Initial base orientation:" << yaw << std::endl);
    return initial_state;
}

BaseState TowrRosInterface::GetGoalState(const TowrCommandMsg& msg) const
{
  BaseState goal;
  goal.lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos);
  goal.lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
  goal.ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
  goal.ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);

  return goal;
}
/**
 * @brief TowrRosInterface::UserCommandCallback Run once any new command sends.
 * @param msg
 */
void TowrRosInterface::UserCommandCallback(const TowrCommandMsg& msg)
{
  ROS_INFO("Get user command once");
  // robot model
  formulation_.model_ = RobotModel(static_cast<RobotModel::Robot>(msg.robot)); //! eric_wang: Get what kind of robots, biped or quadruped.
  auto robot_params_msg = BuildRobotParametersMsg(formulation_.model_);
  robot_parameters_pub_.publish(robot_params_msg);

  // terrain
  auto terrain_id = static_cast<HeightMap::TerrainID>(msg.terrain);
  formulation_.terrain_ = HeightMap::MakeTerrain(terrain_id);

  int n_ee = formulation_.model_.kinematic_model_->GetNumberOfEndeffectors();
  formulation_.params_ = GetTowrParameters(n_ee, msg);//! eric_wang: quadruped gait generator.
  formulation_.final_base_ = GetGoalState(msg);

  SetTowrInitialState();//! eric_wang: Add intialized base pos and nominal stance into formulation.

  // solver parameters
  SetIpoptParameters(msg);//! eric_wang: Sets the solver params required to formulate the TOWR problem.

  // visualization
  PublishInitialState(); //! eric_wang: Publish initialized robot state to XPP.

  // Defaults to /home/user/.ros/
  std::string bag_file = "towr_trajectory.bag";
  if (msg.optimize || msg.play_initialization) {
    //! eric_wang: Convert nlp_formulation into ifpot::Problem nlp_(the actual nonlinear program).
    nlp_ = ifopt::Problem();
    for (auto c : formulation_.GetVariableSets(solution))
      nlp_.AddVariableSet(c);
    for (auto c : formulation_.GetConstraints(solution))
      nlp_.AddConstraintSet(c);
    for (auto c : formulation_.GetCosts())
      nlp_.AddCostSet(c);

    solver_->Solve(nlp_); //! eric_wang: Solve NLP problem.

    // TODO(EricWang): Publish base pose by geometry_msgs::poseArray.
    auto trajectory = GetTrajectory();
    geometry_msgs::PoseArray Pose_;
    Pose_.header.frame_id = "world";
    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Point";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.color.g = 0.1;
    marker.color.a = 0.1;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.pose.orientation.w = 1.0;
    xpp_msgs::StateLin3d eemotion;
    xpp_msgs::RobotStateCartesian state_;

    for(std::vector<xpp::RobotStateCartesian>::iterator it = trajectory.begin(); it != trajectory.end(); ++it)
    {
        for(auto ee : it->ee_contact_.GetEEsOrdered())
        {
            eemotion = xpp::Convert::ToRos(it->ee_motion_.at(ee));
            point.x = eemotion.pos.x;
            point.y = eemotion.pos.y;
            point.z = eemotion.pos.z;
            marker.points.push_back(point);
        }
        ee_motion_pub_.publish(marker);

        state_ = xpp::Convert::ToRos(*it);// To xpp_msgs::RobotStateCartesian.
        pose = state_.base.pose;
        Pose_.poses.push_back(pose);
    }
    base_pose_pub_.publish(Pose_);

    SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, false);
  }


  //! eric_wang: Get the optimization result.
  //ROS_WARN_STREAM(" dd" << solution.base_linear_->GetTotalTime() << std::endl);
  //std::cout << solution.base_linear_->GetTotalTime() << std::endl;

  // playback using terminal commands
  if (msg.replay_trajectory || msg.play_initialization || msg.optimize) {
    int success = system(("rosbag play --topics " // int system(char *string)
        + xpp_msgs::robot_state_desired + " " // topic name 1 and advertise the topic.
        + xpp_msgs::terrain_info // topic name 2
        + " -r " + std::to_string(msg.replay_speed) // replay speed
        + " --quiet " + bag_file).c_str()); // return a pointer of string.
  }
  // plot the trajectory using rqt_bag.
  if (msg.plot_trajectory) {
    int success = system(("killall rqt_bag; rqt_bag " + bag_file + "&").c_str());
  }

  // TODO(EricWang): Send to free_gait for tracking.
  // to publish entire trajectory (e.g. to send to controller)
//   xpp_msgs::RobotStateCartesianTrajectory xpp_msg = xpp::Convert::ToRos(GetTrajectory());

  // Prints the variables, costs and constraints.
  nlp_.PrintCurrent();
}

void
TowrRosInterface::PublishInitialState()
{
  int n_ee = formulation_.initial_ee_W_.size();
  xpp::RobotStateCartesian xpp(n_ee);//! eric_wang: Defines a complete robot state in Cartesian space according to feet number.
  xpp.base_.lin.p_ = formulation_.initial_base_.lin.p();
  xpp.base_.ang.q  = EulerConverter::GetQuaternionBaseToWorld(formulation_.initial_base_.ang.p());

  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;// Converts endeffector IDs of towr into the corresponding number used in xpp.
    xpp.ee_contact_.at(ee_xpp)   = true;
    xpp.ee_motion_.at(ee_xpp).p_ = formulation_.initial_ee_W_.at(ee_towr);
    xpp.ee_forces_.at(ee_xpp).setZero(); // zero for visualization
  }

  initial_state_pub_.publish(xpp::Convert::ToRos(xpp));
}

std::vector<TowrRosInterface::XppVec>
TowrRosInterface::GetIntermediateSolutions ()
{
  std::vector<XppVec> trajectories;

  for (int iter=0; iter<nlp_.GetIterationCount(); ++iter) {
    nlp_.SetOptVariables(iter);
    trajectories.push_back(GetTrajectory());
  }

  return trajectories;
}

TowrRosInterface::XppVec // std::vector<xpp::RobotStateCartesian>;
TowrRosInterface::GetTrajectory () const
{
  XppVec trajectory;
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();

  EulerConverter base_angular(solution.base_angular_);

  while (t<=T+1e-5) {
    int n_ee = solution.ee_motion_.size();
    xpp::RobotStateCartesian state(n_ee);

    state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

    state.base_.ang.q  = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w  = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
      int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;

      state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
      state.ee_motion_.at(ee_xpp)  = ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
      state.ee_forces_ .at(ee_xpp) = solution.ee_force_.at(ee_towr)->GetPoint(t).p();
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += visualization_dt_;
  }

  return trajectory;
}

xpp_msgs::RobotParameters
TowrRosInterface::BuildRobotParametersMsg(const RobotModel& model) const
{
  xpp_msgs::RobotParameters params_msg;
  auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
  int n_ee = nominal_B.size();
  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    Vector3d pos = nominal_B.at(ee_towr);
    params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(pos));
    params_msg.ee_names.push_back(ToXppEndeffector(n_ee, ee_towr).second);
  }

  params_msg.base_mass = model.dynamic_model_->m();

  return params_msg;
}

void
TowrRosInterface::SaveOptimizationAsRosbag (const std::string& bag_name,
                                   const xpp_msgs::RobotParameters& robot_params,
                                   const TowrCommandMsg user_command_msg,
                                   bool include_iterations)
{
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);
  ::ros::Time t0(1e-6); // t=0.0 throws ROS exception

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::robot_parameters, t0, robot_params);
  bag.write(towr_msgs::user_command+"_saved", t0, user_command_msg);

  // save the trajectory of each iteration
  if (include_iterations) {
    auto trajectories = GetIntermediateSolutions();
    int n_iterations = trajectories.size();
    for (int i=0; i<n_iterations; ++i)
      SaveTrajectoryInRosbag(bag, trajectories.at(i), towr_msgs::nlp_iterations_name + std::to_string(i));

    // save number of iterations the optimizer took
    std_msgs::Int32 m;
    m.data = n_iterations;
    bag.write(towr_msgs::nlp_iterations_count, t0, m);
  }

  // save the final trajectory
  auto final_trajectory = GetTrajectory(); //! eric_wang: the whole trajectory from t0 to T.
  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired);

  bag.close();
}

void
TowrRosInterface::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                 const XppVec& traj,
                                 const std::string& topic) const
{
  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ + 1e-6); // t=0.0 throws ROS exception

    xpp_msgs::RobotStateCartesian msg;
    msg = xpp::Convert::ToRos(state);
    bag.write(topic, timestamp, msg);

    xpp_msgs::TerrainInfo terrain_msg;
    for (auto ee : state.ee_motion_.ToImpl()) {
      Vector3d n = formulation_.terrain_->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
      terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
      terrain_msg.friction_coeff = formulation_.terrain_->GetFrictionCoeff();
    }

    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
  }
}

} /* namespace towr */

