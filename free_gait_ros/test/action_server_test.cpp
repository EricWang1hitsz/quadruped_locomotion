/*
 *  action_server_test.cpp
 *  Descriotion:
 *
 *  Created on: Aug 31, 2017
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include "free_gait_ros/free_gait_ros.hpp"
#include "free_gait_core/free_gait_core.hpp"
#include "pluginlib/class_loader.h"
#include "boost/bind.hpp"
#include "boost/thread.hpp"

#include "sensor_msgs/JointState.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "ros/advertise_service_options.h"
#include "free_gait_msgs/SetLimbConfigure.h"

#include "free_gait_ros/gait_generate_client.hpp"

using namespace free_gait;
using namespace std;
class ActionServerTest
{
public:
  ActionServerTest(ros::NodeHandle& nodehandle)
    : nodeHandle_(nodehandle),
      adapter_loader("free_gait_ros", "free_gait::AdapterBase"),
      use_gazebo(true),
      is_pause(false),
      is_stop(false),
      is_kinematics_control(true),
      is_start_gait(false),
      is_joy_control(false),
      AdapterRos_(nodehandle, free_gait::AdapterRos::AdapterType::Gazebo),
      gait_generate_client_(nodehandle)

  {
    nodeHandle_.getParam("/use_gazebo", use_gazebo);
    nodeHandle_.getParam("/kinematics_control",is_kinematics_control);
    nodeHandle_.getParam("/free_gait/stop_execution_service", stop_service_name_);
    nodeHandle_.getParam("/free_gait/pause_execution_service", pause_service_name_);
//    nodeHandle_.getParam("/gait_generate/")
//    ROS_ERROR("In action server thread");
    if(use_gazebo){
      adapter.reset(AdapterRos_.getAdapterPtr());
//      ROS_ERROR("In action server thread");
      //! eric_wang: Subscribe robot state from Gazebo.
      AdapterRos_.subscribeToRobotState();
//      ROS_ERROR("In action server thread");
    } else {
      adapter.reset(adapter_loader.createUnmanagedInstance("free_gait_ros/AdapterDummy"));
    };
//    ROS_ERROR("In action server thread");
    state.reset(new State());
    parameters.reset(new StepParameters());
    completer.reset(new StepCompleter(*parameters, *adapter));
    computer.reset(new StepComputer());
//    ROS_ERROR("In action server thread");
    //! eric_wang: Get last robot state from Gazebo.
    AdapterRos_.updateAdapterWithState();
    //! WSHY: get the initial pose and set to the fisrt pose command
    state->setPositionWorldToBaseInWorldFrame(AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame());
    state->setOrientationBaseToWorld(AdapterRos_.getAdapter().getOrientationBaseToWorld());
    state->setLinearVelocityBaseInWorldFrame(AdapterRos_.getAdapter().getLinearVelocityBaseInWorldFrame());
    state->setAngularVelocityBaseInBaseFrame(AdapterRos_.getAdapter().getAngularVelocityBaseInBaseFrame());
    std::cout<<*state<<std::endl;

    executor.reset(new Executor(*completer, *computer, *adapter, *state));
    rosPublisher.reset(new StateRosPublisher(nodeHandle_, *adapter));
    if(is_kinematics_control){
      rosPublisher->setTfPrefix("/ideal");
      }else if (!is_kinematics_control) {
      rosPublisher->setTfPrefix("/desired");
    }

    executor->initialize();

    AdapterRos_.updateAdapterWithState();
//    AdvertiseServiceOptions pause_service_option = Advertise("a", boost::bind(&ActionServerTest::PauseServiceCallback, this, _1, _2), ros::VoidConstPtr())
    pause_service_server_ = nodeHandle_.advertiseService(pause_service_name_, &ActionServerTest::PauseServiceCallback, this);
    stop_service_server_ = nodeHandle_.advertiseService(stop_service_name_, &ActionServerTest::StopServiceCallback, this);
    gait_start_server_ = nodeHandle_.advertiseService("/gait_generate_switch", &ActionServerTest::GaitGenerateSwitchCallback, this);
    pace_start_server_ = nodeHandle_.advertiseService("/pace_switch", &ActionServerTest::PaceSwitchCallback, this);
    joy_control_server_ = nodeHandle_.advertiseService("/joy_control_switch", &ActionServerTest::joyControlSwitchCallback, this);
    crawl_start_server_ = nodeHandle_.advertiseService("/crawl_switch", &ActionServerTest::CrawlSwitchCallback, this);

    towr_start_server_ = nodehandle.advertiseService("/towr_switch", &ActionServerTest::TowrSwitchCallback, this);

    limb_configure_switch_server_ = nodeHandle_.advertiseService("/limb_configure", &ActionServerTest::SwitchLimbConfigureCallback, this);

    joint_state_pub_ = nodeHandle_.advertise<sensor_msgs::JointState>("all_joint_position", 1);
    allJointStates_.position.resize(12);

    server_.reset(new FreeGaitActionServer(nodeHandle_, "/free_gait/action_server", *executor, *adapter));
    server_->initialize();
    server_->start();
//    server_->update();
    //-------------------------------------------------------
    // Question How to work together between these two thread?
    //-------------------------------------------------------
    action_server_thread_ = boost::thread(boost::bind(&ActionServerTest::ActionServerThread, this));


    gait_generate_thread_ = boost::thread(boost::bind(&ActionServerTest::GaitGenerateThread, this));

  }
  ~ActionServerTest() {}

  void ActionServerThread()//(const FreeGaitActionServer& server)
  {
      ROS_INFO("In action server thread");
      parameters->footstepParameters.minimumDuration_ = 0.4;
      parameters->baseTargetParameters.minimumDuration = 0.4;
      double dt = 0.01;// change dt cause problem?
      double time = 0.0;
      ros::Rate rate(100);
      ros::Duration(0.5).sleep();
      cout<<"Current base position : "<<AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame()<<endl;
      AdapterRos_.updateAdapterWithState();
      executor->reset(); //! WSHY: adapter update has wrong
      cout<<adapter->getState()<<endl;
      cout<<"Current base position again : "<<AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame()<<endl;
//      //! WSHY: get the initial pose and set to the fisrt pose command
      state->setPositionWorldToBaseInWorldFrame(AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame());
      state->setOrientationBaseToWorld(AdapterRos_.getAdapter().getOrientationBaseToWorld());
      state->setLinearVelocityBaseInWorldFrame(AdapterRos_.getAdapter().getLinearVelocityBaseInWorldFrame());
      state->setAngularVelocityBaseInBaseFrame(AdapterRos_.getAdapter().getAngularVelocityBaseInBaseFrame());

      for(int i=0;i<12;i++){
          allJointStates_.position[i] = adapter->getState().getJointPositionFeedback()(i);
        }
      if(is_kinematics_control)
        joint_state_pub_.publish(allJointStates_);
      std::cout<<AdapterRos_.getAdapter().getState()<<std::endl;
      rosPublisher->publish(AdapterRos_.getAdapter().getState());

      //----------------------------------------------------------
      // (EricWang) Cycling when ros is ok for this node.
      //----------------------------------------------------------
      while (ros::ok()) {
//          ROS_INFO("Action server updated Once");
          server_->update();//! eric_wang: excutor.getqueen.add(step).
//          ROS_INFO("Action Server updated Once");
//          AdapterRos_.updateAdapterWithState();
//          cout<<"Current base position : "<<AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame()<<endl;

//          TODO(Shunyao): How to Update?
//          cout<<"Current base position : "<<AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame()<<endl;
//          cout<<"joint position: "<<AdapterRos_.getAdapter().getAllJointPositions()<<endl;
          if (!executor->getQueue().empty()&&!is_pause&&!is_stop) {
              ROS_INFO("Executor is working");
            boost::recursive_mutex::scoped_lock lock(r_mutex_);
            AdapterRos_.updateAdapterWithState();

            executor->advance(dt, false);
            //----------------------------------------------------------
            // (EricWang) Publish desired robot state after planned to the controller.
            //----------------------------------------------------------
            //-----------------------------------------------------------
            // Question Which if it should enter when crawl gait is true?
            // Publish Crawl desired robot state when base auto.
            //-----------------------------------------------------------
            // eric_wang: Trot gait.
            if(is_start_gait && !gait_generate_client_.ignore_vd){
                state->setLinearVelocityBaseInWorldFrame(desired_linear_velocity_);
                state->setAngularVelocityBaseInBaseFrame(desired_angular_velocity_);
                state->setPositionWorldToBaseInWorldFrame(gait_generate_client_.getDesiredBasePose().getPosition());
                state->setOrientationBaseToWorld(gait_generate_client_.getDesiredBasePose().getRotation());
              }
            // eric_wang: Pace gait.
            if(is_start_gait && gait_generate_client_.getGaitType() == 2)
              {
                state->setPositionWorldToBaseInWorldFrame(gait_generate_client_.getDesiredBasePose().getPosition());
                state->setOrientationBaseToWorld(gait_generate_client_.getDesiredBasePose().getRotation());
              }
//            ROS_WARN_STREAM("Desired Velocity :"<<desired_linear_velocity_<<endl);
//            ROS_WARN_STREAM("Desired in State Velocity :"<<state->getTargetLinearVelocityBaseInWorldFrame()<<endl);
            time = time + dt;
            for(int i=0;i<12;i++){
                allJointStates_.position[i] = adapter->getState().getJointPositions()(i);
              }
            if(is_kinematics_control){
                //! WSHY: directly publish joint positions
              joint_state_pub_.publish(allJointStates_);
              }
//            cout<<"*************************************send joint command : "<<endl<<adapter->getState().getJointPositions()<<endl;
            if(!use_gazebo){
                //! WSHY: fake mode, publish fake state to rviz visualization
              rosPublisher->publish(adapter->getState());
              }
            if(!is_kinematics_control){
                //! WSHY: publish robot state to balance controller
              rosPublisher->publish(adapter->getState(), executor->getQueue());
              }
            // TODO(EricWang): Towr is as planner, send desired robot state to the balance controller.

            lock.unlock();
            }
          else{ // directly publish current state
//              state->setPositionWorldToBaseInWorldFrame(AdapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame());
//              state->setOrientationBaseToWorld(AdapterRos_.getAdapter().getOrientationBaseToWorld());
//              state->setLinearVelocityBaseInWorldFrame(AdapterRos_.getAdapter().getLinearVelocityBaseInWorldFrame());
//              state->setAngularVelocityBaseInBaseFrame(AdapterRos_.getAdapter().getAngularVelocityBaseInBaseFrame());
//              if(!is_kinematics_control){
//                  //! WSHY: publish robot state to balance controller
//                rosPublisher->publish(*state);
//                }
              // eric_wang: start pace gait.
              if(is_start_gait && gait_generate_client_.getGaitType() == 2)
                {
                  state->setPositionWorldToBaseInWorldFrame(gait_generate_client_.getDesiredBasePose().getPosition());
                  state->setOrientationBaseToWorld(gait_generate_client_.getDesiredBasePose().getRotation());
                  rosPublisher->publish(gait_generate_client_.getDesiredBasePose()); //eric_wang: just base pose.
                }
              // eric_wang: start trot gait.
              else if(is_start_gait && gait_generate_client_.getGaitType() == 1)
                {
                  state->setPositionWorldToBaseInWorldFrame(gait_generate_client_.getDesiredBasePose().getPosition());
                  state->setOrientationBaseToWorld(gait_generate_client_.getDesiredBasePose().getRotation());
                  state->setLinearVelocityBaseInWorldFrame(desired_linear_velocity_);
                  state->setAngularVelocityBaseInBaseFrame(desired_angular_velocity_);
                  rosPublisher->publish(gait_generate_client_.getDesiredBasePose());
                }
//              else if(!is_joy_control)
//                {
//                  rosPublisher->publish();
//                }
              else if(use_towr)
              {
                  ROS_INFO("Test if executor not working");
              }
            }

          rate.sleep();
        }

  }

  void GaitGenerateThread()
  {
    ROS_INFO("In GaitGenerateThread thread");
    ros::Rate rate(100);
    double dt = 0.01;
    gait_generate_client_.copyRobotState(AdapterRos_.getAdapter().getState());

//    gait_generate_client_.initializeTrot(0.45, 0.45);
//    gait_generate_client_.initializePace(0.45, 3*0.5);

    //--------------------------------------------------------------
    //! (EricWang) Cycling all the time, together with action server.
    //--------------------------------------------------------------
    while (ros::ok()) {
//        ROS_INFO("Gait Generate updated Once");
        double ts = ros::Time::now().toSec();
        //--------------------------------------------------------------
        //! (EricWang) Start generating gait once gait request turn true
        //--------------------------------------------------------------
        if(is_start_gait){
//            double ts = ros::Time::now().toSec();
            boost::recursive_mutex::scoped_lock lock(r_mutex_);
            AdapterRos_.updateAdapterWithState();
            gait_generate_client_.copyRobotState(AdapterRos_.getAdapter().getState());
            gait_generate_client_.advance(dt);
            //! eric_wang: ADD footstep generation in gait generation thread.
//            gait_generate_client_.footstepGeneration();
            gait_generate_client_.generateFootHolds("foot_print");
            gait_generate_client_.updateBaseMotion(desired_linear_velocity_, desired_angular_velocity_);
//            ROS_WARN_STREAM("Desired Velocity :"<<desired_linear_velocity_<<endl);
            lock.unlock();
            gait_generate_client_.sendMotionGoal();
//            dt = ros::Time::now().toSec() - ts;
          }else{
            //! WSHY: for marker publish
            boost::recursive_mutex::scoped_lock lock(r_mutex_);
            AdapterRos_.updateAdapterWithState();
            gait_generate_client_.copyRobotState(AdapterRos_.getAdapter().getState());
            lock.unlock();
          }

        rate.sleep();
        dt = ros::Time::now().toSec() - ts;
      }
  }

  bool StopServiceCallback(std_srvs::Trigger::Request& request,
                              std_srvs::Trigger::Response& response)
  {
    ROS_INFO("STOP Trigger");
    server_->setPreempted();
    is_stop = true;
    response.success = true;
    return true;
  }

  bool PauseServiceCallback(std_srvs::SetBool::Request& request,
                           std_srvs::SetBool::Response& response)
  {
    if(request.data == false){
        is_pause = false;
        is_stop = false;
        ROS_INFO("Start....");
      }
    if(request.data == true){
      is_pause = true;
      ROS_INFO("STOP....");
      }
    response.success = true;
    return true;
  }

  bool GaitGenerateSwitchCallback(std_srvs::SetBool::Request& request,
                                  std_srvs::SetBool::Response& response)
  {
    if(request.data == false){
        is_start_gait = false;
        ROS_INFO("STOP GAIT....");
        gait_generate_client_.current_velocity_buffer_.clear();
        desired_linear_velocity_ = LinearVelocity(0,0,0);
        desired_angular_velocity_ = LocalAngularVelocity(0,0,0);
      }
    if(request.data == true){
        is_start_gait = true;
        parameters->footstepParameters.minimumDuration_ = 0.3;
        parameters->baseTargetParameters.minimumDuration = 0.3;
        gait_generate_client_.initializeTrot(0.3,0.3);
//        gait_generate_client_.initializePace(0.45, 3*0.5);
      ROS_INFO("START GAIT....");
      }
    response.success = true;
    return true;
  }

  bool PaceSwitchCallback(std_srvs::SetBool::Request& request,
                                  std_srvs::SetBool::Response& response)
  {
    if(request.data == false){
        is_start_gait = false;
        gait_generate_client_.current_velocity_buffer_.clear();
        ROS_INFO("STOP GAIT....");
      }
    if(request.data == true){
        is_start_gait = true;
        parameters->footstepParameters.minimumDuration_ = 0.45;
        parameters->baseTargetParameters.minimumDuration = 0.45;
        gait_generate_client_.initializePace(0.45,0.45*3);
        gait_generate_client_.updateBaseMotion(desired_linear_velocity_,desired_angular_velocity_);

//        gait_generate_client_.initializePace(0.45, 3*0.5);
      ROS_INFO("START GAIT....");
      }
    response.success = true;
    return true;
  }

  bool CrawlSwitchCallback(std_srvs::SetBool::Request& request,
                                  std_srvs::SetBool::Response& response)
  {
    if(request.data == false){
        is_start_gait = false;
        gait_generate_client_.current_velocity_buffer_.clear();
        ROS_INFO("STOP CRAWL GAIT....");
      }
    if(request.data == true){
        is_start_gait = true;
        gait_generate_client_.initializeCrawl(1,0.45*3);
//        gait_generate_client_.initializePace(0.45, 3*0.5);
      ROS_INFO("START CRAWL GAIT....");
      }
    response.success = true;
    return true;
  }

  bool joyControlSwitchCallback(std_srvs::SetBool::Request& request,
                                  std_srvs::SetBool::Response& response)
  {
    if(request.data == false){
        is_joy_control = false;
        ROS_INFO("STOP JOY CONTROL MODE....");
      }
    if(request.data == true){
        is_joy_control = true;
        ROS_INFO("START JOY CONTROL MODE....");
      }
    response.success = true;
    return true;
  }

  bool TowrSwitchCallback(std_srvs::SetBool::Request& request,
                          std_srvs::SetBool::Response& response)
  {
      if(request.data == false)
      {
          use_towr = false;
          ROS_INFO("Not use towr");
      }
      if(request.data == true)
      {
          use_towr = true;
          ROS_INFO("Use towr");
      }
      response.success = true;
      return true;
  }


  bool SwitchLimbConfigureCallback(free_gait_msgs::SetLimbConfigure::Request& request,
                                   free_gait_msgs::SetLimbConfigure::Response& response)
  {
    response.result = state->setLimbConfigure(request.configure);
    return true;
  }

private:
//  FreeGaitActionServer server_;
  std::unique_ptr<FreeGaitActionServer> server_;
  boost::thread action_server_thread_, gait_generate_thread_;
  ros::NodeHandle nodeHandle_;

  free_gait::AdapterRos AdapterRos_;

  std::unique_ptr<AdapterBase> adapter;
  pluginlib::ClassLoader<AdapterBase> adapter_loader;


  std::unique_ptr<State> state;
  std::unique_ptr<StepParameters> parameters;
  std::unique_ptr<StepCompleter> completer;
  std::unique_ptr<StepComputer> computer;
  std::unique_ptr<Executor> executor;
  std::unique_ptr<StateRosPublisher> rosPublisher;

  GaitGenerateClient gait_generate_client_;

  sensor_msgs::JointState allJointStates_;
  ros::Publisher joint_state_pub_;
  ros::ServiceServer pause_service_server_, stop_service_server_, towr_start_server_,
  gait_start_server_, limb_configure_switch_server_, pace_start_server_, joy_control_server_, crawl_start_server_;
  std::string pause_service_name_, stop_service_name_;
  bool use_gazebo, is_pause, is_stop, is_kinematics_control, is_start_gait, is_joy_control;
  bool use_towr;
  /**
   * @brief r_mutex_
   */
  boost::recursive_mutex r_mutex_;
  LinearVelocity desired_linear_velocity_;
  LocalAngularVelocity desired_angular_velocity_;
};



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "action_server_test");
    ros::NodeHandle nh("~");
    ActionServerTest ac_test(nh);
//    pause_service_server = nh.advertiseService("/free_gait/pause_execution_service", &ActionServerTest::PauseServiceCallback, &ac_test);

    ros::spin();
//    pluginlib::ClassLoader<AdapterBase> adapter_loader("free_gait_ros", "free_gait::AdapterBase");
//    std::unique_ptr<AdapterBase> adapter;
//    adapter.reset(adapter_loader.createUnmanagedInstance("free_gait_ros/AdapterDummy"));


//    double dt = 0.001;
//    double time = 0.0;
////    ros::Time t_start = ros::Time::now();
//    ros::Rate rate(1000);
//    ros::spin();

//        while (!executor.getQueue().empty()) {
//          executor.advance(dt, true);
//          time = time + dt;
//          rosPublisher.publish(adapter->getState());

//        }

//    ros::Time t_end = ros::Time::now();
//    cout<<"end time : "<<time<<endl;
//    cout<<"Real time costs is : "<<t_end-t_start<<endl;
    return 0;
}
