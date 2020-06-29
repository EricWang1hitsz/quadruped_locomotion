#include <iostream>
#include <ros/ros.h>
#include <robot_state_lcm_hardware_interface.hpp>
#include <controller_manager/controller_manager.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "laikago_controller_manager_test");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    laikago_ros_control::RobotStateLcmHardwareInterface hw_;
    bool init_success = hw_.init(nh,nh);
    ROS_INFO("TEST1");
    controller_manager::ControllerManager cm(&hw_, nh);
    ROS_INFO("TEST2");
    ros::Duration period(1.0/500); // 500Hz update rate

    while(ros::ok()){
        ROS_INFO("RUN ONCE");
        hw_.read(ros::Time::now(), period);
        cm.update(ros::Time::now(), period);
        hw_.write(ros::Time::now(), period);
        period.sleep();
    }

    spinner.stop();
return 0;
}
