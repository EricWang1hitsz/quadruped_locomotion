#include <iostream>
#include <ros/ros.h>
#include <robot_state_lcm_hardware_interface.hpp>
#include <controller_manager/controller_manager.h>
#include <robot_state_interface.hpp>

class LaikagoControllerManager
{
public:

    LaikagoControllerManager(const ros::NodeHandle& nh)
        : nh_(nh)
    {
        LCM_HW_.reset(new laikago_ros_control::RobotStateLcmHardwareInterface);
        LCM_HW_->init(nh_, nh_);
        controller_manager_.reset(new controller_manager::ControllerManager(LCM_HW_.get()));
        double loop_hz = 500;
        ros::Duration update_freq = ros::Duration(1.0 / loop_hz);

        control_loop_ = nh_.createTimer(update_freq, &LaikagoControllerManager::update, this);
    }

    void update(const ros::TimerEvent&)
    {
        ros::Time time = ros::Time::now();
        ros::Duration period(0.0020);

        LCM_HW_->read(time, period);
        controller_manager_->update(time, period);
        LCM_HW_->write(time, period);
    }


private:

    ros::NodeHandle nh_;
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    std::shared_ptr<laikago_ros_control::RobotStateLcmHardwareInterface> LCM_HW_;

    ros::Timer control_loop_;


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laikago_controller_manager");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(2);
    LaikagoControllerManager controller_manager(nh);

    spinner.spin();

    return 0;
}
