#include <ros/ros.h>
#include <free_gait_msgs/RobotState.h>

using namespace std;

free_gait_msgs::RobotState robot_state_;

void basePoseCallback(const free_gait_msgs::RobotStateConstPtr& msg)
{
    robot_state_.base_pose.pose.pose.position.x = msg->base_pose.pose.pose.position.x;
    robot_state_.base_pose.pose.pose.position.y = msg->base_pose.pose.pose.position.y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stand_up_test");
    ros::NodeHandle nh_;

    ros::Publisher robot_state_pub_ = nh_.advertise<free_gait_msgs::RobotState>("/desired_robot_state", 100);

    ros::Subscriber base_pose_sub_ = nh_.subscribe("/gazebo/robot_states", 100, basePoseCallback);

    ros::Rate loop_rate(100);

    float count = 0.0;

    while(ros::ok())
    {

        double z = 0.2133 * count + 0.137918;
        double v = -0.2878227 * pow(count - 1 , 2) + 0.299971;

        robot_state_.base_pose.pose.pose.position.z = z;
        robot_state_.base_pose.twist.twist.linear.z = v;
        robot_state_.lf_leg_mode.support_leg = true;
        robot_state_.rf_leg_mode.support_leg = true;
        robot_state_.lh_leg_mode.support_leg = true;
        robot_state_.rh_leg_mode.support_leg = true;

        robot_state_pub_.publish(robot_state_);

        ros::spinOnce();
        loop_rate.sleep();

        count = count + 0.01;

        ROS_INFO_STREAM("count now: " << count << std::endl);

        if(count > 1.95)
            ros::shutdown();

    }
}
