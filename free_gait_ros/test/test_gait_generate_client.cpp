/********************************************************

@File test gait generate client

@Description

@Author  Eric Wang

@Date:   2020-1-7

@Association: Harbin Institute of Technology, Shenzhen.

*********************************************************/

#include "free_gait_ros/gait_generate_client.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gait_generate_client_test");
    ros::NodeHandle nh("~");
    GaitGenerateClient gait_generate_client_(nh, true);
    ros::Rate rate(0.5);
    free_gait::State robot_state_;
    robot_state_.setPoseBaseToWorld(Pose(Position(0, 0, 0), RotationQuaternion(1, 0, 0, 0)));// yaw = 0.

    while(ros::ok())
    {
        gait_generate_client_.footstepGeneration();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
