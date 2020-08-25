#include <balance_controller/ros_controler/path_tracking_controller.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathTrackingController");
    ros::NodeHandle nodehandle_;

    ros::Rate rate_(1);

    pathTrackingController track_;
    track_.init(nodehandle_);

    geometry_msgs::PoseArray msgs_;
    msgs_.header.stamp = ros::Time();
    msgs_.header.frame_id = "odom";
    geometry_msgs::Pose msg_1;
    msg_1.position.x = 0.0;
    msg_1.position.y = 0.0;
    msg_1.position.z = 0.5;
    msg_1.orientation.x = 0.0;
    msg_1.orientation.y = 0.0;
    msg_1.orientation.z = 0.0;
    msg_1.orientation.w = 1.0;
    geometry_msgs::Pose msg_2;
    msg_2.position.x = 0.0;
    msg_2.position.y = -1.0;
    msg_2.position.z = 0.5;
    msg_2.orientation.x = 0.0;
    msg_2.orientation.y = 0.0;
    msg_2.orientation.z = 0.0;
    msg_2.orientation.w = 1.0;
    geometry_msgs::Pose msg_3;
    msg_3.position.x = 0.0;
    msg_3.position.y = -2.0;
    msg_3.position.z = 0.5;
    msg_3.orientation.x = 0.0;
    msg_3.orientation.y = 0.0;
    msg_3.orientation.z = 0.0;
    msg_3.orientation.w = 1.0;
    geometry_msgs::Pose msg_4;
    msg_4.position.x = 1.0;
    msg_4.position.y = -2.0;
    msg_4.position.z = 0.5;
    msg_4.orientation.x = 0.0;
    msg_4.orientation.y = 0.0;
    msg_4.orientation.z = 0.0;
    msg_4.orientation.w = 1.0;
//    msgs_.poses.push_back(msg_1);
    msgs_.poses.push_back(msg_2);
    msgs_.poses.push_back(msg_3);
    msgs_.poses.push_back(msg_4);

    track_.setFootprintPath(msgs_);

    while(ros::ok())
    {
        track_.updateVelocityCommand();

        ros::spinOnce();

        rate_.sleep();
    }

    return 0;


}
