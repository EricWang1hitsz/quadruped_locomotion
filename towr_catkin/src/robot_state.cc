#include <towr/robot_state.h>

#include <tf/tf.h>
namespace towr{

BaseState Robot_State::getInitialBaseStateInWorldFrame()
{
    return initial_base_;
}

bool Robot_State::setInitialBasePositionInWorldFrame(const Position position)
{
//    double x, y, z;
//    x = position.x;
//    y = position.y;
//    z = position.z;
//    initial_base_.lin.at(kPos).x() = x;
//    initial_base_.lin.at(kPos).y() = y;
//    initial_base_.lin.at(kPos).z() = z;
    return true;
}

bool Robot_State::setInitialBaseOrentationInWorldFrame(const RotationQuaternion rotation)
{
    geometry_msgs::Quaternion geometry_msgs_quaternion;
    kindr_ros::convertToRosGeometryMsg(rotation, geometry_msgs_quaternion);
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(geometry_msgs_quaternion, quaternion);

    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    initial_base_.ang.at(kPos).x() = roll;
    initial_base_.ang.at(kPos).y() = pitch;
    initial_base_.ang.at(kPos).z() = yaw;

    return true;
}
}
