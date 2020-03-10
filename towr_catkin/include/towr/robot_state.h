#pragma once

#include <geometry_msgs/PoseWithCovariance.h>
#include <towr/variables/state.h>
#include <quadruped_model/common/typedefs.hpp>
#include <kindr_ros/RosGeometryMsgRotation.hpp>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>

using namespace romo;

namespace towr {

class Robot_State
{
public:

    /**
     * @brief getInitialBaseStateInWorldFrame
     * @return Initial base state.
     */
    BaseState getInitialBaseStateInWorldFrame();
    /**
     * @brief setInitialBasePositionInWorldFrame
     * @param position Base position from Gazebo.
     * @return True.
     */
    bool setInitialBasePositionInWorldFrame(const Position position);

    /**
     * @brief setInitialBaseOrentationInWorldFrame
     * @param rotation Base orentation from Gazebo.
     * @return True.
     */
    bool setInitialBaseOrentationInWorldFrame(const RotationQuaternion rotation);


private:

    BaseState initial_base_;


};
}
