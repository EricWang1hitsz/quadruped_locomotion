#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

class pathTrackingController
{

public:

    pathTrackingController();

    ~pathTrackingController();

    bool init(ros::NodeHandle nodehandle_);
    /*!
     * \brief fakeBasePoseCallback Get current base pose.
     * \param fakePose x, y, yaw.
     */
    void fakeBasePoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& fakePose);
    /*!
     * \brief footPrintPathCallback Get planned path needed to be tracked.
     * \param footPrintPath x, y, yaw;
     */
    void footPrintPathCallback(const geometry_msgs::PoseArrayPtr& footPrintPath);
    /*!
     * \brief publishCmdCallback Publish cmd velocity every @brief seconds.
     */
    void publishCmdCallback(const ros::TimerEvent&);
    /*!
     * \brief checkSegmentIsArrived Check robot is arrive at the segment.
     * \return
     */
    bool checkSegmentIsArrived();
    /*!
     * \brief updateVelocityCommand Cal base velocity command.
     */
    void updateVelocityCommand();
    /*!
     * \brief findNextNearestPoint Find next nearest point.
     */
    void findNextNearestPoint();
    /*!
     * \brief setFootprintPath Set planned path when unit test.
     * \param footPrintPath
     */
    void setFootprintPath(geometry_msgs::PoseArray footPrintPath);


private:

    ros::NodeHandle nh_;
    /*!
     * \brief fakePoseSubscriber_ Subscribe fake base pose from simpledog
     */
    ros::Subscriber fakePoseSubscriber_;
    /*!
     * \brief pathFootprintSubscriber_ Subscribe planned footprint path from quadruped motionplan.
     */
    ros::Subscriber pathFootprintSubscriber_;
    /*!
     * \brief cmdVelocityPublisher_ Publish x y yaw velocity command to free_gait.
     */
    ros::Publisher baseVelocityCommandPublisher_;
    /*!
     * \brief nextNearestPoint_ Next point when path tracking.
     */
    geometry_msgs::Point nextNearestPoint_;
    /*!
     * \brief currentPoint_ Current base pose point x y.
     */
    geometry_msgs::Point currentPoint_;
    /*!
     * \brief current_yaw Current robot haeading yaw.
     */
    double current_yaw;
    /*!
     * \brief footPrintPath_ Global path.
     */
    geometry_msgs::PoseArray footPrintPath_;
    /*!
     * \brief fakePose_ Current fake base pose.
     */
    geometry_msgs::PoseWithCovarianceStamped fakePose_;
    /*!
     * \brief cmd_ Base velocity command.
     */
    geometry_msgs::Twist cmd_;

    double disThreshold_;

    double yawThreshold_;
    /*!
     * \brief segmentIsArrived Flag robot arrive at the segment.
     */
    bool segmentIsArrived;
};
