#include <balance_controller/ros_controler/path_tracking_controller.hpp>

pathTrackingController::pathTrackingController()
    : disThreshold_(0.15),
      yawThreshold_(0.08) //5 angle degree
{
    ROS_INFO("PathTrackingController Class Initialized");
}

pathTrackingController::~pathTrackingController()
{
    ROS_INFO("PathTrackingController Class Shutdown");
}

bool pathTrackingController::init(ros::NodeHandle nodehandle_)
{
    ROS_INFO("Path Tracking Controller Initialized");
    fakePoseSubscriber_ = nh_.subscribe("/pose_pub_node/base_pose", 1, &pathTrackingController::fakeBasePoseCallback, this);
//    pathFootprintSubscriber_ = nh_.subscribe("", 1, &pathTrackingController::footPrintPathCallback, this);
    baseVelocityCommandPublisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void pathTrackingController::fakeBasePoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &fakePose)
{
    geometry_msgs::PoseWithCovarianceStamped pose_;
    pose_ = *fakePose;
    fakePose_ = pose_;
    currentPoint_.x = fakePose_.pose.pose.position.x;
    currentPoint_.y = fakePose_.pose.pose.position.y;
    tf::Quaternion quaternion_;
    tf::quaternionMsgToTF(fakePose_.pose.pose.orientation, quaternion_);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion_).getRPY(roll, pitch, yaw);
    current_yaw = yaw;
    ROS_INFO_ONCE("Get robot base fake pose");
}

void pathTrackingController::footPrintPathCallback(const geometry_msgs::PoseArrayPtr &footPrintPath)
{
    geometry_msgs::PoseArray poseArray_;
    poseArray_ = *footPrintPath;
    footPrintPath_ = poseArray_;
}

bool pathTrackingController::checkSegmentIsArrived()
{
    double currentX = currentPoint_.x;
    double currentY = currentPoint_.y;
    double segmentX = nextNearestPoint_.x;
    double segmentY = nextNearestPoint_.y;
    double disError, disError2;
    disError2 = (pow(segmentX, 2) + pow(segmentY, 2))
            - (pow(currentX, 2) + pow(currentY, 2));
    disError = pow(disError2, 0.5);
    if(disError <=  disThreshold_)
        segmentIsArrived = true;
    else
        segmentIsArrived = false;

    return true;
}

void pathTrackingController::findNextNearestPoint()
{
    geometry_msgs::Pose segment_;
    segment_ = footPrintPath_.poses.front();
    footPrintPath_.poses.pop_back();
    nextNearestPoint_.x = segment_.position.x;
    nextNearestPoint_.y = segment_.position.y;
}

void pathTrackingController::setFootprintPath(geometry_msgs::PoseArray footPrintPath)
{
    footPrintPath_ = footPrintPath;
    ROS_INFO("Set Footprint Path");
}

void pathTrackingController::updateVelocityCommand()
{
    double segment_yaw; // segment yaw
    double currentX = currentPoint_.x;
    double currentY = currentPoint_.y;
    double segmentX = nextNearestPoint_.x;
    double segmentY = nextNearestPoint_.y;

    segment_yaw = std::atan2((segmentY - currentY), (segmentX - currentX));
    ROS_INFO_STREAM_THROTTLE(5, "yaw error " << segment_yaw - current_yaw << std::endl);

    while(std::abs(segment_yaw - current_yaw) > yawThreshold_)
    {
        cmd_.linear.x = 0.0;
        cmd_.linear.y = 0.0; // robot stop when rotate.
        if(segment_yaw > current_yaw)
            cmd_.angular.z = 0.03;
        else
            cmd_.angular.z = -0.03;
        baseVelocityCommandPublisher_.publish(cmd_);
        sleep(1);
    }

    cmd_.linear.x = 0.05;
    cmd_.linear.y = 0.0;
    cmd_.angular.z = 0.0;

    baseVelocityCommandPublisher_.publish(cmd_);

    if(segmentIsArrived)
        findNextNearestPoint();
    ROS_INFO_STREAM_THROTTLE(5, "next segment " << nextNearestPoint_.x << nextNearestPoint_.y << std::endl);
}
