#include <robot_state_lcm_hardware_interface.hpp>

namespace laikago_ros_control {

using namespace laikago;

//! Must set control level here
Control control(LOWLEVEL);

RobotStateLcmHardwareInterface::RobotStateLcmHardwareInterface()
{
    //Publish IMU data for stata estimation.
    Imu_data_pub_ = node_handle_.advertise<sensor_msgs::Imu>("/imu/data", 100);
    //Publish joint state for showing on RVIZ.
    joint_state_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 100);
    ROS_INFO("Build Lcm Hardware Interface ");
}

RobotStateLcmHardwareInterface::~RobotStateLcmHardwareInterface()
{
    ROS_INFO("Lcm Hardware Interface Shutdown");
}

bool RobotStateLcmHardwareInterface::loadParameters(ros::NodeHandle &nh)
{
    if(!nh.getParam("/imu_topic_name", imu_topic_name_))
    {
        ROS_ERROR("Can not load IMU topic name ");
        return false;
    }
}

bool RobotStateLcmHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    ROS_INFO("Initializing RobotStateLcmHardwareInterface");
    node_handle_ = root_nh;

    //! WSHY: initial data of robot state
    robot_state_data_.name = "base_controller";
    robot_state_data_.position = position;
    robot_state_data_.orientation = orinetation;
    robot_state_data_.linear_velocity = linear_vel;
    robot_state_data_.angular_velocity = angular_vel;
    robot_state_data_.joint_position_read = pos_read;
    robot_state_data_.joint_position_write = pos_write;
    robot_state_data_.joint_velocity_read = vel_read;
    robot_state_data_.joint_velocity_write = vel_write;
    robot_state_data_.joint_effort_read = eff_read;
    robot_state_data_.joint_effort_write = eff_write;
    robot_state_data_.foot_contact = foot_contact;
    //! WSHY: registerhandle pass the data point to the hardwareResourseManager and then
    //! the read() method update data which the pointer points to or write() the
    //! updated commmand
    robot_state_interface_.registerHandle(hardware_interface::RobotStateHandle(robot_state_data_));

    std::string urdf_string;
    if(!root_nh.getParam("/robot_description", urdf_string))
      {
        ROS_ERROR("Failed to load urdf from robot_descriptions");
        return false;
      }
    std::string param_name = "/base_balance_controller/joints";
    if(!root_nh.getParam(param_name, joint_names_))
      {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << root_nh.getNamespace() << ").");
        return false;
      }

    n_dof_ = joint_names_.size();
    std::cout << "number is " << n_dof_ << std::endl;
    // Resize vectors to our DOF
    joint_names_.resize(n_dof_);
    joint_types_.resize(n_dof_);
    joint_lower_limits_.resize(n_dof_);
    joint_upper_limits_.resize(n_dof_);
    joint_effort_limits_.resize(n_dof_);
    joint_control_methods_.resize(n_dof_);
    last_joint_control_methods_.resize(n_dof_);
    pid_controllers_.resize(n_dof_);
    joint_position_.resize(n_dof_);
    joint_velocity_.resize(n_dof_);
    joint_effort_.resize(n_dof_);
    joint_effort_command_.resize(n_dof_);
    joint_position_command_.resize(n_dof_);
    joint_velocity_command_.resize(n_dof_);

    // Initialize values
    const ros::NodeHandle joint_limit_nh(root_nh);

    hardware_interface::JointHandle joint_handle;

    for(unsigned int j = 0;j<n_dof_;j++)
    {
        joint_position_[j] = 1.0;
        joint_velocity_[j] = 0.0;
        joint_effort_[j] = 1.0;  // N/m for continuous joints
        joint_effort_command_[j] = 0.0;
        joint_position_command_[j] = 0.0;
        joint_velocity_command_[j] = 0.0;

        //ROS_INFO("Joint name is ====================== %s", joint_names_[j].c_str());

        // Create joint state interface for all joints
        js_interface_.registerHandle(hardware_interface::JointStateHandle(
            joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));


        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_position_command_[j]);
        pj_interface_.registerHandle(joint_handle);

    }

    // Register interfaces
    //! WSHY: the controller pass the interface in it
    registerInterface(&js_interface_);
    registerInterface(&ej_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&vj_interface_);
    std::cout << "register" <<std::endl;
  //  registerInterface(&imu_interface_);
    registerInterface(&robot_state_interface_);

    // Initialize the emergency stop code.
    e_stop_active_ = false;
    last_e_stop_active_ = false;

    // Lcm subscribe robot state
    roslcm.SubscribeState();
    // update loop
    update_thread_ = boost::thread(boost::bind(&RobotStateLcmHardwareInterface::update_loop, this));
    // Set control level
    SendLowROS.levelFlag = laikago::LOWLEVEL;
    for(int i = 1; i < 13; i++)
    {
        SendLowROS.motorCmd[i].mode = 0x0A;
    }

    ROS_INFO("Successfully Initialize Lcm Hardware Interface");

    return true;
  }

void RobotStateLcmHardwareInterface::update_loop()
{
    while(ros::ok())
    {
        //ROS_INFO("update loop");
        boost::mutex::scoped_lock lock(mutex);
        //ROS_INFO("Receive robot state info ");
        roslcm.Recv();
        lock.unlock();
        roslcm.Get(RecvLowLCM);
        memcpy(&RecvLowROS, &RecvLowLCM, sizeof (laikago::LowState));
        sensor_msgs::Imu imu_msgs_;
        imu_msgs_ = getImuMsgs(RecvLowROS);
        Imu_data_pub_.publish(imu_msgs_);
        sensor_msgs::JointState joint_state_msgs_;
        joint_state_msgs_ = getJointStateMsgs(RecvLowROS);
        joint_state_pub_.publish(joint_state_msgs_);
        usleep(2000); // 500HZ
    }

}

const sensor_msgs::Imu RobotStateLcmHardwareInterface::getImuMsgs(laikago_msgs::LowState &low_state)
{
    sensor_msgs::Imu imu_msgs;
    //orientation data conversion
    imu_msgs.orientation.w = low_state.imu.quaternion[0];
    imu_msgs.orientation.x = low_state.imu.quaternion[1];
    imu_msgs.orientation.y = low_state.imu.quaternion[2];
    imu_msgs.orientation.z = low_state.imu.quaternion[3];
    //angular velocity data conversion
    imu_msgs.angular_velocity.x = low_state.imu.gyroscope[0];
    //ROS_INFO_STREAM("velocity x: " << imu_msgs.angular_velocity.x << std::endl);
    imu_msgs.angular_velocity.y = low_state.imu.gyroscope[1];
    //ROS_INFO_STREAM("velocity y: " << imu_msgs.angular_velocity.y << std::endl);
    imu_msgs.angular_velocity.z = low_state.imu.gyroscope[2];
    ROS_INFO_STREAM("velocity z: " << imu_msgs.angular_velocity.z << std::endl);
    //linear acceleration data conversion
    imu_msgs.linear_acceleration.x = low_state.imu.acceleration[0];
    imu_msgs.linear_acceleration.y = low_state.imu.acceleration[1];
    imu_msgs.linear_acceleration.z = low_state.imu.acceleration[2];

    return imu_msgs;
}

const sensor_msgs::JointState RobotStateLcmHardwareInterface::getJointStateMsgs(laikago_msgs::LowState &lowState)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.frame_id = "base";
    joint_state.name.resize(12);
    joint_state.name[0] = "FL_hip_joint";
    joint_state.name[1] = "FL_thigh_joint";
    joint_state.name[2] = "FL_calf_joint";
    joint_state.name[3] = "FR_hip_joint";
    joint_state.name[4] = "FR_thigh_joint";
    joint_state.name[5] = "FR_calf_joint";
    joint_state.name[6] = "RL_hip_joint";
    joint_state.name[7] = "RL_thigh_joint";
    joint_state.name[8] = "RL_calf_joint";
    joint_state.name[9] = "RR_hip_joint";
    joint_state.name[10] = "RR_thigh_joint";
    joint_state.name[11] = "RR_calf_joint";

    joint_state.position.resize(12);
    joint_state.position[0] = lowState.motorState[FL_0].position;
    joint_state.position[1] = lowState.motorState[FL_1].position;
    joint_state.position[2] = lowState.motorState[FL_2].position;
    joint_state.position[3] = lowState.motorState[FR_0].position;
    joint_state.position[4] = lowState.motorState[FR_1].position;
    joint_state.position[5] = lowState.motorState[FR_2].position;
    joint_state.position[6] = lowState.motorState[RL_0].position;
    joint_state.position[7] = lowState.motorState[RL_1].position;
    joint_state.position[8] = lowState.motorState[RL_2].position;
    joint_state.position[9] = lowState.motorState[RR_0].position;
    joint_state.position[10] = lowState.motorState[RR_1].position;
    joint_state.position[11] = lowState.motorState[RR_2].position;

    return joint_state;
}

void RobotStateLcmHardwareInterface::read(const ros::Time &time, const ros::Duration &period)
{
    //ROS_INFO("READ ONCE");
    boost::recursive_mutex::scoped_lock lock(r_mutex_);
    //roslcm.Recv();
    //roslcm.Get(RecvLowLCM);
    //memcpy(&RecvLowROS, &RecvLowLCM, sizeof (laikago::LowState));

    //roslcm.Get(RecvHighLCM);
    //memcpy(&RecvHighROS, &RecvHighLCM, sizeof (laikago::HighState));

    //ROS_INFO_STREAM("angular velocity x : " << RecvHighROS.imu.gyroscope[0]);
    //ROS_INFO_STREAM("angular velocity y : " << RecvHighROS.imu.gyroscope[1]);
    //ROS_INFO_STREAM("angular velocity z : " << RecvHighROS.imu.gyroscope[2]);


    //read joint position
    //printf("%f\n",  RecvLowROS.motorState[FL_2].position);
    robot_state_data_.joint_position_read[0] = RecvLowROS.motorState[FL_0].position;
    //ROS_WARN_STREAM("joint position: " << robot_state_data_.joint_position_read[1]);
    robot_state_data_.joint_position_read[1] = RecvLowROS.motorState[FL_1].position;
    robot_state_data_.joint_position_read[2] = RecvLowROS.motorState[FL_2].position;
    //ROS_WARN_STREAM("joint position: " << robot_state_data_.joint_position_read[2]);
    robot_state_data_.joint_position_read[3] = RecvLowROS.motorState[FR_0].position;
    robot_state_data_.joint_position_read[4] = RecvLowROS.motorState[FR_1].position;
    robot_state_data_.joint_position_read[5] = RecvLowROS.motorState[FR_2].position;
    robot_state_data_.joint_position_read[6] = RecvLowROS.motorState[RR_0].position;
    robot_state_data_.joint_position_read[7] = RecvLowROS.motorState[RR_1].position;
    robot_state_data_.joint_position_read[8] = RecvLowROS.motorState[RR_2].position;
    robot_state_data_.joint_position_read[9] = RecvLowROS.motorState[RL_0].position;
    robot_state_data_.joint_position_read[10] = RecvLowROS.motorState[RL_1].position;
    robot_state_data_.joint_position_read[11] = RecvLowROS.motorState[RL_2].position;
    //read joint velocity
    robot_state_data_.joint_velocity_read[0] = RecvLowROS.motorState[FL_0].velocity;
    robot_state_data_.joint_velocity_read[1] = RecvLowROS.motorState[FL_1].velocity;
    robot_state_data_.joint_velocity_read[2] = RecvLowROS.motorState[FL_2].velocity;
    robot_state_data_.joint_velocity_read[3] = RecvLowROS.motorState[FR_0].velocity;
    robot_state_data_.joint_velocity_read[4] = RecvLowROS.motorState[FR_1].velocity;
    robot_state_data_.joint_velocity_read[5] = RecvLowROS.motorState[FR_2].velocity;
    robot_state_data_.joint_velocity_read[6] = RecvLowROS.motorState[RR_0].velocity;
    robot_state_data_.joint_velocity_read[7] = RecvLowROS.motorState[RR_1].velocity;
    robot_state_data_.joint_velocity_read[8] = RecvLowROS.motorState[RR_2].velocity;
    robot_state_data_.joint_velocity_read[9] = RecvLowROS.motorState[RL_0].velocity;
    robot_state_data_.joint_velocity_read[10] = RecvLowROS.motorState[RL_1].velocity;
    robot_state_data_.joint_velocity_read[11] = RecvLowROS.motorState[RL_2].velocity;
    //read joint effort
    robot_state_data_.joint_effort_read[0] = RecvLowROS.motorState[FL_0].torque;
    robot_state_data_.joint_effort_read[1] = RecvLowROS.motorState[FL_1].torque;
    robot_state_data_.joint_effort_read[2] = RecvLowROS.motorState[FL_2].torque;
    robot_state_data_.joint_effort_read[3] = RecvLowROS.motorState[FR_0].torque;
    robot_state_data_.joint_effort_read[4] = RecvLowROS.motorState[FR_1].torque;
    robot_state_data_.joint_effort_read[5] = RecvLowROS.motorState[FR_2].torque;
    robot_state_data_.joint_effort_read[6] = RecvLowROS.motorState[RR_0].torque;
    robot_state_data_.joint_effort_read[7] = RecvLowROS.motorState[RR_1].torque;
    robot_state_data_.joint_effort_read[8] = RecvLowROS.motorState[RR_2].torque;
    robot_state_data_.joint_effort_read[9] = RecvLowROS.motorState[RL_0].torque;
    robot_state_data_.joint_effort_read[10] = RecvLowROS.motorState[RL_1].torque;
    robot_state_data_.joint_effort_read[11] = RecvLowROS.motorState[RL_2].torque;
    //read foot contact state
    //robot_state_data_.foot_contact[1] = RecvLowROS.footForce[FL_];


    lock.unlock();
}

void RobotStateLcmHardwareInterface::write(const ros::Time &time, const ros::Duration &period)
{
    //ROS_INFO("Write Once");
    boost::recursive_mutex::scoped_lock lock(r_mutex_);

//    SendLowROS.motorCmd[FL_0].position = 0;
//    SendLowROS.motorCmd[FL_0].velocity = 0;
//    SendLowROS.motorCmd[FL_0].torque = 0.65f;

//    SendLowROS.motorCmd[FL_1].position = 0.5;
//    SendLowROS.motorCmd[FL_1].velocity = 0;
//    SendLowROS.motorCmd[FL_1].torque = 0.0f;

//    SendLowROS.motorCmd[FL_2].position =  -1.1;
//    SendLowROS.motorCmd[FL_2].velocity = 0;
//    SendLowROS.motorCmd[FL_2].torque = 0.0f;

//    SendLowROS.motorCmd[FR_0].position = 0;
//    SendLowROS.motorCmd[FR_0].velocity = 0;
//    SendLowROS.motorCmd[FR_0].torque = 0.65f;

//    SendLowROS.motorCmd[FR_1].position = 0.5;
//    SendLowROS.motorCmd[FR_1].velocity = 0;
//    SendLowROS.motorCmd[FR_1].torque = 0.0f;

//    SendLowROS.motorCmd[FR_2].position =  -1.1;
//    SendLowROS.motorCmd[FR_2].velocity = 0;
//    SendLowROS.motorCmd[FR_2].torque = 0.0f;

//    SendLowROS.motorCmd[RL_0].position = 0;
//    SendLowROS.motorCmd[RL_0].velocity = 0;
//    SendLowROS.motorCmd[RL_0].torque = 0.65f;

//    SendLowROS.motorCmd[RL_1].position = 0.5;
//    SendLowROS.motorCmd[RL_1].velocity = 0;
//    SendLowROS.motorCmd[RL_1].torque = 0.0f;

//    SendLowROS.motorCmd[RL_2].position =  -1.1;
//    SendLowROS.motorCmd[RL_2].velocity = 0;
//    SendLowROS.motorCmd[RL_2].torque = 0.0f;

//    SendLowROS.motorCmd[RR_0].position = 0;
//    SendLowROS.motorCmd[RR_0].velocity = 0;
//    SendLowROS.motorCmd[RR_0].torque = 0.65f;

//    SendLowROS.motorCmd[RR_1].position = 0.5;
//    SendLowROS.motorCmd[RR_1].velocity = 0;
//    SendLowROS.motorCmd[RR_1].torque = 0.0f;

//    SendLowROS.motorCmd[RR_2].position =  -1.1;
//    SendLowROS.motorCmd[RR_2].velocity = 0;
//    SendLowROS.motorCmd[RR_2].torque = 0.0f;


//    memcpy(&SendLowLCM, &SendLowROS, sizeof(LowCmd));
//    roslcm.Send(SendLowLCM);

    lock.unlock();

    //boost::recursive_mutex::scoped_lock lock(r_mutex_);

}




}

PLUGINLIB_EXPORT_CLASS(laikago_ros_control::RobotStateLcmHardwareInterface, hardware_interface::RobotHW)

