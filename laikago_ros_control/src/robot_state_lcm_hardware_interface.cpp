#include <robot_state_lcm_hardware_interface.hpp>

namespace laikago_ros_control {

using namespace laikago;

//! Must set control level here
Control control(LOWLEVEL);

RobotStateLcmHardwareInterface::RobotStateLcmHardwareInterface()
{
    ROS_INFO("Build Lcm Hardware Interface ");
}

RobotStateLcmHardwareInterface::~RobotStateLcmHardwareInterface()
{
    ROS_INFO("Lcm Hardware Interface Shutdown");
}

bool RobotStateLcmHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    ROS_INFO("Initializing RobotStateLcmHardwareInterface");
    node_handle_ = root_nh;

    //! WSHY: initial data of robot state
    robot_state_data_.name = "base_control";
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

    //TODO why controller can find the handle after add these two code?
//    hardware_interface::RobotStateHandle robot_state_handle_;
//    robot_state_handle_ = robot_state_interface_.getHandle("base_control");

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
        usleep(2000); // 500HZ
    }

}

void RobotStateLcmHardwareInterface::read(const ros::Time &time, const ros::Duration &period)
{
    //ROS_INFO("READ ONCE");
    boost::recursive_mutex::scoped_lock lock(r_mutex_);
    //roslcm.Recv();
    roslcm.Get(RecvLowLCM);
    memcpy(&RecvLowROS, &RecvLowLCM, sizeof (laikago::LowState));

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
    ROS_WARN_STREAM("joint position: " << robot_state_data_.joint_position_read[2]);
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

    SendLowROS.motorCmd[FR_0].torque = -0.65f;
    SendLowROS.motorCmd[FL_0].torque = +0.65f;
    SendLowROS.motorCmd[RR_0].torque = -0.65f;
    SendLowROS.motorCmd[RL_0].torque = +0.65f;

    SendLowROS.motorCmd[FR_1].torque = 0.0f;
    SendLowROS.motorCmd[FL_1].torque = 0.0f;
    SendLowROS.motorCmd[RR_1].torque = 0.0f;
    SendLowROS.motorCmd[RL_1].torque = 0.0f;


    memcpy(&SendLowLCM, &SendLowROS, sizeof(LowCmd));
    roslcm.Send(SendLowLCM);

//    ej_sat_interface_.enforceLimits(period);
//    ej_limits_interface_.enforceLimits(period);
//    pj_sat_interface_.enforceLimits(period);
//    pj_limits_interface_.enforceLimits(period);
//    vj_sat_interface_.enforceLimits(period);
//    vj_limits_interface_.enforceLimits(period);

    //boost::recursive_mutex::scoped_lock lock(r_mutex_);

}




}

PLUGINLIB_EXPORT_CLASS(laikago_ros_control::RobotStateLcmHardwareInterface, hardware_interface::RobotHW)

