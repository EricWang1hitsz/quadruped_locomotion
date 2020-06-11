#include <balance_controller/ros_controler/whole_body_controller.hpp>
#include <ros/ros.h>
#include <quadruped_model/common/typedefs.hpp>
#include <free_gait_core/TypeDefs.hpp>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "whole_body_controller_test");
    ros::NodeHandle nh("~");
    Position base_pos(0, 0, 0);
    RotationQuaternion base_w(0, 0, 0, 1);
    JointPositions joint_pos;
    joint_pos << 0.0, 0.75, -1.5,
                 0.0, -0.75, 1.5,
                 0.0, 0.75, -1.5,
                 0.0, -0.75, 1.5;
    std::cout << joint_pos << std::endl;

    LinearVelocity base_vel(0, 0, 0);
    LocalAngularVelocity base_vel_w(0, 0, 0);

    JointVelocities joint_vel;
    joint_vel << 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0;
    std:: cout << joint_vel << std::endl;

    std::shared_ptr<free_gait::State> state_;
    state_.reset(new free_gait::State);
    state_->setPositionWorldToBaseInWorldFrame(base_pos);
    state_->setOrientationBaseToWorld(base_w);
    state_->setJointPositions(joint_pos);
    ROS_INFO("test  1  here");
    state_->setLinearVelocityBaseInWorldFrame(base_vel);
    state_->setAngularVelocityBaseInBaseFrame(base_vel_w);
    state_->setJointVelocities(joint_vel);
    ROS_INFO("test here");

    // swing leg foot acceleration
//    LinearAcceleration foot_acceleration;
//    foot_acceleration << 1, 1, 1;
//    state_->setTargetFootAccelerationInBaseForLimb(foot_acceleration, free_gait::LimbEnum::LF_LEG);
//    state_->setTargetFootAccelerationInBaseForLimb(foot_acceleration, free_gait::LimbEnum::RF_LEG);
//    state_->setTargetFootAccelerationInBaseForLimb(foot_acceleration, free_gait::LimbEnum::LH_LEG);
//    state_->setTargetFootAccelerationInBaseForLimb(foot_acceleration, free_gait::LimbEnum::RH_LEG);

    std::cout << " test here 1 " << std::endl;
    WholeBodyController wbc(state_);

    LinearAcceleration reference_acc;
    reference_acc << 1, 1, 1;
    AngularAcceleration reference_w;
    reference_w << 0.0, 0.0, 0.0;



//    LimbSelector limbset_;
    // quadruped robot foot order
//    limbset_.push_back("lf_foot");
//    limbset_.push_back("rf_foot");
//    limbset_.push_back("lh_foot");
//    limbset_.push_back("rh_foot");
    // HyQ foot order
//    limbset_.push_back("lf_foot");
//    limbset_.push_back("lh_foot");
//    limbset_.push_back("rf_foot");
//    limbset_.push_back("rh_foot");

//    // tset joint space inertial matrix
//    std::cout << "--------------------joint space inertial matrix----------------" << std::endl;
//    Eigen::MatrixXd joint_space_inertial_matrix;
//    joint_space_inertial_matrix = wbc.computeJointSpaceInertialMatrix(base_pos, base_w, joint_pos);
//    std::cout << joint_space_inertial_matrix << std::endl;
//    std::cout << "-------------------------inertial base matrix----------------" << std::endl;
//    std::cout << wbc.getInertialBaseMatrix() << std::endl;
//    std::cout << "-------------------------inertial joint matrix----------------" << std::endl;
//    std::cout << wbc.getInertialJointMatrix() << std::endl;

//    // test nonlinear effect force matrix
//    std::cout << "--------------------nonlinear effect force matrix----------------" << std::endl;
//    Eigen::VectorXd nonlinear_effect_force;
//    nonlinear_effect_force = wbc.computeNonlinearEffectsForce(base_pos, base_w, base_vel, base_vel_w, joint_pos, joint_vel);
//    std::cout << nonlinear_effect_force << std::endl;
//    std::cout << "--------------------nonlinear base effect matrix----------------" << std::endl;
//    std::cout << wbc.getNonlinearBaseEffectsForce() << std::endl;
//    std::cout << "--------------------nonlinear joint effect matrix----------------" << std::endl;
//    std::cout << wbc.getNonlinearJointEffectsForce() << std::endl;

//    // tset jacobian matrix
//    std::cout << "-----------------------jacobian matrix---------------------------" << std::endl;
//    Eigen::MatrixXd jacobian;
//    jacobian = wbc.computeJacobian(base_pos, base_w, joint_pos, limbset_);
//    std::cout << jacobian << std::endl;
    wbc.prepareLegLoading();
    wbc.prepareOptimazation(reference_acc, reference_w);
    wbc.addPhysicalConsistencyConstraints();
    wbc.addStanceAndSwingLegConstraints();
    wbc.addFrictionConeConstraints();
    wbc.addTorqueLimitConstraints();
    wbc.solveOptimazation();
    JointTorques joint_torque;
    joint_torque = wbc.getFeedforwardJointTorque();
    std::cout << "-------------------------Feed-Forward Joint Torque--------------------" << std::endl;
    std::cout << joint_torque << std::endl;

}
